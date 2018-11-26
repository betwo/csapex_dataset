#include "rgbd_id_importer.hpp"

#include <csapex/model/node_modifier.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/msg/end_of_sequence_message.h>
#include <csapex/msg/io.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/signal/event.h>

#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>
#include <csapex_point_cloud/msg/indeces_message.h>

#include <map>

#include <boost/regex.hpp>

#include <pcl/io/file_io.h>

using namespace csapex;
using namespace dataset;
using namespace biwi;

using namespace csapex::connection_types;
namespace bfs = boost::filesystem;

CSAPEX_REGISTER_CLASS(csapex::dataset::biwi::RGBDIDImporter, csapex::Node)

RGBDIDImporter::RGBDIDImporter() :
    playing_(false)
{
}

void RGBDIDImporter::setup(NodeModifier &node_modifier)
{
    output_pointcloud_      = node_modifier.addOutput<PointCloudMessage>("Pointcloud");
    output_mask_            = node_modifier.addOutput<CvMatMessage>("Mask");
    output_indices_         = node_modifier.addOutput<GenericVectorMessage, pcl::PointIndices>("Clusters");
    output_indices_msgs_    = node_modifier.addOutput<GenericVectorMessage, pcl::PointIndices>("Clusters");
    output_rgb_             = node_modifier.addOutput<CvMatMessage>("RGB");
    output_depth_           = node_modifier.addOutput<CvMatMessage>("Depth");
    event_finished_         = node_modifier.addEvent("finished");
}

void RGBDIDImporter::setupParameters(Parameterizable &parameters)
{
    auto path    = param::factory::declareDirectoryInputPath("path", "").build();

    std::vector<std::string> types = {{"still", "walking", "still+walking"}};

    auto reload          = param::factory::declareTrigger("reload");
    auto start_play      = param::factory::declareTrigger("start play");
    auto stop_play       = param::factory::declareTrigger("stop play");
    auto start_instantly = param::factory::declareBool("start instantly", false);
    auto ignore_errors   = param::factory::declareBool("ignore errors", false);
    auto play_progress   = param::factory::declareOutputProgress("played").build<param::OutputProgressParameter>();
    auto current_frame   = param::factory::declareOutputText("current frame").build<param::OutputTextParameter>();
    auto types_to_play   = param::factory::declareParameterStringSet("type", types, "still+walking");
    auto cluster_depth_deviation = param::factory::declareRange("cluster depth deviation", 0.0, 1.0, 0.1, 0.001);
    auto cluster_depth_maximum   = param::factory::declareRange("cluster depth maximum", 0.0, 5.0, 0.1, 0.001);

    auto play_only = [this]() { return playing_; };
    auto no_play_only = [this]() { return data_.size() > 0 && !playing_; };

    parameters.addParameter(path, [this](param::Parameter* param) { import(param->as<std::string>()); });
    parameters.addParameter(types_to_play, types_to_play_);
    parameters.addParameter(cluster_depth_deviation, depth_deviation_);
    parameters.addParameter(cluster_depth_maximum, cluster_depth_maximum_);
    parameters.addConditionalParameter(reload, no_play_only, [this, path](param::Parameter* param) { import(path->as<std::string>()); });
    parameters.addParameter(start_instantly,
                            [this](param::Parameter* param)
    {
        param_start_instantly_ = param->as<bool>();
        if (param_start_instantly_ && data_.size() > 0)
        {
            startPlay();
            triggerStartPlayEvent();
        }
    });
    parameters.addParameter(ignore_errors, ignore_errors_);

    parameters.addConditionalParameter(start_play, no_play_only, [this](param::Parameter*) { if ( data_.size() > 0) startPlay(); });
    parameters.addConditionalParameter(stop_play, play_only,
                                       [this](param::Parameter*)
    {
        playing_ = false;
        ainfo << "Stop playing" << std::endl;;
    });
    parameters.addConditionalParameter(play_progress, play_only);
    parameters.addConditionalParameter(current_frame, play_only);

    play_progress_ = play_progress;
    current_frame_ = current_frame;

}

bool RGBDIDImporter::canProcess() const
{
    return data_.size() > 0 && playing_;
}

void RGBDIDImporter::process()
{
    if (data_iterator_ != data_end_)
    {
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto micros = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();

        auto& entry          = *data_iterator_;
        auto  pointcloud_msg = std::make_shared<PointCloudMessage>("depth_frame", micros);
        auto  rgb_msg        = std::make_shared<CvMatMessage>(csapex::enc::bgr,     "rgb_frame", micros);
        auto  depth_msg      = std::make_shared<CvMatMessage>(csapex::enc::depth_f, "depth_frame", micros);
        auto  mask_msg       = std::make_shared<CvMatMessage>(csapex::enc::mono,    "depth_frame", micros);
        auto  indices_msg    = std::make_shared<std::vector<pcl::PointIndices>>();
        auto  incides_msgs   = std::make_shared<std::vector<PointIndicesMessage>>();

        cv::Mat mask        = cv::imread(entry.path_mask.string(),  CV_LOAD_IMAGE_UNCHANGED);
        cv::Mat depth_image = cv::imread(entry.path_depth.string(), CV_LOAD_IMAGE_UNCHANGED);
        cv::Mat rgb_image   = cv::imread(entry.path_rgb.string(),   CV_LOAD_IMAGE_UNCHANGED);

        const static float d_cx     = 319.5f;
        const static float d_cy     = 239.5f;
        const static float d_fx     = 575.8f;
        const static float d_fy     = 575.8f;
        const static int   d_width  = 640;
        const static int   d_height = 480;
        const static float rgb_cx     = 319.5f * 2.f;
        const static float rgb_cy     = 239.5f * 2.f;
        const static float rgb_fx     = 525.0f * 2.f;
        const static float rgb_fy     = 525.0f * 2.f;
        const static int   rgb_height = 960;
        const static int   rgb_width  = 1280;
        const static float tx         = 0.025f;


        bool error = mask.rows != d_height;
        if(error && !ignore_errors_) {
            throw std::runtime_error("Mask image '" + entry.path_mask.string() + "' height does not match!");
        }
        error = mask.cols != d_width;
        if(error && !ignore_errors_) {
            throw std::runtime_error("Mask image '" + entry.path_mask.string() + "' width does not match!");
        }
        error = depth_image.rows != d_height;
        if(error && !ignore_errors_) {
            throw std::runtime_error("Depth image '" + entry.path_depth.string() + "' height does not match!");
        }
        error = depth_image.cols != d_width;
        if(error && !ignore_errors_) {
            throw std::runtime_error("Depth image '" + entry.path_depth.string() + "' width does not match!");
        }
        error = rgb_image.rows != rgb_height;
        if(error && !ignore_errors_) {
            throw std::runtime_error("RGB image '" + entry.path_rgb.string() + "' height does not match!");
        }
        error = rgb_image.cols != rgb_width;
        if(error && !ignore_errors_) {
            throw std::runtime_error("RGB image '" + entry.path_rgb.string() + "' width does not match!");
        }
        auto depth = [&depth_image, this](const int y, const int x){
            double d = depth_image.at<ushort>(y,x) * 1e-3f;
            return d <= cluster_depth_maximum_ || cluster_depth_maximum_ == 0.0 ? d : std::numeric_limits<double>::quiet_NaN();
        };

        if(error) {
            std::cout << "error" << std::endl;
        }

        /// TAKEN FROM THE INFORMATION FILE OF BIWI ...
        //  RGB intrinsics matrix: [525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0]
        //  Depth intrinsics matrix: [575.8, 0.0, 319.5, 0.0, 575.8, 239.5, 0.0, 0.0, 1.0]
        //  RGB-Depth extrinsic parameters: T = [0.025 0.0 0.0], R = [0.0 0.0 0.0].
        if(!error) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr points(new pcl::PointCloud<pcl::PointXYZRGB>(640, 480));
            for(int i = 0 ; i < d_height; ++i) {
                for(int j = 0 ; j < d_width ; ++j) {
                    const float d = depth(i,j);
                    pcl::PointXYZRGB &p = points->at(j,i);

                    if(std::isnormal(d)) {
                        p.x = (static_cast<float>(j) - d_cx) * d / d_fx;
                        p.y = (static_cast<float>(i) - d_cy) * d / d_fy;
                        p.z = d;

                        /// get the color image point
                        const int rgb_x = static_cast<int>(((p.x + tx) * rgb_fx) / d + rgb_cx);
                        const int rgb_y = static_cast<int>(( p.y       * rgb_fy) / d + rgb_cy);
                        if(rgb_x >= 0 && rgb_x < rgb_width &&
                                rgb_y >= 0 && rgb_y < rgb_height) {
                            const auto bgr = rgb_image.at<cv::Vec3b>(rgb_y, rgb_x);
                            p.b = bgr[0];
                            p.g = bgr[1];
                            p.r = bgr[2];
                        }
                    } else {
                        p.x = std::numeric_limits<float>::quiet_NaN();
                        p.y = std::numeric_limits<float>::quiet_NaN();
                        p.z = std::numeric_limits<float>::quiet_NaN();
                    }


                }
            }

            std::map<uchar, pcl::PointIndices> clusters;
            std::map<uchar, double>            mean_depths;

            for(int i = 0 ; i < d_height ; ++i) {
                for(int j = 0 ; j < d_width ; ++j) {
                    const uchar id = mask.at<uchar>(i,j);
                    if(id > 0) {
                        const double d = depth(i,j);
                        if(std::isnormal(d)) {
                            clusters[id].indices.emplace_back(i * d_width + j);
                            mean_depths[id] += depth(i,j);
                        }
                    }
                }
            }

            for(const auto &c : clusters) {
                indices_msg->emplace_back(pcl::PointIndices());
                pcl::PointIndices &indices = indices_msg->back();
                double mean_depth = mean_depths[c.first] / static_cast<double>(c.second.indices.size());

                auto in_range = [mean_depth, this](const double d) {
                    return std::fabs(d - mean_depth) <= depth_deviation_;
                };

                for(const auto i : c.second.indices) {
                    if(in_range(depth_image.at<ushort>(i) * 1e-3f)) {
                        indices.indices.emplace_back(i);
                    }
                }
            }

            for(const auto &i : *indices_msg) {
                PointIndicesMessage im;
                im.value.reset(new pcl::PointIndices(i));
                incides_msgs->emplace_back(im);
            }

            pointcloud_msg->value = points;
            rgb_msg->value   = rgb_image.clone();
            depth_msg->value = depth_image.clone();
            mask_msg->value  = mask.clone();

            msg::publish(output_pointcloud_, pointcloud_msg);
            msg::publish(output_rgb_, rgb_msg);
            msg::publish(output_depth_, depth_msg);
            msg::publish(output_mask_, mask_msg);
            msg::publish<GenericVectorMessage, pcl::PointIndices>(output_indices_, indices_msg);
            msg::publish<GenericVectorMessage, PointIndicesMessage>(output_indices_msgs_, incides_msgs);
            current_frame_->set("Frame ID: " + entry.id);
        }

        const double percent = 1.0 - static_cast<double>(std::distance(data_iterator_, data_end_)) /
                static_cast<double>(data_entries_to_play_);

        play_progress_->set<int>(static_cast<int>(percent * 100));

        //        play_progress_->advanceProgress();
        ++data_iterator_;
    }
    else
    {
        playing_ = false;
        auto end = makeEmpty<EndOfSequenceMessage>();
        msg::publish(output_pointcloud_, end);
        msg::publish(output_mask_,       end);
        msg::publish(output_rgb_,        end);
        msg::publish(output_indices_msgs_, end);
        msg::publish(output_indices_, end);
        msg::trigger(event_finished_);
    }
}

void RGBDIDImporter::import(const boost::filesystem::path &path)
{
    if(path.empty())
        return;

    if(!bfs::exists(path))
        throw std::runtime_error("Path '" + path.string() + "' does not exist!");

    playing_ = false;

    data_.clear();

    std::map<std::string, MetaEntry> still;
    std::map<std::string, MetaEntry> walking;

    const bfs::recursive_directory_iterator end;
    bfs::recursive_directory_iterator dir_iter( path );
    while(dir_iter != end) {
        const bfs::path current_path = dir_iter->path();
        if(bfs::is_regular_file(current_path)) {
            const std::string file = current_path.filename().string();
            const std::size_t delim_pos = file.find_first_of('-');
            const std::string id = file.substr(0,delim_pos);
            const char type = file[delim_pos+1];

            bool w = current_path.string().find("Walking") != current_path.string().npos;
            auto &entry = w ? walking[id] : still[id];
            switch(type) {
            case 'a':
                /// rgb image
                entry.path_rgb = current_path;
                break;
            case 'b':
                /// depth image
                entry.path_depth = current_path;
                break;
            case 'c':
                /// depth label map
                entry.path_mask = current_path;
                break;
            default:
                /// ignore
                break;
            }
        }
        ++dir_iter;
    }

    for(const auto &e : still) {
        MetaEntry m = e.second;
        m.id = e.first;
        data_.emplace_back(m);
    }
    for(const auto &e : walking) {
        MetaEntry m = e.second;
        m.id = e.first;
        data_.emplace_back(m);
    };
    data_walking_begin_ = data_.begin() + still.size() + 1;

    ainfo << "Loaded " << data_.size() << " entries! \n";
}

void RGBDIDImporter::startPlay()
{
    /// choose which parts to play
    if(types_to_play_ == "still") {
        data_iterator_  = data_.begin();
        data_end_       = data_walking_begin_;
    } else if(types_to_play_ == "walking") {
        data_iterator_  = data_walking_begin_;
        data_end_       = data_.end();
    } else {
        data_iterator_  = data_.begin();
        data_end_       = data_.end();
    }

    data_entries_to_play_ = std::distance(data_iterator_, data_end_);

    playing_ = true;
    play_progress_->set<int>(0);
    ainfo << "Start playing " + std::to_string(data_entries_to_play_) + " entries..." << std::endl;
}

void RGBDIDImporter::triggerStartPlayEvent()
{
    const auto& events = node_modifier_->getEvents();
    auto itr = std::find_if(events.begin(), events.end(),
                            [](const EventPtr& event) { return event->getLabel() == "start play"; });
    if (itr == events.end())
        throw std::runtime_error("Unable to find 'start play' event");

    auto& ptr = (*itr);
    ptr->trigger();
}
