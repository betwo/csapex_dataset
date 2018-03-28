#include "rgbd_id_importer.hpp"

#include <csapex/model/node_modifier.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/msg/end_of_sequence_message.h>
#include <csapex/msg/io.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/signal/event.h>

#include <csapex_opencv/cv_mat_message.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>

#include <map>

#include <boost/regex.hpp>

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
    output_pointcloud_ = node_modifier.addOutput<PointCloudMessage>("Pointcloud");
    output_mask_       = node_modifier.addOutput<CvMatMessage>("Mask");
    output_rgb_        = node_modifier.addOutput<CvMatMessage>("RGB");
    output_depth_      = node_modifier.addOutput<CvMatMessage>("Depth");
    event_finished_    = node_modifier.addEvent("finished");
}

void RGBDIDImporter::setupParameters(Parameterizable &parameters)
{
    auto path    = param::ParameterFactory::declareDirectoryInputPath("path", "").build();

    auto reload          = param::ParameterFactory::declareTrigger("reload");
    auto start_play      = param::ParameterFactory::declareTrigger("start play");
    auto stop_play       = param::ParameterFactory::declareTrigger("stop play");
    auto start_instantly = param::ParameterFactory::declareBool("start instantly", false);
    auto play_progress   = param::ParameterFactory::declareOutputProgress("played").build<param::OutputProgressParameter>();
    auto current_frame   = param::ParameterFactory::declareOutputText("current frame").build<param::OutputTextParameter>();

    auto play_only = [this]() { return playing_; };
    auto no_play_only = [this]() { return data_.size() > 0 && !playing_; };

    parameters.addParameter(path, [this](param::Parameter* param) { import(param->as<std::string>()); });
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
    if (data_iterator_ != data_.end())
    {
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto micros = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();

        auto& entry          = *data_iterator_;
        auto  pointcloud_msg = std::make_shared<PointCloudMessage>("depth_frame", micros);
        auto  rgb_msg        = std::make_shared<CvMatMessage>(csapex::enc::bgr, "rgb_frame", micros);
        auto  depth_msg      = std::make_shared<CvMatMessage>(csapex::enc::depth_f, "depth_frame", micros);
        auto  mask_msg       = std::make_shared<CvMatMessage>(csapex::enc::mono, "depth_frame", micros);

        cv::Mat mask        = cv::imread(entry.path_mask.string(),  CV_LOAD_IMAGE_UNCHANGED);
        cv::Mat depth_image = cv::imread(entry.path_depth.string(), CV_LOAD_IMAGE_UNCHANGED);
        cv::Mat rgb_image   = cv::imread(entry.path_rgb.string(),   CV_LOAD_IMAGE_UNCHANGED);

        const static float d_cx   = 319.5f;
        const static float d_cy   = 239.5f;
        const static float d_fx   = 575.8f;
        const static float d_fy   = 575.8f;
        const static int d_width   = 640;
        const static int d_height  = 480;
        const static float rgb_cx = 319.5f;
        const static float rgb_cy = 239.5f;
        const static float rgb_fx = 525.0f;
        const static float rgb_fy = 525.0f;
        const static int rgb_height = 960;
        const static int rgb_width  = 1280;
        const static float tx = 0.025f;

        if(mask.rows != d_height) {
            throw std::runtime_error("Mask image height does not match!");
        }
        if(mask.cols != d_width) {
            throw std::runtime_error("Mask image width does not match!");
        }
        if(depth_image.rows != d_height) {
            throw std::runtime_error("Depth image height does not match!");
        }
        if(depth_image.cols != d_width) {
            throw std::runtime_error("Depth image width does not match!");
        }
        if(rgb_image.rows != rgb_height) {
            throw std::runtime_error("RGB image height does not match!");
        }
        if(rgb_image.cols != rgb_width) {
            throw std::runtime_error("RGB image width does not match!");
        }

        auto depth = [&depth_image](const int y, const int x){
            return depth_image.at<ushort>(y,x) * 1000.0f;
        };

        /// TAKEN FROM THE INFORMATION FILE OF BIWI ...
        //  RGB intrinsics matrix: [525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0]
        //  Depth intrinsics matrix: [575.8, 0.0, 319.5, 0.0, 575.8, 239.5, 0.0, 0.0, 1.0]
        //  RGB-Depth extrinsic parameters: T = [0.025 0.0 0.0], R = [0.0 0.0 0.0].

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr points(new pcl::PointCloud<pcl::PointXYZRGB>(640, 480));
        for(int i = 0 ; i < d_height; ++i) {
            for(int j = 0 ; j < d_width ; ++j) {
                const float d = depth(i,j);
                pcl::PointXYZRGB &p = points->at(j,i);
                if(std::isnormal(d)) {
                    p.x = (static_cast<float>(j) - d_cx) * d / d_fx;
                    p.y = (static_cast<float>(i) - d_cy) * d / d_fy;
                    p.z = d;

                    std::cout << p.x << " " << p.y << " " << p.z << "\n";

                    /// get the color image point
                    const int rgb_x = static_cast<int>(((p.x + tx) * rgb_fx) / d + rgb_cx);
                    const int rgb_y = static_cast<int>(( p.y       * rgb_fy) / d + rgb_cy);
                    if(rgb_x >= 0 && rgb_x < rgb_width &&
                            rgb_y >= 0 && rgb_y < rgb_height) {
                        const auto bgr = rgb_image.at<cv::Vec3b>(rgb_y, rgb_x);
                        p.b = bgr[0];
                        p.r = bgr[1];
                        p.g = bgr[2];
                    }
                } else {
                    p.x = std::numeric_limits<float>::quiet_NaN();
                    p.y = std::numeric_limits<float>::quiet_NaN();
                    p.z = std::numeric_limits<float>::quiet_NaN();
                }


            }
        }

        pointcloud_msg->value = points;
        rgb_msg->value   = rgb_image.clone();
        depth_msg->value = depth_image.clone();
        mask_msg->value  = mask.clone();

        msg::publish(output_pointcloud_, pointcloud_msg);
        msg::publish(output_rgb_, rgb_msg);
        msg::publish(output_depth_, depth_msg);
        msg::publish(output_mask_, mask_msg);

        play_progress_->advanceProgress();
        current_frame_->set("Frame ID: " + entry.id);
        ++data_iterator_;
    }
    else
    {
        playing_ = false;
        auto end = makeEmpty<EndOfSequenceMessage>();
        msg::publish(output_pointcloud_, end);
        msg::publish(output_mask_,       end);
        msg::publish(output_rgb_,        end);
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

    std::map<std::string, MetaEntry> content;

    const bfs::recursive_directory_iterator end;
    bfs::recursive_directory_iterator dir_iter( path );
    while(dir_iter != end) {
        const bfs::path current_path = dir_iter->path();
        if(bfs::is_regular_file(current_path)) {
            const std::string file = current_path.filename().string();
            const std::size_t delim_pos = file.find_first_of('-');
            const std::string id = file.substr(0,delim_pos);
            const char type = file[delim_pos+1];

            auto &entry = content[id];
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

    for(const auto &e : content) {
        MetaEntry m = e.second;
        m.id = e.first;
        data_.emplace_back(m);
    }

    ainfo << "Loaded " << data_.size() << " entries! \n";
}

void RGBDIDImporter::startPlay()
{
    data_iterator_ = data_.begin();
    playing_ = true;
    ainfo << "Start playing..." << std::endl;
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
