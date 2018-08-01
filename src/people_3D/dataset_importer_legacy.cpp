/// HEADER
#include "dataset_importer_legacy.hpp"

/// PROJECT
#include <csapex/model/connector_type.h>
#include <csapex/model/node_modifier.h>
#include <csapex/signal/event.h>
#include <csapex/msg/io.h>
#include <csapex/msg/message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/factory/message_factory.h>
#include <csapex/msg/end_of_sequence_message.h>

#include <csapex/param/parameter_factory.h>
#include <csapex/param/path_parameter.h>
#include <csapex/param/interval_parameter.h>
#include <csapex/param/output_progress_parameter.h>

#include <csapex_point_cloud/msg/point_cloud_message.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex_opencv/roi_message.h>
#include <csapex/msg/generic_vector_message.hpp>

/// SYSTEM
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <boost/mpl/for_each.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex::dataset;


CSAPEX_REGISTER_CLASS(csapex::dataset::PeopleDatasetImporterLegacy, csapex::Node)


PeopleDatasetImporterLegacy::PeopleDatasetImporterLegacy() :
    dir_structure_{{DEPTH,"depth"},
{PCL,"pointcloud"},
{VISUAL, "visual"},
{ROI, "roi"}},
    file_types_{{DEPTH, ".yaml.tar.gz"},
{PCL, ".pcd"},
{VISUAL, ".ppm"},
{ROI, ".yaml"}},
    lets_play_(false)
{
}

void PeopleDatasetImporterLegacy::setup(csapex::NodeModifier &node_modifier)
{
    out_pointcloud_  = node_modifier.addOutput<PointCloudMessage>("Pointcloud");
    out_depth_image_ = node_modifier.addOutput<CvMatMessage>("Depth Image");
    out_bgr_image_   = node_modifier.addOutput<CvMatMessage>("BGR Image");
    out_rois_        = node_modifier.addOutput<GenericVectorMessage, RoiMessage>("ROIs");
    finished_        = node_modifier.addEvent("finished");
}

void PeopleDatasetImporterLegacy::setupParameters(Parameterizable &parameters)
{
    std::map<std::string, int> ratio_selection = {
            {"1:2", Ratio_1_to_2},
            {"1:1", Ratio_1_to_1},
            {"free", Ratio_free}
    };
    import_path_ = param::factory::declareDirectoryInputPath("path", "");
    import_      = param::factory::declareTrigger("import");
    use_random_  = param::factory::declareBool("random", false);;
    random_seed_ = param::factory::declareValue("random seed", 0);
    interval_    = param::factory::declareInterval<int>("selected samples", 0, 0, 0, 0, 1);
    play_        = param::factory::declareBool("play", false);
    play_btn_    = param::factory::declareTrigger("start play");
    play_progress_ = param::factory::declareOutputProgress("played");
    prep_progress_ = param::factory::declareOutputProgress("preparation");
    non_human_rois_ = param::factory::declareBool("generate non-human rois", true);
    ratio_ = param::factory::declareParameterSet("ratio",
                                                          param::ParameterDescription("Ratio width to height."),
                                                          ratio_selection, (int) Ratio_1_to_2);

    std::function<bool()> random =
            [this]() { return use_random_->as<bool>() &&
                !play_->as<bool>();};
    std::function<bool()> no_random =
            [this](){ return !use_random_->as<bool>() &&
                dataset_.size() > 0 &&
                !play_->as<bool>();};
    std::function<bool()> playing =
            [this](){return play_->as<bool>();};
    std::function<bool()> not_playing =
            [this](){return !play_->as<bool>();};

    std::function<bool()> non_human_generation =
            [this](){return non_human_rois_->as<bool>();};

    addParameter(import_path_);
    addConditionalParameter(import_,
                            not_playing,
                            std::bind(&PeopleDatasetImporterLegacy::import, this));

    addConditionalParameter(random_seed_,
                            not_playing,
                            std::bind(&PeopleDatasetImporterLegacy::resetPlaySet, this));

    addConditionalParameter(use_random_,
                            not_playing,
                            std::bind(&PeopleDatasetImporterLegacy::resetPlaySet, this));

    addConditionalParameter(param::factory::declareRange("sample ratio", 0.01, 0.99, 0.5, 0.01),
                            random,
                            std::bind(&PeopleDatasetImporterLegacy::resetPlaySet, this));

    addConditionalParameter(interval_,
                            no_random,
                            std::bind(&PeopleDatasetImporterLegacy::resetPlaySet, this));

    addConditionalParameter(param::factory::declareBool("complement", false),
                            not_playing,
                            std::bind(&PeopleDatasetImporterLegacy::resetPlaySet, this));

    addConditionalParameter(non_human_rois_,
                            not_playing,
                            std::bind(&PeopleDatasetImporterLegacy::resetPlaySet, this));

    addConditionalParameter(param::factory::declareBool("stride", false),
                            not_playing,
                            stride_);

    addConditionalParameter(ratio_,
                            not_playing,
                            std::bind(&PeopleDatasetImporterLegacy::resetPlaySet, this));

    addConditionalParameter(param::factory::declareRange("min generation size", 10, 640, 32, 1),
                            non_human_generation,
                            std::bind(&PeopleDatasetImporterLegacy::resetPlaySet, this));

    addConditionalParameter(param::factory::declareRange("max generation size", 10, 640, 128, 1),
                            non_human_generation,
                            std::bind(&PeopleDatasetImporterLegacy::resetPlaySet, this));

    addConditionalParameter(param::factory::declareBool("publish partly visible humans", true),
                            not_playing,
                            std::bind(&PeopleDatasetImporterLegacy::resetPlaySet, this));




    addParameter(play_, std::bind(&PeopleDatasetImporterLegacy::play, this));
    addParameter(play_btn_, [this](csapex::param::Parameter*) {
        play_->set(true);
    });
    addParameter(param::factory::declareBool("hold", false),
                 hold_);
    addConditionalParameter(prep_progress_, playing);
    addConditionalParameter(play_progress_, playing);
}

namespace
{

struct loader
{
    loader(connection_types::PointCloudMessage::Ptr& out, const std::string& path) :
        out_(out),
        success_(false)
    {
        pcl::io::loadPCDFile(path, cloud_);
    }

    template<typename PointT>
    void operator()(PointT& pt)
    {
        if (success_)
            return;

        std::vector<pcl::PCLPointField> fields;
        pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(pcl::detail::FieldAdder<PointT>(fields));

        const std::vector<pcl::PCLPointField>& reference = cloud_.fields;

        if (fields.size() != reference.size())
            return;

        for (std::size_t d = 0; d < fields.size(); ++d) {
            bool found = false;
            for (std::size_t f = 0; f < reference.size() && !found; ++f) {
                if (fields[d].name == reference[f].name)
                    found = true;
            }

            if (!found)
                return;
        }

        success_ = true;
        typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::fromPCLPointCloud2(cloud_, *cloud);
        out_->value = cloud;
    }

private:
    connection_types::PointCloudMessage::Ptr& out_;
    pcl::PCLPointCloud2 cloud_;


    bool success_;
};

}

bool PeopleDatasetImporterLegacy::canProcess() const
{
    return lets_play_;
}

void PeopleDatasetImporterLegacy::process()
{
    if(play_pos_ < play_set_.size()) {

        DataSetEntry &entry = play_set_.at(play_pos_);

        CvMatMessage::Ptr      depth_msg(new CvMatMessage(enc::depth, "camera_depth_optical_frame", entry.ts));
        CvMatMessage::Ptr      img_msg(new CvMatMessage(enc::bgr, "camera_depth_optical_frame", entry.ts));
        PointCloudMessage::Ptr pcl_msg(new PointCloudMessage("camera_depth_optical_frame", entry.ts));
        std::shared_ptr<std::vector<RoiMessage>> roi_msg(new std::vector<RoiMessage>);

        cv::FileStorage depth_fs(entry.path_depth.string(), cv::FileStorage::READ);
        depth_fs["data"] >> depth_msg->value;
        depth_fs.release();
        img_msg->value = cv::imread(entry.path_rgb.string(), cv::IMREAD_UNCHANGED);
        if(img_msg->value.channels() == 1)
            img_msg->setEncoding(enc::mono);

        loader loader(pcl_msg, entry.path_pcl.string());
        boost::mpl::for_each<connection_types::PointCloudPointTypes>( loader );

        if(readParameter<bool>("publish partly visible humans")) {
            roi_msg->assign(entry.rois.begin(), entry.rois.end());
        } else {
            for(auto &r : entry.rois) {
                if(r.value.classification() == HUMAN_PART)
                    continue;
                roi_msg->emplace_back(r);
            }
        }

        msg::publish(out_depth_image_, depth_msg);
        msg::publish(out_bgr_image_, img_msg);
        msg::publish(out_pointcloud_, pcl_msg);
        msg::publish<GenericVectorMessage, RoiMessage>(out_rois_, roi_msg);

        if(!hold_)
            ++play_pos_;
        auto *prog = (csapex::param::OutputProgressParameter*) play_progress_.get();
        prog->setProgress(play_pos_ + 1, play_set_.size());

    } else {
        play_->set(false);
        lets_play_ = false;
        msg::trigger(finished_);

        auto end = makeEmpty<EndOfSequenceMessage>();

        msg::publish(out_depth_image_, end);
        msg::publish(out_bgr_image_, end);
        msg::publish(out_pointcloud_, end);
        msg::publish(out_rois_, end);

    }
}

void PeopleDatasetImporterLegacy::import()
{
    import_->setEnabled(false);

    resetPlaySet();
    node_modifier_->setNoError();
    root_path_ = readParameter<std::string>("path");
    checkDirectoryStructure();

    param::IntervalParameter *interval = (param::IntervalParameter*) interval_.get();
    dataset_.clear();
    interval->setMax(0);
    interval->setLower(0);
    interval->setUpper(0);

    bfs::path roi_path(root_path_);
    roi_path /= dir_structure_.at(ROI);

    auto file_type = file_types_.at(ROI);
    for(bfs::directory_iterator it(roi_path) ;
        it != bfs::directory_iterator() ;
        ++it) {
        bfs::path entry_path = it->path();
        if(bfs::is_regular_file(entry_path) &&
                bfs::extension(entry_path) == file_type) {
            std::string entry_id = bfs::basename(entry_path);
            std::cout << "import: " << entry_id << std::endl;
            DataSetEntry entry;
            createEntry(entry_id,
                        entry_path,
                        entry);
            dataset_.emplace_back(entry);
        }
    }

    std::sort(dataset_.begin(), dataset_.end(), DataSetEntry::compare);

    int max_idx = dataset_.size() - 1;
    interval->setMax(max_idx);
    interval->setUpper(max_idx);
    import_->setEnabled(true);
}

void PeopleDatasetImporterLegacy::resetPlaySet()
{
    play_set_.clear();
    play_->set(false);
    lets_play_ = false;
}

void PeopleDatasetImporterLegacy::createEntry(const std::string &id,
                                  const bfs::path &rois_path,
                                  DataSetEntry &entry)
{
    entry.id = id;
    /// check if all necessary files are there
    entry.path_depth = root_path_ / dir_structure_.at(DEPTH) /
            (id + file_types_.at(DEPTH).string());
    entry.path_pcl   = root_path_ / dir_structure_.at(PCL) /
            (id + file_types_.at(PCL).string());
    entry.path_rgb   = root_path_ / dir_structure_.at(VISUAL) /
            (id + file_types_.at(VISUAL).string());

    if(!bfs::exists(entry.path_depth)) {
        import_->setEnabled(true);
        node_modifier_->setError("Inconsistent content for '" + id + "' - missing depth");
        throw std::runtime_error("Inconsistent content for '" + id + "' - missing depth");
    }
    if(!bfs::exists(entry.path_rgb)) {
        import_->setEnabled(true);
        node_modifier_->setError("Inconsistent content for '" + id + "' - missing rgb / intensity");
        throw std::runtime_error("Inconsistent content for '" + id + "' - missing rgb / intensity");

    }
    if(!bfs::exists(entry.path_pcl)) {
        import_->setEnabled(true);
        node_modifier_->setError("Inconsistent content for '" + id + "' - missing pcl");
        throw std::runtime_error("Inconsistent content for '" + id + "' - missing pcl");
    }

    /// import yaml
    YAML::Node file = YAML::LoadFile(rois_path.string());
    if(!file.IsMap())
        return;
    entry.size_depth.height = file["img_d_h"].as<int>();
    entry.size_depth.width = file["img_d_w"].as<int>();
    entry.ts = file["ts"].as<ulong>();
    cv::Rect depth_rect(0,0, entry.size_depth.width, entry.size_depth.height);
    YAML::Node rois = file["rois"];
    if(!rois.IsSequence())
        return;
    for(YAML::iterator it = rois.begin() ; it != rois.end() ; ++it) {
        if(it->IsMap()) {
            int vis = (*it)["vis"].as<int>();

//            if(vis != HUMAN && vis != HUMAN_PART)
//                continue;

            cv::Rect rect;
            rect.x       = (*it)["x_d"   ].as<int>();
            rect.y       = (*it)["y_d"   ].as<int>();
            rect.width   = (*it)["w_d"   ].as<int>();
            rect.height  = (*it)["h_d"   ].as<int>();
            if(rect.height <= 0 || rect.width <= 0)
                continue;

            RoiMessage roi;
            roi.value.setRect(rect & depth_rect);

            if(vis == HUMAN)
                roi.value.setColor(colors_[HUMAN]);
            else
                roi.value.setColor(colors_[HUMAN_PART]);
            roi.value.setClassification(vis);

            entry.rois.push_back(roi);

            if(stride_) {
                int dx[] = {-5, -5, 5, 5};
                int dy[] = {-5,  5,-5, 5};
                for(int i = 0 ; i < 4 ; ++i) {
                    RoiMessage stride_roi = roi;
                    cv::Rect stride_rect = stride_roi.value.rect();
                    stride_rect.x += dx[i];
                    stride_rect.y += dy[i];
                    stride_roi.value.setRect(stride_rect & depth_rect);
                    entry.rois.push_back(stride_roi);
                }
            }

        }
    }

}

void PeopleDatasetImporterLegacy::checkDirectoryStructure() const
{
    node_modifier_->setNoError();
    for(auto &sub : dir_structure_) {
        bfs::path sub_path(root_path_);
        sub_path /= sub.second;
        if(!bfs::exists(sub_path)) {
            node_modifier_->setError("Path '" + sub_path.string() + "' not found!");
        }
        if(!bfs::is_directory(sub_path)) {
            node_modifier_->setError("Path '" + sub_path.string() + "' is not a directory!");
        }
    }
}

void PeopleDatasetImporterLegacy::play()
{

    bool lets_play = play_->as<bool>();
    if(dataset_.empty()) {
        play_->set(false);
        return;
    }

    if(lets_play) {
        prepareThePlaySet();
        play_pos_ = 0;
    }

    if(lets_play)
        std::cout << "Play" << std::endl;
    else
        std::cout << "End Play" << std::endl;

    lets_play_ = lets_play;
    if(lets_play) {
        yield();
    }
}

void PeopleDatasetImporterLegacy::prepareThePlaySet()
{
    /// if the play set was emptied due to parameter update, update the play set
    if(play_set_.empty() &&
            !dataset_.empty()) {
        bool complement = readParameter<bool>("complement");
        int  seed = readParameter<int>("random seed");
        random_ = std::minstd_rand(seed);
        auto *prog = (csapex::param::OutputProgressParameter*) prep_progress_.get();


        if(use_random_->as<bool>()) {
            double      ratio = readParameter<double>("sample ratio");
            std::size_t samples = ratio * dataset_.size();
            std::vector<char> mask(dataset_.size(), 0);
            play_set_.resize(samples);
            for(std::size_t i = 0 ; i < samples ; ++i) {
                std::size_t index;
                do {
                    index = random_() % dataset_.size();
                } while(mask[index] == 1);
                mask[index] = 1;
                prog->setProgress(i+1, samples);
            }

            char selection = complement ? 0 : 1;
            std::size_t sample_index = 0;
            for(std::size_t i = 0 ; i < dataset_.size() ; ++i) {
                if(mask[i] == selection) {
                    play_set_[sample_index] = dataset_[i];
                }
            }
        } else {
            /// assign inclusively
            param::IntervalParameter *interval = (param::IntervalParameter*) interval_.get();
            std::size_t lower = interval->lower<int>();
            std::size_t upper = interval->upper<int>();
            prog->setProgress(0,1);
            if(complement) {
                if(lower > 0) {
                    play_set_.insert(play_set_.end(), dataset_.begin(), dataset_.begin() + lower);
                }
                if(upper < dataset_.size() - 1) {
                    play_set_.insert(play_set_.end(), dataset_.begin() + upper + 1, dataset_.end());
                }
            } else {
                play_set_.assign(dataset_.begin() + lower, dataset_.begin() + upper + 1);
            }
            prog->setProgress(1,1);
        }

        std::sort(play_set_.begin(), play_set_.end(), DataSetEntry::compare);

        if(readParameter<bool>("generate non-human rois")) {
            std::size_t positives = 0;
            bool ignore_partly_visible =
                    readParameter<bool>("publish partly visible humans");
            int ratio;
            switch(readParameter<int>("ratio")) {
            case Ratio_1_to_1:
                ratio = 1;
                break;
            case Ratio_1_to_2:
                ratio = 2;
                break;
            default:
                ratio = -1;
            }

            //// update this shit here
            if(ignore_partly_visible) {
                for(auto &entry : play_set_) {
                    for(RoiMessage &roi : entry.rois) {
                        if(roi.value.classification() == HUMAN)
                            ++positives;
                    }
                }
            } else {
                for(auto &entry : play_set_) {
                    positives += entry.rois.size();
                }
            }

            std::size_t min_size = readParameter<int>("min generation size");
            std::size_t max_size = readParameter<int>("max generation size");
             /// now we know how much negative samples we need
            std::size_t i = 0;
            while(i < positives) {
                std::size_t   index = random_() % play_set_.size();
                DataSetEntry &entry = play_set_.at(index);
                cv::Rect dept_rect(0,0, entry.size_depth.width, entry.size_depth.height);
                /// generate a rondom roi depending on entry parameters
                cv::Rect roi;
                if(ratio == -1) {
                    roi.width  = min_size + (random_() % (max_size - min_size));
                    roi.height = min_size + (random_() % (max_size - min_size));
                } else {
                    roi.width  = min_size + (random_() % (max_size - min_size));
                    roi.height = roi.width * ratio;
                }
                roi.width = std::max(roi.width, 20);
                roi.height = std::max(roi.height, 20);
                if(roi.width >= entry.size_depth.width)
                    continue;
                if(roi.height >= entry.size_depth.height)
                    continue;

                roi.x = random_() % (entry.size_depth.width - roi.width);
                roi.y = random_() % (entry.size_depth.height - roi.height);

                /// check if the roi interferes with anything in the image
                bool interfering = false;
                for(RoiMessage &m : entry.rois) {
                    cv::Rect msg_rect = m.value.rect();
                    interfering |= (msg_rect & roi).area() > 0;
                    if(interfering)
                        break;
                }
                if(!interfering) {
                    RoiMessage msg;
                    msg.value.setRect(roi & dept_rect);
                    msg.value.setClassification(BACKGROUND);
                    msg.value.setColor(colors_[BACKGROUND]);
                    entry.rois.push_back(msg);
                    ++i;
                    prog->setProgress(i + 1, positives);
                }
            }
        }
    }
}
