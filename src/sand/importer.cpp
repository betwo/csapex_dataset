#include "importer.hpp"

#include <csapex/model/node_modifier.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/msg/end_of_sequence_message.h>
#include <csapex/msg/io.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/signal/event.h>

#include <csapex_point_cloud/msg/point_cloud_message.h>
#include <pcl/conversions.h>
#include <boost/make_shared.hpp>
#include <boost/mpl/for_each.hpp>
#include <boost/filesystem/operations.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex::dataset::sand;
namespace bfs = boost::filesystem;

CSAPEX_REGISTER_CLASS(csapex::dataset::sand::SandDatasetImporter, csapex::Node)

namespace
{
int classToBit(Annotation::Class clazz)
{
    if (clazz < 0)
        return 1 << (std::numeric_limits<int>::digits + clazz);
    else
        return 1 << static_cast<int>(clazz);
}

static const std::string LABEL_START_PLAY = "start play";
}

SandDatasetImporter::SandDatasetImporter() :
        playing_(false)
{}

void SandDatasetImporter::setup(NodeModifier& node_modifier)
{
    output_pointcloud_ = node_modifier.addOutput<PointCloudMessage>("Pointcloud");
    output_rois_       = node_modifier.addOutput<GenericVectorMessage, RoiMessage>("ROIs");
    event_finished_    = node_modifier.addEvent("finished");
}

void SandDatasetImporter::setupParameters(Parameterizable& parameters)
{
    auto index_file    = param::ParameterFactory::declareFileInputPath("index file", "").build();

    auto reload        = param::ParameterFactory::declareTrigger("reload");
    auto start_play    = param::ParameterFactory::declareTrigger(LABEL_START_PLAY);
    auto stop_play     = param::ParameterFactory::declareTrigger("stop play");
    auto start_instantly = param::ParameterFactory::declareBool("start instantly", false);
    auto play_progress = param::ParameterFactory::declareOutputProgress("played").build<param::OutputProgressParameter>();
    auto current_frame = param::ParameterFactory::declareOutputText("current frame").build<param::OutputTextParameter>();

    const std::map<std::string, int> CLASS_SET{
            { "background",       classToBit(Annotation::CLASS_BACKGROUND)},
            { "unknown",          classToBit(Annotation::CLASS_UNKNOWN)},
            { "person",           classToBit(Annotation::CLASS_PERSON)},
            { "person (partial)", classToBit(Annotation::CLASS_PARTIAL_PERSON)}};
    auto load_classes = param::ParameterFactory::declareParameterBitSet("load classes", CLASS_SET,
            classToBit(Annotation::CLASS_PERSON));

    auto generate_negative = param::ParameterFactory::declareBool("negative samples/generate", false);
    auto generate_seed   = param::ParameterFactory::declareValue("negative samples/random seed", 0);
    auto negative_ratio  = param::ParameterFactory::declareRange("negative samples/ratio", 0.0, 8.0, 0.0, 0.1);
    auto negative_width  = param::ParameterFactory::declareInterval("negative samples/width", 1, 640, 32, 256, 1);
    auto negative_height = param::ParameterFactory::declareInterval("negative samples/height", 1, 640, 32, 256, 1);
    auto negative_class  = param::ParameterFactory::declareValue("negative samples/class", -1);
    auto no_overlap      = param::ParameterFactory::declareBool("negative samples/no overlap", true).build();
    auto check_classes   = param::ParameterFactory::declareParameterBitSet("negative samples/overlap check classes", CLASS_SET,
            classToBit(Annotation::CLASS_BACKGROUND) | classToBit(Annotation::CLASS_PERSON) | classToBit(Annotation::CLASS_PARTIAL_PERSON));

    auto play_only = [this]() { return playing_; };
    auto no_play_only = [this]() { return dataset_ && !playing_; };
    auto generate_negative_only = [this]() { return param_generate_negative_; };
    auto no_ratio = [this]() { return param_generate_negative_ && param_negative_ratio_ == 0.0; };
    auto overlap_only = [this]() { return param_generate_negative_ && param_no_overlap_; };

    param::Parameter::Ptr interval =  param::ParameterFactory::declareInterval("sample interval", 0, 0, 0, 0, 1);
    interval_ = std::dynamic_pointer_cast<param::IntervalParameter>(interval);
    parameters.addParameter(interval_, interval_boundaries_);

    parameters.addParameter(index_file, [this](param::Parameter* param) { import(param->as<std::string>()); });
    parameters.addConditionalParameter(reload, no_play_only, [this, index_file](param::Parameter* param) { import(index_file->as<std::string>()); });
    parameters.addParameter(start_instantly,
                            [this](param::Parameter* param)
                            {
                                param_start_instantly_ = param->as<bool>();
                                if (param_start_instantly_ && dataset_)
                                {
                                    startPlay();
                                    triggerStartPlayEvent();
                                }
                            });

    parameters.addParameter(load_classes, param_load_classes_);

    parameters.addParameter(generate_negative, param_generate_negative_);
    parameters.addConditionalParameter(generate_seed, generate_negative_only, param_generate_seed_);
    parameters.addConditionalParameter(negative_ratio, generate_negative_only, param_negative_ratio_);
    parameters.addConditionalParameter(negative_width, generate_negative_only, param_negative_width_);
    parameters.addConditionalParameter(negative_height, no_ratio, param_negative_height_);
    parameters.addConditionalParameter(negative_class, generate_negative_only, param_negative_class_);
    parameters.addConditionalParameter(no_overlap, generate_negative_only, param_no_overlap_);
    parameters.addConditionalParameter(check_classes, overlap_only, param_check_overlap_classes_);

    parameters.addConditionalParameter(start_play, no_play_only, [this](param::Parameter*) { if (dataset_) startPlay(); });
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

bool SandDatasetImporter::canProcess() const
{
    return dataset_ && playing_;
}

namespace
{
class PointCloudLoader
{
public:
    PointCloudLoader(const Entry& entry, PointCloudMessage& msg) :
            msg_(msg),
            success_(false)
    {
        entry.getPointcloud(pointcloud_);
    }

    template<typename PointT>
    void operator()(PointT)
    {
        if (success_)
            return;

        std::vector<pcl::PCLPointField> fields;
        pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(pcl::detail::FieldAdder<PointT>(fields));

        auto& reference = pointcloud_.fields;

        if (fields.size() != reference.size())
            return;

        for (auto& field : fields)
        {
            auto count = std::count_if(reference.begin(), reference.end(), [&](const pcl::PCLPointField& ref) { return ref.name == field.name; });
            if (count != 1)
                return;
        }

        auto pointcloud = boost::make_shared<pcl::PointCloud<PointT>>();
        pcl::fromPCLPointCloud2(pointcloud_, *pointcloud);
        msg_.value = pointcloud;

        success_ = true;
    }

private:
    PointCloudMessage& msg_;
    pcl::PCLPointCloud2 pointcloud_;
    bool success_;
};
}

void SandDatasetImporter::process()
{


    if(play_itr_ - dataset_->begin() < interval_boundaries_.first)
        play_itr_ = dataset_->begin() + interval_boundaries_.first;

    auto end = dataset_->begin() + interval_boundaries_.second + 1;

    if (play_itr_ != end)
    {
        auto& entry = *play_itr_;

        auto pointcloud_msg = std::make_shared<PointCloudMessage>(entry.getAnnotation().getFrame(), entry.getAnnotation().getTimestamp());
        auto rois_msg = std::make_shared<std::vector<RoiMessage>>();

        // load pointcloud
        {
            PointCloudLoader loader(entry, *pointcloud_msg);
            boost::mpl::for_each<PointCloudPointTypes>(loader);
        }

        // load labeled rois
        const auto& annotation = entry.getAnnotation();
        const cv::Rect box = cv::Rect(0, 0, annotation.getWidth(), annotation.getHeight());
        for (auto& region : annotation.getRegions())
        {
            if ((classToBit(region.clazz) & param_load_classes_) == 0)
                continue;

            auto roi = cv::Rect(region.x, region.y, region.width, region.height) & box;
            if (roi.area() == 0)
                continue;

            RoiMessage msg;
            msg.frame_id = entry.getAnnotation().getFrame();
            msg.stamp_micro_seconds = entry.getAnnotation().getTimestamp();
            msg.value.setRect(roi);
            msg.value.setClassification(region.clazz);
            rois_msg->push_back(std::move(msg));
        }

        // load generated negative samples
        {
            auto negative = negative_rois_.find(entry.getId());
            if (negative != negative_rois_.end())
                std::copy(negative->second.begin(), negative->second.end(), std::back_inserter(*rois_msg));
        }

        msg::publish(output_pointcloud_, pointcloud_msg);
        msg::publish(output_rois_, rois_msg);

        play_progress_->advanceProgress();
        current_frame_->set("Frame ID: " + std::to_string(entry.getId()));
        ++play_itr_;

        if ((int)play_progress_->getProgress() % 100 == 0)
            ainfo << "Play progress " << (play_progress_->getProgress() / play_progress_->getProgressMaximum() * 100) << "%"
                          <<" (" << play_progress_->getProgress() << " of " << play_progress_->getProgressMaximum() << ")"
                          << std::endl;
    }
    else
    {
        playing_ = false;

        auto end = makeEmpty<EndOfSequenceMessage>();

        msg::publish(output_pointcloud_, end);
        msg::publish(output_rois_, end);
        msg::trigger(event_finished_);

        ainfo << "Play progress 100%, finished" << std::endl;
    }
}

void SandDatasetImporter::import(const boost::filesystem::path& path)
{
    if (path.empty())
        return;

    if (!bfs::exists(path))
        throw std::runtime_error("Unknown dataset file: " + path.string());

    playing_ = false;

    dataset_.reset(new Dataset(path));
    play_itr_ = dataset_->begin();
    play_progress_->setProgress(0, dataset_->size());
    negative_rois_.clear();

    int max_sample = static_cast<int>(dataset_->size() - 1);
    interval_->setMin(0);
    interval_->setMax(max_sample);
    interval_->setLower(0);
    interval_->setUpper(max_sample);

    if (param_start_instantly_)
    {
        startPlay();
        triggerStartPlayEvent();
    }
}

void SandDatasetImporter::generateNegativeSamples()
{
    negative_rois_.clear();

    std::size_t samples = 0;
    for (auto& entry : *dataset_)
        samples += entry.getAnnotation().getRegions().size();

    std::minstd_rand rng(param_generate_seed_);
    std::uniform_int_distribution<> index_dist(0, dataset_->size() - 1);

    std::size_t i = 0;
    while (i < samples)
    {
        const auto& entry      = (*dataset_)[index_dist(rng)];
        const auto& annotation = entry.getAnnotation();

        cv::Rect roi;
        {
            std::uniform_int_distribution<> width_dist(
                    std::min(param_negative_width_.first, annotation.getWidth()),
                    std::min(param_negative_width_.second, annotation.getWidth()));
            roi.width  = width_dist(rng);

            if (param_negative_ratio_ == 0.0)
            {
                std::uniform_int_distribution<> height_dist(
                        std::min(param_negative_height_.first, annotation.getHeight()),
                        std::min(param_negative_height_.second, annotation.getHeight()));
                roi.height = height_dist(rng);
            }
            else
                roi.height = static_cast<int>(roi.width * param_negative_ratio_);

            std::uniform_int_distribution<> x_dist(0, annotation.getWidth() - roi.width - 1);
            std::uniform_int_distribution<> y_dist(0, annotation.getHeight() - roi.height - 1);
            roi.x = x_dist(rng);
            roi.y = y_dist(rng);
        }

        if (param_no_overlap_)
        {
            const auto& regions = annotation.getRegions();
            if (std::any_of(regions.begin(), regions.end(),
                    [this, &roi](const Annotation::Region& region)
                    {
                        if ((classToBit(region.clazz) & param_check_overlap_classes_) == 0)
                            return false;

                        return (cv::Rect(region.x, region.y, region.width, region.height) & roi).area() > 0;
                    }))
                continue;

            auto negative_itr = negative_rois_.find(entry.getId());
            if (negative_itr != negative_rois_.end())
                if (std::any_of(negative_itr->second.begin(), negative_itr->second.end(),
                        [this, &roi](const RoiMessage& msg)
                        {
                            if ((classToBit(static_cast<Annotation::Class>(msg.value.classification())) & param_check_overlap_classes_) == 0)
                                return false;

                            return (msg.value.rect() & roi).area() > 0;
                        }))
                    continue;
        }

        RoiMessage msg;
        msg.frame_id = entry.getAnnotation().getFrame();
        msg.stamp_micro_seconds = entry.getAnnotation().getTimestamp();
        msg.value = roi;
        msg.value.setClassification(param_negative_class_);
        negative_rois_[entry.getId()].push_back(std::move(msg));

        ++i;
    }
}

void SandDatasetImporter::startPlay()
{
    if (param_generate_negative_)
        generateNegativeSamples();

    play_itr_ = dataset_->begin();
    playing_ = true;

    ainfo << "Start playing..." << std::endl;
}

void SandDatasetImporter::triggerStartPlayEvent()
{
    const auto& events = node_modifier_->getEvents();
    auto itr = std::find_if(events.begin(), events.end(),
                            [](const EventPtr& event) { return event->getLabel() == LABEL_START_PLAY; });
    if (itr == events.end())
        throw std::runtime_error("Unable to find 'start play' event");

    auto& ptr = (*itr);
    ptr->trigger();
}
