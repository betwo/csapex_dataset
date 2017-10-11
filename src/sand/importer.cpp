#include "importer.hpp"

#include <csapex/model/node_modifier.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/msg/end_of_sequence_message.h>
#include <csapex/msg/io.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/utility/register_apex_plugin.h>

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
    auto index_file    = param::ParameterFactory::declareFileInputPath("index file", "");
    auto start_play    = param::ParameterFactory::declareTrigger("start play");
    auto stop_play     = param::ParameterFactory::declareTrigger("stop play");
    auto play_progress = param::ParameterFactory::declareOutputProgress("played").build<param::OutputProgressParameter>();
    auto generate_negative = param::ParameterFactory::declareBool("negative samples/generate", false);
    auto generate_seed   = param::ParameterFactory::declareValue("negative samples/random seed", 0);
    auto negative_width  = param::ParameterFactory::declareInterval("negative samples/width", 1, 640, 32, 256, 1);
    auto negative_height = param::ParameterFactory::declareInterval("negative samples/height", 1, 640, 32, 256, 1);
    auto negative_class  = param::ParameterFactory::declareValue("negative samples/class", -1);

    auto play_only = [this]() { return playing_; };
    auto no_play_only = [this]() { return dataset_ && !playing_; };
    auto generate_negative_only = [this]() { return param_generate_negative_; };

    parameters.addParameter(index_file, [this](param::Parameter* param) { import(param->as<std::string>()); });
    parameters.addParameter(generate_negative, param_generate_negative_);
    parameters.addConditionalParameter(generate_seed, generate_negative_only, param_generate_seed_);
    parameters.addConditionalParameter(negative_width, generate_negative_only, param_negative_width_);
    parameters.addConditionalParameter(negative_height, generate_negative_only, param_negative_height_);
    parameters.addConditionalParameter(negative_class, generate_negative_only, param_negative_class_);
    parameters.addConditionalParameter(start_play, no_play_only, [this](param::Parameter*) { if (dataset_) startPlay(); });
    parameters.addConditionalParameter(stop_play, play_only, [this](param::Parameter*) { playing_ = false; });
    parameters.addConditionalParameter(play_progress, play_only);

    play_progress_ = play_progress;
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
    if (play_itr_ != dataset_->end())
    {
        auto& entry = *play_itr_;

        auto pointcloud_msg = std::make_shared<PointCloudMessage>(entry.getFrame(), entry.getTimestamp());
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
            auto roi = cv::Rect(region.x, region.y, region.width, region.height) & box;
            if (roi.area() == 0)
                continue;

            RoiMessage msg;
            msg.frame_id = entry.getFrame();
            msg.stamp_micro_seconds = entry.getTimestamp();
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
        ++play_itr_;
    }
    else
    {
        playing_ = false;

        auto end = makeEmpty<EndOfSequenceMessage>();

        msg::publish(output_pointcloud_, end);
        msg::publish(output_rois_, end);
        msg::trigger(event_finished_);
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
            std::uniform_int_distribution<> height_dist(
                    std::min(param_negative_height_.first, annotation.getHeight()),
                    std::min(param_negative_height_.second, annotation.getHeight()));
            roi.width  = width_dist(rng);
            roi.height = height_dist(rng);

            std::uniform_int_distribution<> x_dist(0, annotation.getWidth() - roi.width - 1);
            std::uniform_int_distribution<> y_dist(0, annotation.getHeight() - roi.height - 1);
            roi.x = x_dist(rng);
            roi.y = y_dist(rng);
        }

        {
            const auto& regions = annotation.getRegions();
            if (std::any_of(regions.begin(), regions.end(),
                    [&roi](const Annotation::Region& region)
                    {
                        return (cv::Rect(region.x, region.y, region.width, region.height) & roi).area() > 0;
                    }))
                continue;
        }

        RoiMessage msg;
        msg.frame_id = entry.getFrame();
        msg.stamp_micro_seconds = entry.getTimestamp();
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
}
