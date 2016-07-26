/// HEADER
#include "dataset_exporter.hpp"
#include "dataset_common.hpp"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex_vision/roi_message.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_point_cloud/point_cloud_message.h>

CSAPEX_REGISTER_CLASS(csapex::dataset::PeopleDatasetExporter, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex::dataset;
using namespace csapex::dataset::people;

PeopleDatasetExporter::PeopleDatasetExporter()
{
}

void PeopleDatasetExporter::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(param::ParameterFactory::declareDirectoryOutputPath("directory",
                                                                                param::ParameterDescription("Dataset root directory"),
                                                                                ""),
                            directory_);

    parameters.addParameter(param::ParameterFactory::declareValue<int>("class",
                                                                       param::ParameterDescription("Classification class to be saved"),
                                                                       people::HUMAN),
                            class_);

    parameters.addParameter(param::ParameterFactory::declareBool("only_with_rois",
                                                                 param::ParameterDescription("Only exports frames with at least 1 ROI"),
                                                                 false),
                            only_with_rois_);
}

void PeopleDatasetExporter::setup(NodeModifier& node_modifier)
{
    in_depth_ = node_modifier.addInput<CvMatMessage>("depth");
    in_visual_ = node_modifier.addInput<CvMatMessage>("visual");
    in_pointcloud_ = node_modifier.addInput<PointCloudMessage>("pointcloud");
    in_rois_ = node_modifier.addOptionalInput<VectorMessage, RoiMessage>("rois");
    in_rois_gen_ = node_modifier.addOptionalInput<GenericVectorMessage, RoiMessage>("rois");
}

void PeopleDatasetExporter::process()
{
    CvMatMessage::ConstPtr depth_msg = msg::getMessage<CvMatMessage>(in_depth_);
    CvMatMessage::ConstPtr visual_msg = msg::getMessage<CvMatMessage>(in_visual_);
    PointCloudMessage::ConstPtr pointcloud_msg = msg::getMessage<PointCloudMessage>(in_pointcloud_);
    std::vector<RoiMessage> rois_msg;

    if (msg::hasMessage(in_rois_gen_))
    {
        std::shared_ptr<std::vector<RoiMessage> const> rois_vector_msg = msg::getMessage<GenericVectorMessage, RoiMessage>(in_rois_gen_);
        rois_msg = *rois_vector_msg;
    }
    if (msg::hasMessage(in_rois_))
    {
        VectorMessage::ConstPtr rois = msg::getMessage<VectorMessage>(in_rois_);
        for(const TokenData::ConstPtr& msg : rois->value)
            rois_msg.push_back(*std::dynamic_pointer_cast<RoiMessage const>(msg));
    }

    Entry entry;
    entry.meta.id = std::to_string(pointcloud_msg->stamp_micro_seconds);
    entry.meta.timestamp = pointcloud_msg->stamp_micro_seconds;
    std::copy_if(rois_msg.begin(), rois_msg.end(),
                 std::back_inserter(entry.meta.rois),
                 [=](const RoiMessage& msg) { return msg.value.classification() == class_; });

    if (only_with_rois_ && entry.meta.rois.empty())
        return;

    entry.depth = depth_msg->value;
    entry.visual = visual_msg->value;
    auto pcl = boost::get<typename pcl::PointCloud<pcl::PointXYZI>::Ptr>(pointcloud_msg->value);
    if (pcl)
        entry.pointcloud = *(pcl.get());

    if (!directory_.empty())
    {
        save_fill_meta(entry, directory_);
        save(entry);
    }
}
