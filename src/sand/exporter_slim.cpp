#include "exporter_slim.hpp"

#include <csapex/model/node_modifier.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/msg/end_of_sequence_message.h>
#include <csapex/msg/io.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/utility/register_apex_plugin.h>

#include <csapex_point_cloud/msg/point_cloud_message.h>
#include <csapex_opencv/roi_message.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex::dataset::sand;
namespace bfs = boost::filesystem;

CSAPEX_REGISTER_CLASS(csapex::dataset::sand::SandDatasetExporterSlim, csapex::Node)


SandDatasetExporterSlim::SandDatasetExporterSlim()
{}

void SandDatasetExporterSlim::setup(NodeModifier& node_modifier)
{
    in_pointcloud_ = node_modifier.addInput<PointCloudMessage>("PointCloud");
    in_rois_ = node_modifier.addOptionalInput<GenericVectorMessage, RoiMessage>("ROIs");
}

void SandDatasetExporterSlim::setupParameters(Parameterizable& parameters)
{
    auto ref_index_file = param::ParameterFactory::declareFileInputPath("reference index file", "").build();
    auto index_file     = param::ParameterFactory::declareFileOutputPath("index file", "").build();
    auto annotation_dir = param::ParameterFactory::declareDirectoryOutputPath("annoation dir", "").build();
    auto save_empty_rois = param::ParameterFactory::declareBool("save empty rois", false);
    auto save = param::ParameterFactory::declareTrigger("save");

    std::function<void(param::Parameter*)> createDataset = [this, ref_index_file, index_file, annotation_dir](param::Parameter* /*param*/)
    {
        this->annotation_dir_ = annotation_dir->as<std::string>();
        this->createDataset(ref_index_file->as<std::string>(), index_file->as<std::string>());
    };

    parameters.addParameter(ref_index_file, createDataset);
    parameters.addParameter(index_file, createDataset);
    parameters.addParameter(annotation_dir, createDataset);
    parameters.addParameter(save_empty_rois, save_empty_rois_);
    parameters.addParameter(save, [this](param::Parameter*) { saveDataset(); });
}

bool SandDatasetExporterSlim::canProcess() const
{
    return static_cast<bool>(dataset_);
}

void SandDatasetExporterSlim::process()
{
    auto pointcloud_msg = msg::getMessage<PointCloudMessage>(in_pointcloud_);
    auto rois_msg = msg::getMessage<GenericVectorMessage, RoiMessage>(in_rois_);

    const auto id  = pointcloud_msg->stamp_micro_seconds;
    auto reference = reference_dataset_->findById(id);
    if (!reference)
        throw std::runtime_error("Unknown entry (id: " + std::to_string(id) + ") in reference dataset");

    Annotation annotation(reference->getAnnotation());
    {
        auto& regions = annotation.getRegions();
        regions.clear();
        if (rois_msg)
        {
            for (const auto& roi_msg : *rois_msg)
            {
                const auto& roi = roi_msg.value;
                Annotation::Region region;
                region.x      = roi.x();
                region.y      = roi.y();
                region.width  = roi.w();
                region.height = roi.h();
                region.clazz  = static_cast<Annotation::Class>(roi.classification());
                regions.push_back(region);
            }
        }

        if (regions.empty() && !save_empty_rois_)
            return;
    }

    bfs::path annotation_file = annotation_dir_ / (std::to_string(id) + ".yaml");
    Annotation::save(annotation, annotation_file);

    Entry entry(id, reference->getPointCloudPath(), annotation_file);
    dataset_->add(entry);
}

void SandDatasetExporterSlim::createDataset(boost::filesystem::path reference_index_file,
                                            boost::filesystem::path index_file)
{
    reference_dataset_.reset(new Dataset(reference_index_file));
    dataset_.reset(new Dataset());
    index_file_ = index_file;
}

void SandDatasetExporterSlim::saveDataset()
{
    if (dataset_)
        dataset_->save(index_file_);
}

