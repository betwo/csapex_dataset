#include "roi_exporter.hpp"
#include "dataset_common.hpp"

#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

#include <fstream>
#include <yaml-cpp/yaml.h>


CSAPEX_REGISTER_CLASS(csapex::dataset::ROIExporter, csapex::Node);


using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex::dataset;
using namespace csapex::dataset::people;
namespace fs = boost::filesystem;


void ROIExporter::setup(csapex::NodeModifier& node_modifier)
{
    in_timestamp_ = node_modifier.addInput<AnyMessage>("Timestamp");
    in_rois_      = node_modifier.addInput<GenericVectorMessage, RoiMessage>("ROIs");
}

void ROIExporter::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(
            param::ParameterFactory::declareDirectoryInputPath("roi_dir", "", ""),
            config_roi_dir_);
}

void ROIExporter::process()
{
    std::uint64_t timestamp = 0;
    {
        TokenDataConstPtr timestamp_message = msg::getMessage(in_timestamp_);
        auto casted = msg::message_cast<const Message>(timestamp_message);

        if (casted)
            timestamp = casted->stamp_micro_seconds;
    }

    std::shared_ptr<std::vector<RoiMessage> const> rois_msg = msg::getMessage<GenericVectorMessage, RoiMessage>(in_rois_);

    YAML::Emitter yaml;
    yaml << YAML::BeginMap;
    yaml << "img_d_w" << 640;
    yaml << "img_d_h" << 480;
    yaml << "rois" << YAML::BeginSeq;
    for (const RoiMessage& roi : *rois_msg)
    {
        yaml << YAML::BeginMap;
        yaml << "ts" << roi.stamp_micro_seconds;
        yaml << "img_id" << timestamp;
        yaml << "x_d" << roi.value.rect().x;
        yaml << "y_d" << roi.value.rect().y;
        yaml << "w_d" << roi.value.rect().width;
        yaml << "h_d" << roi.value.rect().height;
        yaml << "x_rgb" << roi.value.rect().x;
        yaml << "y_rgb" << roi.value.rect().y;
        yaml << "w_rgb" << roi.value.rect().width;
        yaml << "h_rgb" << roi.value.rect().height;
        yaml << "vis" << roi.value.classification();
        yaml << YAML::EndMap;
    }
    yaml << YAML::EndSeq;
    yaml << "ts" << timestamp;
    yaml << YAML::EndMap;

    std::ostringstream filename;
    filename << config_roi_dir_ << "/" << timestamp << ".yaml";

    fs::path roi_file(filename.str());
    if (fs::exists(roi_file) && fs::is_symlink(roi_file))
        fs::remove(roi_file);

    std::ofstream out(roi_file.string());
    out << yaml.c_str();
    out.close();
}
