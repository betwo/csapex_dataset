#include "roi_importer.hpp"
#include "dataset_common.hpp"

#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>


CSAPEX_REGISTER_CLASS(csapex::dataset::ROIImporter, csapex::Node);


using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex::dataset;
using namespace csapex::dataset::people;
namespace fs = boost::filesystem;


void ROIImporter::setup(csapex::NodeModifier& node_modifier)
{
    in_timestamp_ = node_modifier.addInput<AnyMessage>("Timestamp");
    out_rois_ = node_modifier.addOutput<GenericVectorMessage, RoiMessage>("ROIs");
}

void ROIImporter::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(
            param::ParameterFactory::declareDirectoryInputPath("roi_dir", "", ""),
            [this](param::Parameter* param)
            {
                loadRois(param->as<std::string>());
            });
}

void ROIImporter::process()
{
    static const auto EMPTY_MESSAGE = std::make_shared<RoiMessageVector>();

    TokenDataConstPtr message = msg::getMessage(in_timestamp_);
    auto casted = msg::message_cast<const Message>(message);

    std::uint64_t timestamp = 0;
    if (casted)
        timestamp = casted->stamp_micro_seconds;

    auto find = rois_.find(timestamp);
    if (find != rois_.end())
        msg::publish(out_rois_, find->second);
    else
        msg::publish(out_rois_, EMPTY_MESSAGE);
}

void ROIImporter::loadRois(const std::string& roi_dir_str)
{
    rois_.clear();

    const fs::path roi_dir(roi_dir_str);

    for (fs::directory_iterator file_itr(roi_dir); file_itr != fs::directory_iterator(); ++file_itr)
    {
        fs::path file = file_itr->path();
        if (fs::extension(file) != ".yaml")
            continue;

        std::string id = fs::basename(file);

        YAML::Node yaml = YAML::LoadFile(file.string());
        if (!yaml.IsMap())
            throw std::runtime_error("Inconsistent content for '" + id + "' - roi yaml invalid");

        auto timestamp = std::stoull(id);

        auto width = yaml["img_d_w"].as<int>();
        auto height = yaml["img_d_h"].as<int>();
        cv::Rect crop_mask(0, 0, width, height);

        YAML::Node rois = yaml["rois"];
        if (!rois.IsSequence())
            throw std::runtime_error("Inconsistent content for '" + id + "' - roi yaml invalid");

        auto parsed_rois = std::make_shared<RoiMessageVector>();
        for (YAML::iterator itr = rois.begin(); itr != rois.end(); ++itr)
        {
            const YAML::Node& node = *itr;
            if (!node.IsMap())
                throw std::runtime_error("Inconsistent content for '" + id + "' - roi yaml invalid");

            cv::Rect rect;
            rect.x      = node["x_d"].as<int>();
            rect.y      = node["y_d"].as<int>();
            rect.width  = node["w_d"].as<int>();
            rect.height = node["h_d"].as<int>();

            rect = rect & crop_mask;
            if (rect.height <= 0
                || rect.width <= 0)
                continue;

            int vis = node["vis"].as<int>();
//            if (vis != HUMAN
//                && vis != HUMAN_PART)
//                continue;

            RoiMessage roi;
            roi.value.setRect(rect);
            roi.value.setClassification(vis);
            auto cl = ClassificationColors.find(vis);
            if (cl != ClassificationColors.end())
                roi.value.setColor(cl->second);
            else
                roi.value.setColor(ClassificationColors.at(UNKNOWN));

            parsed_rois->push_back(std::move(roi));
        }

        rois_.emplace(timestamp, std::move(parsed_rois));
    }
}
