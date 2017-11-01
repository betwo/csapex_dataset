#pragma once

#include <csapex/model/node.h>
#include <csapex_opencv/roi_message.h>

#include <unordered_map>

namespace csapex { namespace dataset {

class ROIImporter : public Node
{
public:
    void setup(csapex::NodeModifier &node_modifier) override;
    void setupParameters(Parameterizable &parameters) override;
    void process() override;

private:
    void loadRois(const std::string& roi_dir);

private:
    Input* in_timestamp_;
    Output* out_rois_;

    using RoiMessageVector = std::vector<connection_types::RoiMessage>;
    std::unordered_map<std::uint64_t, std::shared_ptr<RoiMessageVector>> rois_;
};

}}
