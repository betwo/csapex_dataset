#pragma once

#include <csapex/model/node.h>
#include <csapex_opencv/roi_message.h>

#include <unordered_map>

namespace csapex { namespace dataset {

class ROIExporter : public Node
{
public:
    void setup(csapex::NodeModifier &node_modifier) override;
    void setupParameters(Parameterizable &parameters) override;
    void process() override;

private:
    Input* in_timestamp_;
    Input* in_size_;
    Input* in_rois_;

    std::string config_roi_dir_;
};

}}
