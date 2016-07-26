#pragma once

#include <csapex/model/node.h>

namespace csapex
{
namespace dataset
{
    class PeopleDatasetExporter : public csapex::Node
    {
    public:
        PeopleDatasetExporter();

        void setupParameters(csapex::Parameterizable& parameters) override;
        void setup(csapex::NodeModifier& node_modifier) override;
        void process() override;

    private:
        csapex::Input* in_depth_;
        csapex::Input* in_visual_;
        csapex::Input* in_pointcloud_;
        csapex::Input* in_rois_;
        csapex::Input* in_rois_gen_;

        std::string directory_;
        int class_;
        bool only_with_rois_;
    };
}
}
