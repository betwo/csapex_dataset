#pragma once

#include "common/dataset.hpp"
#include <csapex/model/node.h>
#include <csapex_opencv/roi_message.h>
#include <unordered_map>

namespace csapex { namespace dataset { namespace sand {

class SandDatasetExporterSlim : public Node
{
public:
    SandDatasetExporterSlim();

    void setup(NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    bool canProcess() const override;
    void process() override;

private:
    void createDataset(boost::filesystem::path reference_index_file,
                       boost::filesystem::path index_file);
    void saveDataset();

private:
    std::unique_ptr<Dataset> reference_dataset_;
    std::unique_ptr<Dataset> dataset_;
    boost::filesystem::path annotation_dir_;
    boost::filesystem::path index_file_;
    bool save_empty_rois_;

    Input* in_pointcloud_;
    Input* in_rois_;
};

}}}
