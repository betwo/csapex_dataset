#pragma once

#include "common/dataset.hpp"
#include <csapex/model/node.h>
#include <csapex/param/output_progress_parameter.h>
#include <csapex/param/output_text_parameter.h>
#include <csapex_opencv/roi_message.h>
#include <unordered_map>
#include <csapex/param/interval_parameter.h>

namespace csapex { namespace dataset { namespace sand {

class SandDatasetImporter : public Node
{
public:
    SandDatasetImporter();

    void setup(NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    bool canProcess() const override;
    void process() override;

private:
    void import(const boost::filesystem::path& path);
    void triggerStartPlayEvent();
    void startPlay();
    void generateNegativeSamples();

private:
    std::unique_ptr<Dataset> dataset_;
    std::unordered_map<uint64_t, std::vector<connection_types::RoiMessage>> negative_rois_;

    Dataset::const_iterator play_itr_;
    bool playing_;
    param::OutputProgressParameter::Ptr play_progress_;
    param::OutputTextParameter::Ptr current_frame_;

    Output*                       output_pointcloud_;
    Output*                       output_rois_;
    Event*                        event_finished_;
    param::IntervalParameter::Ptr interval_;
    std::pair<int,int>            interval_boundaries_;

    bool param_start_instantly_;
    int param_load_classes_;
    bool param_generate_negative_;
    int param_generate_seed_;
    double param_negative_ratio_;
    std::pair<int, int> param_negative_width_;
    std::pair<int, int> param_negative_height_;
    int param_negative_class_;
    bool param_no_overlap_;
    int param_check_overlap_classes_;
};

}}}
