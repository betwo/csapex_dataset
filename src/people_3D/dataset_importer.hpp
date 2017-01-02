#pragma once

#include "dataset_common.hpp"

#include <csapex/model/node.h>

/// SYSTEM
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <random>

namespace csapex
{
namespace dataset
{

class PeopleDatasetImporter : public csapex::Node
{
    enum Ratio {RATIO_1_TO_2, RATIO_1_TO_1, RATIO_FREE};

public:
    PeopleDatasetImporter();

    virtual void setup(csapex::NodeModifier &node_modifier) override;
    virtual void setupParameters(csapex::Parameterizable &parameters) override;
    virtual bool canProcess() const override;
    virtual void process() override;

private:
    /// I/O
    csapex::Output  *out_pointcloud_;
    csapex::Output  *out_depth_image_;
    csapex::Output  *out_bgr_image_;
    csapex::Output  *out_rois_;
    csapex::Event   *tri_finished_;
    /// PAREMTERS
    csapex::param::Parameter::Ptr action_import_;
    csapex::param::Parameter::Ptr action_play_start_;
    csapex::param::Parameter::Ptr action_play_stop_;
    csapex::param::Parameter::Ptr param_data_positive_;
    csapex::param::Parameter::Ptr param_data_negative_;
    csapex::param::Parameter::Ptr param_sample_negative_range_;
    csapex::param::Parameter::Ptr param_sample_positive_range_;
    csapex::param::Parameter::Ptr param_sample_positive_count_;
    csapex::param::Parameter::Ptr param_sample_negative_count_;
    csapex::param::Parameter::Ptr param_sample_negative_rois_;
    csapex::param::Parameter::Ptr param_sample_negative_size_min_;
    csapex::param::Parameter::Ptr param_sample_negative_size_max_;
    csapex::param::Parameter::Ptr param_sample_negative_size_ratio_;
    csapex::param::Parameter::Ptr param_sample_random_seed_;
    csapex::param::Parameter::Ptr param_sample_exclusive_;
    csapex::param::Parameter::Ptr param_sample_sequential_;
    csapex::param::Parameter::Ptr param_play_immediate_;
    csapex::param::Parameter::Ptr param_play_frame_;
    csapex::param::Parameter::Ptr progress_;
    /// STATE
    bool playing_;
    std::size_t play_index_;
    std::vector<people::MetaEntry> play_entries_;
    std::vector<people::MetaEntry> positive_entries_;
    std::vector<people::MetaEntry> negative_entries_;
    std::minstd_rand random_;

private:
    void import();
    void start_play();
    void stop_play();
};

}
}
