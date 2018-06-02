#pragma once

#include <csapex/model/node.h>
#include <csapex/param/output_progress_parameter.h>
#include <csapex/param/output_text_parameter.h>

/// SYSTEM
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <random>


namespace csapex
{
namespace dataset
{
namespace biwi {

class RGBDIDImporter : public csapex::Node
{
private:
    struct MetaEntry
    {
        std::string                 id;
        boost::filesystem::path     path_rgb;
        boost::filesystem::path     path_depth;
        boost::filesystem::path     path_mask;
    };

public:
    RGBDIDImporter();

    virtual void setup(csapex::NodeModifier &node_modifier) override;
    virtual void setupParameters(csapex::Parameterizable &parameters) override;
    virtual bool canProcess() const override;
    virtual void process() override;

private:
    std::vector<MetaEntry>                  data_;
    std::vector<MetaEntry>::const_iterator  data_walking_begin_;
    std::vector<MetaEntry>::const_iterator  data_iterator_;
    std::vector<MetaEntry>::const_iterator  data_end_;

    std::size_t                             data_entries_to_play_;

    std::string                             types_to_play_;
    bool                                    playing_;
    param::OutputProgressParameter::Ptr     play_progress_;
    param::OutputTextParameter::Ptr         current_frame_;

    std::size_t                             progress_;
    std::size_t                             iteration_;

    Output*                                 output_pointcloud_;
    Output*                                 output_mask_;
    Output*                                 output_indices_;
    Output*                                 output_indices_msgs_;
    Output*                                 output_rgb_;
    Output*                                 output_depth_;
    Event*                                  event_finished_;
    double                                  depth_deviation_;
    double                                  cluster_depth_maximum_;

    bool                                    param_start_instantly_;
    bool                                    ignore_errors_;

    void import(const boost::filesystem::path& path);
    void startPlay();
    void triggerStartPlayEvent();

};

}
}
}
