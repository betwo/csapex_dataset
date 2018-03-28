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
    std::vector<MetaEntry>::const_iterator  data_iterator_;

    bool                                    playing_;
    param::OutputProgressParameter::Ptr     play_progress_;
    param::OutputTextParameter::Ptr         current_frame_;

    Output*                                 output_pointcloud_;
    Output*                                 output_mask_;
    Output*                                 output_rgb_;
    Event*                                  event_finished_;

    bool                                    param_start_instantly_;

    void import(const boost::filesystem::path& path);
    void startPlay();
    void triggerStartPlayEvent();

};

}
}
}
