#ifndef IMPORTINRIATRAINDATA_H
#define IMPORTINRIATRAINDATA_H

#include "import_inria.hpp"

#include <csapex/model/node.h>
#include <csapex/param/range_parameter.h>
#include <csapex/param/value_parameter.h>

#include <thread>
#include <atomic>

namespace csapex
{
namespace dataset
{
namespace people
{

class ImportINRIAData : public Node
{
public:
    void setup(NodeModifier &node_modifier) override;
    void setupParameters(Parameterizable &parameters) override;

    bool canProcess() const override;
    void process() override;

private:

    Instance::Set samples_;

    csapex::Output *out_image_;
    csapex::Output *out_rois_;

    csapex::Event  *play_started_;
    csapex::Event  *play_finished_;
    csapex::Event  *play_stopped_;

    csapex::Slot   *play_start_;
    csapex::Slot   *play_stop_;
    csapex::Slot   *play_reset_;

    int                        neg_rng_seed_;
    bool                       neg_do_sample_;
    bool                       play_;
    std::size_t                play_index_;
    param::ValueParameter::Ptr param_play_;
    param::RangeParameter::Ptr param_play_index_;
    cv::Size                   neg_window_size_;
    std::string                path_;

    void import();
    void play();
    void slotPlay();
    void slotReset();
    void slotStop();

};

}

}

}

#endif // IMPORTINRIATRAINDATA_H
