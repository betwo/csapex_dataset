#ifndef IMPORTINRIATRAINDATA_H
#define IMPORTINRIATRAINDATA_H

#include "import_inria.hpp"

#include <csapex/model/tickable_node.h>
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

class ImportINRIAData : public csapex::TickableNode
{
public:
    void setup(NodeModifier &node_modifier) override;
    void setupParameters(Parameterizable &parameters) override;
    void process() override;
    void tick() override;

private:
//    enum ImportState {None, Failed, Imported};
//    std::atomic<ImportState> import_state;

    Instance::Set samples_;

    csapex::Output *out_image_;
    csapex::Output *out_rois_;

    double                     rate_;
    int                        neg_rng_seed_;
    bool                       play_;
    std::size_t                play_index_;
    param::ValueParameter::Ptr param_play_;
    param::RangeParameter::Ptr param_play_index_;
    cv::Size                   neg_window_size_;


    void import();
    void play();

};

}

}

}

#endif // IMPORTINRIATRAINDATA_H
