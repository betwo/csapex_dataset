#pragma once

#include <csapex/model/node.h>

namespace csapex {
namespace dataset {
namespace biwi {
class CSAPEX_EXPORT_PLUGIN Evaluation : public csapex::Node
{
public:
    Evaluation();
    virtual void setup(csapex::NodeModifier &node_modifier) override;
    virtual void setupParameters(csapex::Parameterizable &parameters) override;
    virtual void process() override;

private:
    Input*      in_clusters_;
    Input*      in_labels_;
    Output*     out_tpr_;
    Output*     out_fdr_;
    Output*     out_iou_;

    std::size_t count_;
    double      tpr_;
    double      fdr_;
    double      iou_;

    virtual void reset() override;
    double mean(const double &prev, const double &curr, const std::size_t &n) const;
};
}
}
}
