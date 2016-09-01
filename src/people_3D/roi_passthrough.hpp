#pragma once

#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex
{
class ROIPassthrough : public Node
{
public:
    void setupParameters(csapex::Parameterizable& parameters) override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

private:
    csapex::Input* in_cloud_;
    csapex::Input* in_rois_;
    csapex::Output* out_rois_;

    std::pair<double, double> distance_;
    int min_point_count_;
};
}
