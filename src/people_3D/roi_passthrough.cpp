#include "roi_passthrough.hpp"

#include <csapex/msg/io.h>
#include <csapex/msg/output.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/model/node_modifier.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/interval_parameter.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_opencv/roi_message.h>

CSAPEX_REGISTER_CLASS(csapex::ROIPassthrough, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

void ROIPassthrough::setupParameters(csapex::Parameterizable& parameters)
{
    parameters.addParameter(param::ParameterFactory::declareInterval("distance", -100.0, 100.0, 0.5, 5.0, 0.01),
                            distance_);
    parameters.addParameter(param::ParameterFactory::declareRange("min_point_count", 0, 100000, 0, 1),
                            min_point_count_);
}

void ROIPassthrough::setup(csapex::NodeModifier& node_modifier)
{
    in_cloud_ = node_modifier.addInput<PointCloudMessage>("point cloud");
    in_rois_  = node_modifier.addInput<GenericVectorMessage, RoiMessage>("rois");
    out_rois_ = node_modifier.addOutput<GenericVectorMessage, RoiMessage>("filtered rois");
}

void ROIPassthrough::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_cloud_));
    boost::apply_visitor (PointCloudMessage::Dispatch<ROIPassthrough>(this, msg), msg->value);
}

namespace
{
    template<typename PointT>
    inline bool check_filter(const RoiMessage &roi,
                             const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
                             const std::pair<double, double>& distance,
                             int min_point_count)
    {
        std::size_t valid_points = 0;
        double      mean_depth = 0.0;
        cv::Rect rect = roi.value.rect() & cv::Rect(0, 0, cloud->width, cloud->height);

        for(int i = rect.y ; (i < rect.y + rect.height); ++i) {
            for(int j = rect.x ; (j < rect.x + rect.width); ++j) {
                const PointT& p = cloud->at(j, i);
                if(std::isnan(p.x) ||
                        std::isnan(p.y) ||
                            std::isnan(p.z))
                    continue;

                if(p.x == 0.f &&
                        p.y == 0.f &&
                            p.z == 0.f)
                    continue;

                mean_depth += p.x;
                ++valid_points;
            }
        }

        mean_depth /= fmax(1, valid_points);
        bool constraints_apply = (int) valid_points > min_point_count &&
                                       mean_depth >= distance.first &&
                                       mean_depth <= distance.second;

        return constraints_apply;
    }
}

template<typename PointT>
void ROIPassthrough::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    std::shared_ptr<std::vector<RoiMessage> const> rois_msg = msg::getMessage<GenericVectorMessage, RoiMessage>(in_rois_);

    std::shared_ptr<std::vector<RoiMessage>> out_rois = std::make_shared<std::vector<RoiMessage>>();
    for (const RoiMessage& msg : *rois_msg)
    {
        if (check_filter<PointT>(msg, cloud, distance_, min_point_count_))
            out_rois->push_back(msg);
    }

    msg::publish<GenericVectorMessage, RoiMessage>(out_rois_, out_rois);
}
