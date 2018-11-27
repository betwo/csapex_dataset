#include "evaluation.hpp"

#include <csapex/model/node_modifier.h>
#include <csapex/model/token_data.h>
#include <csapex/param/parameter_factory.h>

#include <csapex/msg/io.h>
#include <csapex/msg/message.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_point_cloud/msg/indeces_message.h>

#include <csapex/utility/register_apex_plugin.h>
CSAPEX_REGISTER_CLASS(csapex::dataset::biwi::Evaluation, csapex::Node)

using namespace csapex;
using namespace dataset;
using namespace biwi;

using namespace csapex::connection_types;

Evaluation::Evaluation()
{
    reset();
}

void Evaluation::setup(NodeModifier &node_modifier)
{
    in_clusters_ = node_modifier.addInput<GenericVectorMessage, pcl::PointIndices>("Detected Clusters");
    in_labels_   = node_modifier.addInput<GenericVectorMessage, pcl::PointIndices>("Labeled Clusters");

    out_tpr_ = node_modifier.addOutput<double>("TPR");
    out_fdr_ = node_modifier.addOutput<double>("FDR");
    out_iou_ = node_modifier.addOutput<double>("IoU");
}

void Evaluation::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(param::factory::declareTrigger("reset"), [this](param::Parameter*) { reset(); });

    parameters.addParameter(param::factory::declareOutputText("Count"));
    parameters.addParameter(param::factory::declareOutputText("TPR"));
    parameters.addParameter(param::factory::declareOutputText("FDR"));
    parameters.addParameter(param::factory::declareOutputText("IoU"));

}

void Evaluation::process()
{
    const std::shared_ptr<std::vector<pcl::PointIndices> const> clusters =
            msg::getMessage<GenericVectorMessage, pcl::PointIndices>(in_clusters_);
    const std::shared_ptr<std::vector<pcl::PointIndices> const> labels =
            msg::getMessage<GenericVectorMessage, pcl::PointIndices>(in_labels_);

    if (labels->size() != 1)
        return;

    // get ground truth person
    const pcl::PointIndices &gt_person = labels->front();
    const std::size_t positives = gt_person.indices.size();

    auto find = [&gt_person](const int &cluster_index) {
        return std::find(gt_person.indices.begin(), gt_person.indices.end(), cluster_index) != gt_person.indices.end();
    };

    // estimate tpr and fdr for all detected clusters
    double tpr = 0.0;
    double fdr = 0.0;
    double iou = 0.0;
    for (const pcl::PointIndices &cluster : *clusters) {
        std::size_t true_positives  = 0u;
        std::size_t false_positives = 0u;
        if (cluster.indices.size() == 0u)
            continue;

        // increase number of true positives and false positives
        for (const int &cluster_index : cluster.indices) {
            if (find(cluster_index))
                ++true_positives;
            else
                ++false_positives;
        }

        // tpr and fdr of the current cluster
        const double tpr_cluster = static_cast<double>(true_positives) / static_cast<double>(positives);
        const double fdr_cluster = static_cast<double>(false_positives) / static_cast<double>(true_positives + false_positives);
        const double iou_cluster = static_cast<double>(true_positives) / static_cast<double>(positives + false_positives);

        // update tpr and fdr for the current frame
        if (tpr_cluster > tpr) {
            tpr = tpr_cluster;
            fdr = fdr_cluster;
            iou = iou_cluster;
        }
    }

    // update global means
    tpr_ = mean(tpr_, tpr, count_);
    fdr_ = mean(fdr_, fdr, count_);
    iou_ = mean(iou_, iou, count_);
    ++count_;

    // plot info
    setParameter("Count", "Count: " + std::to_string(count_));
    setParameter("TPR",   "TPR: "   + std::to_string(tpr_));
    setParameter("FDR",   "FDR: "   + std::to_string(fdr_));
    setParameter("IoU",   "IoU: "   + std::to_string(iou_));

    //// DEBUG
    ainfo << "FRAME " << count_ << "\n"
    << "TPR = " << tpr_ << "\n"
    << "FDR = " << fdr_ << "\n"
    << "IoU = " << iou_ << "\n";
    //// DEBUG

    msg::publish(out_fdr_, fdr_);
    msg::publish(out_tpr_, tpr_);
    msg::publish(out_iou_, iou_);
}

void Evaluation::reset()
{
    count_ = 0u;
    tpr_   = 0.0;
    fdr_   = 0.0;
    iou_   = 0.0;
}

double Evaluation::mean(const double &prev, const double &curr, const std::size_t &n) const
{
    return (prev * static_cast<double>(n) + curr) / static_cast<double>(n+1);
}
