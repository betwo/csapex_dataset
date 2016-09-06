#pragma once

#include <csapex/model/node.h>
#include <csapex_opencv/roi_message.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex_evaluation/confusion_matrix_message.h>

namespace csapex
{
class MatchingROIEvaluator : public Node
{
public:
    MatchingROIEvaluator();
    void setupParameters(csapex::Parameterizable& parameters) override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void process() override;

private:
    std::set<int> parse_labels(const std::string& label_string);
    void updateLabels();

    double calc_overlap(const cv::Rect& gt, const cv::Rect& pred);
    bool is_positive(const Roi& roi);
    bool is_partial(const Roi& roi);

    void reset();

private:
    csapex::Input* in_ground_truth_;
    csapex::Input* in_prediction_;
    csapex::Input* in_rejected_;
    csapex::Input* in_cloud_;
    csapex::Output* out_confusion_;

    std::set<int> negative_labels_;
    std::set<int> positive_labels_;
    std::set<int> partial_labels_;
    double min_overlap_;
    bool filter_distance_;
    std::pair<double, double> filter_distance_range_;
    int filter_distance_min_count_;

    ConfusionMatrix confusion_;
};
}
