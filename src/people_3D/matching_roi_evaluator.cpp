#include "matching_roi_evaluator.hpp"

#include <csapex/msg/io.h>
#include <csapex/msg/output.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/model/node_modifier.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/interval_parameter.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_vector_message.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

CSAPEX_REGISTER_CLASS(csapex::MatchingROIEvaluator, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

MatchingROIEvaluator::MatchingROIEvaluator()
{
    reset();
}

void MatchingROIEvaluator::setupParameters(csapex::Parameterizable& parameters)
{
    parameters.addParameter(param::ParameterFactory::declareTrigger("reset"),
                            [this](param::Parameter*) { reset(); });

    parameters.addParameter(param::ParameterFactory::declareValue<std::string>("labels/negative",
                                                                               param::ParameterDescription("Comma separated list of negative labels"),
                                                                               "0"),
                            std::bind(&MatchingROIEvaluator::updateLabels, this));
    parameters.addParameter(param::ParameterFactory::declareValue<std::string>("labels/positive",
                                                                               param::ParameterDescription("Comma separated list of positive labels"),
                                                                               "1"),
                            std::bind(&MatchingROIEvaluator::updateLabels, this));
    parameters.addParameter(param::ParameterFactory::declareValue<std::string>("labels/partial",
                                                                               param::ParameterDescription("Comma separated list of partial positive labels"),
                                                                               "2"),
                            std::bind(&MatchingROIEvaluator::updateLabels, this));

    parameters.addParameter(param::ParameterFactory::declareRange("min_overlap", 0.0, 1.0, 0.6, 0.01),
                            min_overlap_);

    parameters.addParameter(param::ParameterFactory::declareBool("filter/distance", true),
                            filter_distance_);

    auto cond_filter_distance = [this]() { return readParameter<bool>("filter/distance"); };

    parameters.addConditionalParameter(param::ParameterFactory::declareInterval("distance/range", 0.0, 100.0, 0.5, 7.5, 0.01),
                                       cond_filter_distance,
                                       filter_distance_range_);
    parameters.addConditionalParameter(param::ParameterFactory::declareRange("distance/min_count", 0, 1920*1080, 0, 1),
                                       cond_filter_distance,
                                       filter_distance_min_count_);
}

void MatchingROIEvaluator::setup(csapex::NodeModifier& node_modifier)
{
    in_ground_truth_  = node_modifier.addInput<GenericVectorMessage, RoiMessage>("ground truth");
    in_prediction_ = node_modifier.addInput<GenericVectorMessage, RoiMessage>("prediction");
    in_rejected_ = node_modifier.addOptionalInput<GenericVectorMessage, RoiMessage>("rejected");
    in_cloud_ = node_modifier.addOptionalInput<PointCloudMessage>("point cloud");
    out_confusion_ = node_modifier.addOutput<ConfusionMatrixMessage>("confusion");
}

double MatchingROIEvaluator::calc_overlap(const cv::Rect& gt, const cv::Rect& pred)
{
    return (pred & gt).area() / std::max<double>(gt.area(), pred.area());
}

bool MatchingROIEvaluator::is_positive(const Roi& roi)
{
    return positive_labels_.find(roi.classification()) != positive_labels_.end();
}

bool MatchingROIEvaluator::is_partial(const Roi& roi)
{
    return partial_labels_.find(roi.classification()) != partial_labels_.end();
}

namespace
{
struct DistanceFilter : public boost::static_visitor<void>
{
    DistanceFilter(std::vector<RoiMessage>& ground_truth,
                   const std::pair<double, double>& distance_range,
                   int min_points) :
        ground_truth_(ground_truth),
        distance_range_(distance_range),
        min_points_(min_points)
    {

    }

    template<typename PointCloudT>
    void operator()(PointCloudT cloud) const
    {
        using PointT = typename PointCloudT::element_type::PointType;

        const auto is_finite = [](const PointT& pt)
        {
            return std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z);
        };
        const auto is_zero = [](const PointT& pt)
        {
            return pt.x == 0 && pt.y == 0 && pt.z == 0;
        };

        double mean_depth = 0;
        std::size_t valid_points = 0;

        for (RoiMessage& ground_truth : ground_truth_)
        {
            cv::Rect region = ground_truth.value.rect();

            for (int y = region.y; y  < region.y + region.height; ++y)
                for (int x = region.x; x  < region.x + region.width; ++x)
                {
                    const PointT& pt = cloud->at(x, y);
                    if (!is_finite(pt))
                        continue;
                    if (is_zero(pt))
                        continue;

                    mean_depth += pt.x;
                    ++valid_points;
                }

            mean_depth /= std::max<std::size_t>(1, valid_points);

            bool valid = true;
            valid = valid && (mean_depth >= distance_range_.first && mean_depth <= distance_range_.second);
            valid = valid && (valid_points >= min_points_);

            if (!valid)
                ground_truth.value.setClassification(2); // todo: fucking hardcoded value
        }
    }

private:
    std::vector<RoiMessage>& ground_truth_;
    const std::pair<double, double>& distance_range_;
    std::size_t min_points_;
};
}

void MatchingROIEvaluator::process()
{
    std::shared_ptr<std::vector<RoiMessage> const> in_ground_truth = msg::getMessage<GenericVectorMessage, RoiMessage>(in_ground_truth_);
    std::shared_ptr<std::vector<RoiMessage> const> in_prediction = msg::getMessage<GenericVectorMessage, RoiMessage>(in_prediction_);

    std::vector<RoiMessage> rois_prediction;
    rois_prediction.assign(in_prediction->begin(), in_prediction->end());

    std::vector<RoiMessage> rois_ground_truth;
    rois_ground_truth.assign(in_ground_truth->begin(), in_ground_truth->end());

    std::vector<RoiMessage> rois_rejected;
    if (msg::hasMessage(in_rejected_))
    {
        std::shared_ptr<std::vector<RoiMessage> const> in_rejected = msg::getMessage<GenericVectorMessage, RoiMessage>(in_rejected_);
        rois_rejected.assign(in_rejected->begin(), in_rejected->end());
    }

    if (filter_distance_)
    {
        PointCloudMessage::ConstPtr in_cloud(msg::getMessage<PointCloudMessage>(in_cloud_));

        DistanceFilter filter(rois_ground_truth, filter_distance_range_, filter_distance_min_count_);
        boost::apply_visitor(filter, in_cloud->value);
    }

    std::vector<bool> used_predictions;
    std::generate_n(std::back_inserter(used_predictions), rois_prediction.size(), []() { return false; });
    std::vector<bool> used_ground_truth;
    std::generate_n(std::back_inserter(used_ground_truth), rois_ground_truth.size(), []() { return false; });
    // check ground truth -> predictions
    // -> true positive, false negative
    for (int i = 0; i < rois_ground_truth.size(); ++i)
    {
        const RoiMessage& roi_ground_truth = rois_ground_truth[i];

        double max_overlap = std::numeric_limits<double>::min();
        int max_idx = -1;
        for (std::size_t j = 0; j < rois_prediction.size(); ++j)
        {
            if (used_predictions[j])
                continue;

            const RoiMessage& roi_prediction = rois_prediction[j];

            double overlap = calc_overlap(roi_ground_truth.value.rect(), roi_prediction.value.rect());
            if (overlap < min_overlap_)
                continue;

            if (!is_positive(roi_prediction.value))
                continue;

            if (overlap > max_overlap)
            {
                max_overlap = overlap;
                max_idx = j;
            }
        }


        if (max_idx > -1)
        {
            // found match
            confusion_.reportClassification(1, 1);
            used_predictions[max_idx] = true;
            used_ground_truth[i] = true;
        }
        else
        {
            // no match and not partial
            if (!is_partial(roi_ground_truth.value))
                confusion_.reportClassification(1, 0);
        }
    }

    // check prediction -> ground truth
    // -> false positive, true negative
    for (int i = 0; i < rois_prediction.size(); ++i)
    {
        if (used_predictions[i])
            continue;

        const RoiMessage& roi_prediction = rois_prediction[i];

        bool matched = false;
        for (std::size_t j = 0; j < rois_ground_truth.size(); ++j)
        {
            if (used_ground_truth[i])
                continue;

            const RoiMessage& roi_ground_truth = rois_ground_truth[j];

            double overlap = calc_overlap(roi_ground_truth.value.rect(), roi_prediction.value.rect());
            if (overlap >= min_overlap_)
            {
                matched = true;
                break;
            }
        }

        if (!matched)
        {
            // not matched -> false positive, true negative
            if (is_positive(roi_prediction.value))
                confusion_.reportClassification(0, 1);
            else
                confusion_.reportClassification(0, 0);
        }
    }

    // check rejected -> ground truth
    for (const RoiMessage& roi_rejected : rois_rejected)
    {
        bool matched = false;
        for (std::size_t j = 0; j < rois_ground_truth.size(); ++j)
        {
            const RoiMessage& roi_ground_truth = rois_ground_truth[j];

            if (is_partial(roi_ground_truth.value))
                continue;

            double overlap = calc_overlap(roi_ground_truth.value.rect(), roi_rejected.value.rect());
            if (overlap > min_overlap_)
            {
                matched = true;
                break;
            }
        }

        if (matched)
            confusion_.reportClassification(1, 0);
        else
            confusion_.reportClassification(0, 0);
    }

    ConfusionMatrixMessage::Ptr confusion_msg = std::make_shared<ConfusionMatrixMessage>();
    confusion_msg->confusion = confusion_;
    confusion_msg->confusion.threshold = min_overlap_;
    msg::publish(out_confusion_, confusion_msg);
}

std::set<int> MatchingROIEvaluator::parse_labels(const std::string& label_string)
{
    std::set<int> labels;

    std::vector<std::string> string_labels;
    boost::algorithm::split(string_labels, label_string, boost::is_any_of(","));
    std::transform(string_labels.begin(), string_labels.end(), std::inserter(labels, labels.begin()),
                   [](const std::string& text)
    {
        return boost::lexical_cast<int>(text);
    });

    return labels;
}

void MatchingROIEvaluator::reset()
{
    confusion_ = ConfusionMatrix();
    confusion_.initializeClass(0);
    confusion_.initializeClass(1);
    confusion_.class_names[0] = "non-human";
    confusion_.class_names[1] = "human";
}


void MatchingROIEvaluator::updateLabels()
{
    try
    {
        positive_labels_ = parse_labels(readParameter<std::string>("labels/positive"));
        negative_labels_ = parse_labels(readParameter<std::string>("labels/negative"));
        partial_labels_ = parse_labels(readParameter<std::string>("labels/partial"));
    }
    catch (boost::bad_lexical_cast ex)
    {
        node_modifier_->setError("Invalid label in sequence, must be an integer");
        return;
    }

    {
        std::set<int> all;
        std::copy(positive_labels_.begin(), positive_labels_.end(), std::inserter(all, all.begin()));
        std::copy(negative_labels_.begin(), negative_labels_.end(), std::inserter(all, all.begin()));
        std::copy(partial_labels_.begin(), partial_labels_.end(), std::inserter(all, all.begin()));

        if (all.size() != positive_labels_.size() + negative_labels_.size() + partial_labels_.size())
        {
            node_modifier_->setError("Label sets must be distinct");
            return;
        }
    }

    node_modifier_->setNoError();
}
