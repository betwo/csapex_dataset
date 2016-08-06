/// HEADER
#include "dataset_importer.hpp"

/// PROJECT
#include <csapex/model/connector_type.h>
#include <csapex/model/node_modifier.h>
#include <csapex/signal/event.h>
#include <csapex/msg/io.h>
#include <csapex/msg/message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/factory/message_factory.h>
#include <csapex/msg/end_of_sequence_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/path_parameter.h>
#include <csapex/param/interval_parameter.h>
#include <csapex/param/output_progress_parameter.h>
#include <csapex/param/range_parameter.h>
#include <csapex/profiling/timer.h>
#include <csapex/profiling/interlude.hpp>

#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_vision/roi_message.h>
#include <csapex_core_plugins/vector_message.h>

/// SYSTEM
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/assign.hpp>

CSAPEX_REGISTER_CLASS(csapex::dataset::PeopleDatasetImporter, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex::dataset;
using namespace csapex::dataset::people;

PeopleDatasetImporter::PeopleDatasetImporter() :
    playing_(false)
{
}

void PeopleDatasetImporter::setup(csapex::NodeModifier& node_modifier)
{
    out_pointcloud_  = node_modifier.addOutput<PointCloudMessage>("Pointcloud");
    out_depth_image_ = node_modifier.addOutput<CvMatMessage>("Depth Image");
    out_bgr_image_   = node_modifier.addOutput<CvMatMessage>("BGR Image");
    out_rois_        = node_modifier.addOutput<GenericVectorMessage, RoiMessage>("ROIs");
    tri_finished_    = node_modifier.addEvent("finished");
}

void PeopleDatasetImporter::setupParameters(csapex::Parameterizable& parameters)
{
    action_import_ = param::ParameterFactory::declareTrigger("action/import",
                                            param::ParameterDescription("Load and prepare datasets"));
    action_play_start_ = param::ParameterFactory::declareTrigger("action/start",
                                            param::ParameterDescription("Start playing dataset"));
    action_play_stop_ = param::ParameterFactory::declareTrigger("action/stop",
                                           param::ParameterDescription("Stop playing dataset"));

    param_data_positive_ = param::ParameterFactory::declareDirectoryInputPath("data/positive",
                                                       param::ParameterDescription("Dataset directory for positive samples"),
                                                       "");
    param_data_negative_ = param::ParameterFactory::declareDirectoryInputPath("data/negative",
                                                       param::ParameterDescription("Dataset directory for negative samples"),
                                                       "");
    param_sample_positive_count_ = param::ParameterFactory::declareRange("sample/positive count",
                                          param::ParameterDescription("Number of positive samples"),
                                          0, 100000, 1000, 1);
    param_sample_negative_count_ = param::ParameterFactory::declareRange("sample/negative count",
                                          param::ParameterDescription("Number of negative samples"),
                                          0, 100000, 1000, 1);
    param_sample_negative_rois_ = param::ParameterFactory::declareBool("sample/negative read rois",
                                                                       param::ParameterDescription("Use ROIs instead of random sampling"),
                                                                       false);
    param_sample_negative_size_min_ = param::ParameterFactory::declareRange("sample/negative size min",
                                          param::ParameterDescription("Minimum window width for negative samples"),
                                          1, 640, 32, 1);
    param_sample_negative_size_max_ = param::ParameterFactory::declareRange("sample/negative size max",
                                          param::ParameterDescription("Maximum window width for negative samples"),
                                          1, 640, 128, 1);
    param_sample_negative_size_ratio_ = param::ParameterFactory::declareParameterSet<int>("sample/negative size ratio",
                                                      param::ParameterDescription("Window ratio (width to height) for negative samples"),
                                                      {{"1:2", RATIO_1_TO_2}, {"1:1", RATIO_1_TO_1}, {"free", RATIO_FREE}},
                                                      RATIO_1_TO_2);
    param_sample_positive_range_ = param::ParameterFactory::declareInterval("sample/positive range",
                                          param::ParameterDescription("Range for positive samples"),
                                          0, 100000, 0, 100000, 1);
    param_sample_negative_range_ = param::ParameterFactory::declareInterval("sample/negative range",
                                          param::ParameterDescription("Range for negative samples"),
                                          0, 100000, 0, 100000, 1);
    param_sample_random_seed_ = param::ParameterFactory::declareValue("sample/random seed",
                                          param::ParameterDescription("Seed for sample RNG"),
                                          0);
    param_sample_exclusive_ = param::ParameterFactory::declareBool("sample/exclusive",
                                                                   param::ParameterDescription("Sample positive/negative samples exclusively from positive or negative set"),
                                                                   true);
    param_sample_sequential_ = param::ParameterFactory::declareBool("sample/sequential",
                                                                    param::ParameterDescription("Select samples in time order, instead of random order"),
                                                                    true);

    param_play_immediate_ = param::ParameterFactory::declareBool("play/immediate",
                                         param::ParameterDescription("Immediatly send new entry"),
                                         false);
    param_play_rate_ = param::ParameterFactory::declareRange("play/rate",
                                          param::ParameterDescription("Playing framerate"),
                                          1, 256, 30, 1);
    param_play_frame_ = param::ParameterFactory::declareValue("play/frame",
                                          param::ParameterDescription("TF Frame name"),
                                          std::string("base_link"));

    progress_ = param::ParameterFactory::declareOutputProgress("progress",
                                                   param::ParameterDescription("Playing progess"));

    auto cond_playing = [this]() { return playing_; };
    auto cond_not_playing = [this]() { return !playing_; };
    auto cond_not_immediate = [this]() { return getTickFrequency() >= 0; };
    auto cond_negative = [this]() { return !playing_ && param_sample_negative_count_->as<int>() > 0; };
    auto cond_negative_opts = [this]() { return !playing_ && param_sample_negative_count_->as<int>() > 0 && param_sample_negative_rois_->as<bool>() == false; };


    addConditionalParameter(action_import_, cond_not_playing, std::bind(&PeopleDatasetImporter::import, this));
    addConditionalParameter(action_play_start_, cond_not_playing, std::bind(&PeopleDatasetImporter::start_play, this));
    addConditionalParameter(action_play_stop_, cond_playing, std::bind(&PeopleDatasetImporter::stop_play, this));
    addConditionalParameter(param_data_positive_, cond_not_playing);
    addConditionalParameter(param_data_negative_, cond_not_playing);
    addConditionalParameter(param_sample_positive_count_, cond_not_playing);
    addConditionalParameter(param_sample_positive_range_, cond_not_playing);
    addConditionalParameter(param_sample_negative_count_, cond_not_playing);
    addConditionalParameter(param_sample_negative_range_, cond_negative);
    addConditionalParameter(param_sample_negative_rois_, cond_negative);
    addConditionalParameter(param_sample_negative_size_min_, cond_negative_opts);
    addConditionalParameter(param_sample_negative_size_max_, cond_negative_opts);
    addConditionalParameter(param_sample_negative_size_ratio_, cond_negative_opts);
    addConditionalParameter(param_sample_exclusive_, cond_negative_opts);
    addConditionalParameter(param_sample_sequential_, cond_not_playing);
    addConditionalParameter(param_sample_random_seed_, cond_not_playing);
    addParameter(param_play_immediate_, std::bind(&PeopleDatasetImporter::update_frequency, this));
    addConditionalParameter(param_play_rate_, cond_not_immediate, std::bind(&PeopleDatasetImporter::update_frequency, this));
    addParameter(param_play_frame_);
    addParameter(progress_);
}

void PeopleDatasetImporter::process()
{

}

void PeopleDatasetImporter::tick()
{
    // do nothing if not playing
    if (!playing_)
        return;

    if (play_index_ < play_entries_.size())
    {
        const std::string& frame = param_play_frame_->as<std::string>();
        const MetaEntry& entry = play_entries_.at(play_index_);

        {
            CvMatMessage::Ptr depth_msg = std::make_shared<CvMatMessage>(enc::depth, entry.timestamp);
            CvMatMessage::Ptr visual_msg = std::make_shared<CvMatMessage>(enc::bgr, entry.timestamp);
            PointCloudMessage::Ptr pcl_msg = std::make_shared<PointCloudMessage>(frame, entry.timestamp);
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
            std::shared_ptr<std::vector<RoiMessage>> roi_msg = std::make_shared<std::vector<RoiMessage>>();

            depth_msg->frame_id = frame;
            visual_msg->frame_id = frame;
            {
                INTERLUDE("load");
                read_to(entry, visual_msg->value, depth_msg->value, *cloud);
                pcl_msg->value = cloud;
                roi_msg->assign(entry.rois.begin(), entry.rois.end());

                if (visual_msg->value.channels() == 1)
                    visual_msg->setEncoding(enc::mono);

                std::for_each(roi_msg->begin(), roi_msg->end(),
                              [=](RoiMessage& msg) { msg.frame_id = frame; });
            }

            {
                INTERLUDE("publish");
                msg::publish(out_depth_image_, depth_msg);
                msg::publish(out_bgr_image_, visual_msg);
                msg::publish(out_pointcloud_, pcl_msg);
                msg::publish<GenericVectorMessage, RoiMessage>(out_rois_, roi_msg);
            }
        }

        ++play_index_;
        auto progress = std::dynamic_pointer_cast<param::OutputProgressParameter>(progress_);
        progress->setProgress(play_index_, play_entries_.size());
    }
    else
    {
        auto end = connection_types::makeEmpty<EndOfSequenceMessage>();

        msg::publish(out_depth_image_, end);
        msg::publish(out_bgr_image_, end);
        msg::publish(out_pointcloud_, end);
        msg::publish(out_rois_, end);

        stop_play();
    }
}

void PeopleDatasetImporter::import()
{
    action_import_->setEnabled(false);

    ainfo << "Imporing datasets..." << std::endl;
    {
        playing_ = false;
        play_entries_.clear();
        play_index_ = 0;
    }
    {
        auto load = [](const std::string& dir,
                       std::vector<MetaEntry>& entries,
                       const param::ParameterPtr& interval)
        {
            entries.clear();
            load_dataset(dir, entries);

            auto range = std::dynamic_pointer_cast<param::IntervalParameter>(interval);
            range->setMin<int>(0);
            range->setMax<int>(entries.size() - 1);
            if (range->upper<int>() > range->max<int>())
                range->setUpper<int>(range->max<int>());

            std::sort(entries.begin(), entries.end());
        };

        const std::string pos_dir = param_data_positive_->as<std::string>();
        const std::string neg_dir = param_data_negative_->as<std::string>();

        ainfo << "\tPositive: " << pos_dir << std::endl;
        load(pos_dir, positive_entries_, param_sample_positive_range_);
        if (!neg_dir.empty())
        {
            ainfo << "\tNegative: " << neg_dir << std::endl;
            load(neg_dir, negative_entries_, param_sample_negative_range_);
        }
        else
            ainfo << "\tNegative: skipped" << std::endl;

        const std::size_t pos_label_count = std::accumulate(positive_entries_.begin(), positive_entries_.end(), 0,
                                                            [](std::size_t init, const MetaEntry& entry) { return init + entry.rois.size(); });
        std::dynamic_pointer_cast<param::RangeParameter>(param_sample_positive_count_)->setMax<int>(pos_label_count);
        if (param_sample_negative_rois_->as<bool>())
        {
            const std::size_t neg_label_count = std::accumulate(negative_entries_.begin(), negative_entries_.end(), 0,
                                                                [](std::size_t init, const MetaEntry& entry) { return init + entry.rois.size(); });
            std::dynamic_pointer_cast<param::RangeParameter>(param_sample_negative_count_)->setMax<int>(neg_label_count);
        }
        else
            std::dynamic_pointer_cast<param::RangeParameter>(param_sample_negative_count_)->setMax<int>(100000);
    }
    {
        random_ = std::minstd_rand(param_sample_random_seed_->as<int>());
    }

    const bool sequential = param_sample_sequential_->as<bool>();
    const bool sample_negative = param_sample_negative_count_->as<int>() > 0;
    const bool sample_exclusive = sample_negative && param_sample_exclusive_->as<bool>();

    typedef std::vector<MetaEntry>::iterator Itr;
    auto select_sample = [&](const Itr& begin, const Itr& end, Itr& cur) -> Itr&
    {
        if (sequential)
        {
            ++cur;
        }
        else
        {
            auto dist = std::distance(begin, end);
            cur = std::next(begin, random_() % dist);
        }
        return cur;
    };

    {
        const std::size_t count = param_sample_positive_count_->as<int>();
        auto interval = std::dynamic_pointer_cast<param::IntervalParameter>(param_sample_positive_range_);
        const std::size_t lower = interval->lower<int>();
        const std::size_t upper = interval->upper<int>();

        std::size_t cnt = 0;
        while (cnt < count)
        {
            auto begin = std::next(positive_entries_.begin(), lower);
            auto end = positive_entries_.begin();

            if (sample_exclusive)
            {
                end = std::next(positive_entries_.begin(), upper);

                std::size_t added = 0;
                for (Itr itr = begin; select_sample(begin, end, itr) != end && cnt + added < count;)
                    if (!itr->rois.empty())
                    {
                        play_entries_.push_back(*itr);
                        added += itr->rois.size();
                    }
                cnt += added;
            }
            else
            {
                end = std::next(positive_entries_.begin(), std::min(lower + count, upper));

                std::size_t added = 0;
                for (Itr itr = begin; select_sample(begin, end, itr) != end && cnt + added < count;)
                {
                    play_entries_.push_back(*itr);
                    added += sample_negative ? itr->rois.size() : 1;
                }

                cnt += added;
            }
        }
    }

    if (sample_negative)
    {
        const bool use_rois = param_sample_negative_rois_->as<bool>();
        if (!use_rois)
        {
            const int min_size = param_sample_negative_size_min_->as<int>();
            const int max_size = param_sample_negative_size_max_->as<int>();
            const int ratio = param_sample_negative_size_ratio_->as<int>();
            const std::size_t count = param_sample_negative_count_->as<int>();

            auto sample_negative = [&](MetaEntry& entry) -> boost::optional<ROIType>
            {
                const cv::Rect crop_rect(0, 0, entry.size.width, entry.size.height);

                cv::Rect roi;
                roi.width = min_size + (random_() % (max_size - min_size));
                switch (ratio)
                {
                case RATIO_1_TO_2:
                    roi.height = roi.width * 2;
                    break;
                case RATIO_1_TO_1:
                    roi.height = roi.width;
                    break;
                case RATIO_FREE:
                    roi.height = min_size + (random_() % (max_size - min_size));
                    break;
                }

                roi.x = random_() % (entry.size.width - roi.width);
                roi.y = random_() % (entry.size.height - roi.height);

                roi = roi & crop_rect;

                bool invalid = false;
                for (const RoiMessage& ref: entry.rois)
                    invalid = invalid || ((ref.value.rect() & roi).area() > 0);

                if (invalid)
                    return {};

                ROIType msg;
                msg.value.setRect(roi);
                msg.value.setClassification(BACKGROUND);
                msg.value.setColor(ClassificationColors.at(BACKGROUND));

                return msg;
            };

            if (!sample_exclusive)
            {
                auto begin = play_entries_.begin();
                auto end = play_entries_.end();

                std::size_t cnt = 0;
                while (cnt < count)
                {
                    std::size_t added = 0;
                    for (Itr itr = begin; select_sample(begin, end, itr) != end && cnt + added < count;)
                    {
                        MetaEntry& entry = *itr;

                        boost::optional<ROIType> roi = sample_negative(entry);
                        if (roi)
                        {
                            entry.rois.push_back(roi.get());
                            ++added;
                        }
                    }
                    cnt += added;
                }
            }
            else if (!negative_entries_.empty())
            {
                auto interval = std::dynamic_pointer_cast<param::IntervalParameter>(param_sample_negative_range_);
                const std::size_t lower = interval->lower<int>();
                const std::size_t upper = interval->upper<int>();

                auto begin = std::next(negative_entries_.begin(), lower);
                auto end = std::next(negative_entries_.begin(), upper);

                std::size_t cnt = 0;
                while (cnt < count)
                {
                    std::size_t added = 0;
                    for (Itr itr = begin; select_sample(begin, end, itr) != end && cnt + added < count;)
                    {
                        MetaEntry entry = *itr;
                        if (!entry.rois.empty())
                            continue;

                        boost::optional<ROIType> roi = sample_negative(entry);
                        if (roi)
                        {
                            entry.rois.push_back(roi.get());
                            play_entries_.push_back(entry);
                            ++added;
                        }
                    }
                    cnt += added;
                }
            }
            else
            {
                throw std::runtime_error("Negative sample set must be avaibable");
            }
        }
        else
        {
            const std::size_t count = param_sample_negative_count_->as<int>();
            auto interval = std::dynamic_pointer_cast<param::IntervalParameter>(param_sample_negative_range_);
            const std::size_t lower = interval->lower<int>();
            const std::size_t upper = interval->upper<int>();

            std::size_t cnt = 0;
            while (cnt < count)
            {
                auto begin = std::next(negative_entries_.begin(), lower);
                auto end = negative_entries_.begin();

                if (sample_exclusive)
                {
                    end = std::next(negative_entries_.begin(), upper);

                    std::size_t added = 0;
                    for (Itr itr = begin; select_sample(begin, end, itr) != end && cnt + added < count;)
                        if (!itr->rois.empty())
                        {
                            MetaEntry entry = *itr;
                            std::for_each(entry.rois.begin(), entry.rois.end(),
                                          [](ROIType& roi)
                                          {
                                              roi.value.setClassification(BACKGROUND);
                                              roi.value.setColor(ClassificationColors.at(BACKGROUND));
                                          });
                            play_entries_.push_back(entry);
                            added += itr->rois.size();
                        }
                    cnt += added;
                }
                else
                {
                    end = std::next(negative_entries_.begin(), std::min(lower + count, upper));

                    std::size_t added = 0;
                    for (Itr itr = begin; select_sample(begin, end, itr) != end && cnt + added < count;)
                    {
                        MetaEntry entry = *itr;
                        std::for_each(entry.rois.begin(), entry.rois.end(),
                                      [](ROIType& roi)
                                      {
                                          roi.value.setClassification(BACKGROUND);
                                          roi.value.setColor(ClassificationColors.at(BACKGROUND));
                                      });
                        play_entries_.push_back(entry);
                        added += sample_negative ? itr->rois.size() : 1;
                    }

                    cnt += added;
                }
            }
        }
    }
    std::sort(play_entries_.begin(), play_entries_.end());
    ainfo << "Import done" << std::endl;

    action_import_->setEnabled(true);
}

void PeopleDatasetImporter::start_play()
{
    play_index_ = 0;
    playing_ = true;
}

void PeopleDatasetImporter::stop_play()
{
    playing_ = false;
    tri_finished_->trigger();
}

void PeopleDatasetImporter::update_frequency()
{
    if (param_play_immediate_->as<bool>())
        setTickFrequency(-1.0);
    else
        setTickFrequency(param_play_rate_->as<int>());
}
