#include "import_INRIA_data.hpp"

#include <functional>

#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/connector_type.h>
#include <csapex/model/node_modifier.h>
#include <csapex/signal/event.h>
#include <csapex/msg/io.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex_opencv/roi_message.h>
#include <csapex/msg/generic_vector_message.hpp>


using namespace csapex;
using namespace dataset;
using namespace people;
using namespace connection_types;

CSAPEX_REGISTER_CLASS(csapex::dataset::people::ImportINRIAData, csapex::Node)

void ImportINRIAData::setup(NodeModifier &node_modifier)
{
    out_image_ = node_modifier.addOutput<CvMatMessage>("Image");
    out_rois_ = node_modifier.addOutput<GenericVectorMessage, RoiMessage>("ROIs");

    play_started_ = node_modifier.addEvent("Started");
    play_finished_ = node_modifier.addEvent("Finished");
    play_stopped_ = node_modifier.addEvent("Stopped");

    play_start_ = node_modifier.addSlot("Start",
                                        std::bind(&ImportINRIAData::slotPlay, this));
    play_stop_ = node_modifier.addSlot("Stop",
                                        std::bind(&ImportINRIAData::slotStop, this));
    play_reset_ = node_modifier.addSlot("Reset",
                                        std::bind(&ImportINRIAData::slotReset, this));


}

void ImportINRIAData::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(param::ParameterFactory::declareDirectoryInputPath("/path", ""),
                            path_);
    parameters.addParameter(param::ParameterFactory::declareRange("/negative/window/width", 10, 640, 128, 1),
                            neg_window_size_.width);
    parameters.addParameter(param::ParameterFactory::declareRange("/negative/window/height", 10, 480, 64, 1),
                            neg_window_size_.height);
    parameters.addParameter(param::ParameterFactory::declareValue("/negative/rng/seed", 0),
                            neg_rng_seed_);
    parameters.addParameter(param::ParameterFactory::declareBool("/negative/do_sample", true),
                            neg_do_sample_);

    param::Parameter::Ptr param_play = param::ParameterFactory::declareBool("/play", false);
    param_play_ = std::dynamic_pointer_cast<param::ValueParameter>(param_play);
    parameters.addParameter(param_play_, std::bind(&ImportINRIAData::play, this));

    param::Parameter::Ptr param_play_index = param::ParameterFactory::declareRange("/play/index", 0, 1, 1, 1);
    param_play_index_ = std::dynamic_pointer_cast<param::RangeParameter>(param_play_index);
    std::function<bool()> show_index =
            [this]() {return samples_.size() > 0;};
    parameters.addConditionalParameter(param_play_index_, show_index);

}

bool ImportINRIAData::canProcess() const
{
    return play_;
}

void ImportINRIAData::process()
{
    if(samples_.empty() && path_ != "")
        import();

    play_index_ = param_play_index_->as<int>();

    if(play_index_ < samples_.size()) {
        Instance::Set::iterator it = samples_.begin();
        std::advance(it, play_index_);

        CvMatMessage::Ptr                        img_msg(new CvMatMessage(enc::bgr, 0));
        std::shared_ptr<std::vector<RoiMessage>> rois_msg(new std::vector<RoiMessage>());
        img_msg->value = cv::imread(it->second.path);

        std::vector<cv::Rect> &rois = it->second.rois;
        Instance::Label label = it->second.label;
        for(cv::Rect &roi : rois) {
            RoiMessage roi_msg;
            roi_msg.value.setRect(roi);
            roi_msg.value.setClassification(label);
            if(label == Instance::NEGATIVE) {
                roi_msg.value.setColor(cv::Scalar(0,0,255));
            } else {
                roi_msg.value.setColor(cv::Scalar(0,255,0));
            }
            rois_msg->emplace_back(roi_msg);
        }

        msg::publish(out_image_, img_msg);
        msg::publish<GenericVectorMessage, RoiMessage>(out_rois_, rois_msg);

        ++it;
        ++play_index_;
        param_play_index_->set((int) play_index_);
    } else {
        play_ = false;
        param_play_->set(play_);
        play_finished_->trigger();
    }
}

void ImportINRIAData::import()
{
    std::srand(neg_rng_seed_);
    samples_.clear();
    Instance::Set neg_samples;
    Instance::Set pos_samples;
    std::size_t pos_sample_count;
    std::size_t neg_sample_count;
    readFolder(path_, pos_samples, neg_samples, pos_sample_count, neg_sample_count);

    if(neg_sample_count > pos_sample_count && neg_do_sample_) {
        std::vector<std::size_t> random_vector = randomVector(neg_sample_count);
        std::sort(random_vector.begin(), random_vector.begin() + pos_sample_count + 1);

        Instance::Set buffer_neg_samples;
        std::size_t pos_random = 0;
        std::size_t pos_sample = 0;
        for(auto &i : neg_samples) {
            Instance &instance = i.second;
            Instance buffer_instance;
            for(std::size_t i = 0 ; i < instance.rois.size() ; ++i) {
                if(pos_sample == random_vector[pos_random]) {
                    buffer_instance.path = instance.path;
                    buffer_instance.label = instance.label;
                    buffer_instance.rois.push_back(instance.rois.at(i));
                    ++pos_random;
                }
                ++pos_sample;
            }
            if(buffer_instance.rois.size() > 0) {
                buffer_neg_samples.insert(std::make_pair(i.first, buffer_instance));
            }
        }
        std::swap(neg_samples, buffer_neg_samples);
    }

    samples_.insert(pos_samples.begin(), pos_samples.end());
    samples_.insert(neg_samples.begin(), neg_samples.end());

    param_play_index_->setMax((int) samples_.size() - 1);
    param_play_index_->set((int) 0);
}

void ImportINRIAData::play()
{
    play_ = param_play_->as<bool>();
    if(play_) {
        play_started_->trigger();
        yield();
    } else {
        play_stopped_->trigger();
    }
}

void ImportINRIAData::slotPlay()
{
    if(!play_) {
        play_ = true;
        param_play_->set(true);
        play_started_->trigger();
        yield();
    }
}

void ImportINRIAData::slotStop()
{
    if(play_) {
        play_ = false;
        param_play_->set(false);
        play_stopped_->trigger();
    }
}

void ImportINRIAData::slotReset()
{
    if(samples_.size() > 0) {
        param_play_index_->set(0);
        play_index_ = 0;
    }
}

