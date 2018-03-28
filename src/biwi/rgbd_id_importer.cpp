#include "rgbd_id_importer.hpp"

#include <csapex/model/node_modifier.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/msg/end_of_sequence_message.h>
#include <csapex/msg/io.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/signal/event.h>

#include <csapex_opencv/cv_mat_message.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>

#include <map>

#include <boost/regex.hpp>

using namespace csapex;
using namespace dataset;
using namespace biwi;

using namespace csapex::connection_types;
namespace bfs = boost::filesystem;

CSAPEX_REGISTER_CLASS(csapex::dataset::biwi::RGBDIDImporter, csapex::Node)

RGBDIDImporter::RGBDIDImporter() :
    playing_(false)
{
}

void RGBDIDImporter::setup(NodeModifier &node_modifier)
{
    output_pointcloud_ = node_modifier.addOutput<PointCloudMessage>("Pointcloud");
    output_mask_       = node_modifier.addOutput<CvMatMessage>("Mask");
    output_rgb_        = node_modifier.addOutput<CvMatMessage>("RGB");
    event_finished_    = node_modifier.addEvent("finished");
}

void RGBDIDImporter::setupParameters(Parameterizable &parameters)
{
    auto path    = param::ParameterFactory::declareDirectoryInputPath("path", "").build();

    auto reload          = param::ParameterFactory::declareTrigger("reload");
    auto start_play      = param::ParameterFactory::declareTrigger("start play");
    auto stop_play       = param::ParameterFactory::declareTrigger("stop play");
    auto start_instantly = param::ParameterFactory::declareBool("start instantly", false);
    auto play_progress   = param::ParameterFactory::declareOutputProgress("played").build<param::OutputProgressParameter>();
    auto current_frame   = param::ParameterFactory::declareOutputText("current frame").build<param::OutputTextParameter>();

    auto play_only = [this]() { return playing_; };
    auto no_play_only = [this]() { return data_.size() > 0 && !playing_; };

    parameters.addParameter(path, [this](param::Parameter* param) { import(param->as<std::string>()); });
    parameters.addConditionalParameter(reload, no_play_only, [this, path](param::Parameter* param) { import(path->as<std::string>()); });
    parameters.addParameter(start_instantly,
                            [this](param::Parameter* param)
                            {
                                param_start_instantly_ = param->as<bool>();
                                if (param_start_instantly_ && data_.size() > 0)
                                {
                                    startPlay();
                                    triggerStartPlayEvent();
                                }
                            });

    parameters.addConditionalParameter(start_play, no_play_only, [this](param::Parameter*) { if ( data_.size() > 0) startPlay(); });
    parameters.addConditionalParameter(stop_play, play_only,
                                       [this](param::Parameter*)
                                       {
                                           playing_ = false;
                                           ainfo << "Stop playing" << std::endl;;
                                       });
    parameters.addConditionalParameter(play_progress, play_only);
    parameters.addConditionalParameter(current_frame, play_only);

    play_progress_ = play_progress;
    current_frame_ = current_frame;

}

bool RGBDIDImporter::canProcess() const
{
    return data_.size() > 0 && playing_;
}

void RGBDIDImporter::process()
{
    if (data_iterator_ != data_.end())
    {
        auto& entry = *data_iterator_;

     //   auto pointcloud_msg = std::make_shared<PointCloudMessage>(entry.getAnnotation().getFrame(), entry.getAnnotation().getTimestamp());
     //   auto rois_msg = std::make_shared<std::vector<RoiMessage>>();


     //   msg::publish(output_pointcloud_, pointcloud_msg);
     //   msg::publish(output_rgb_, rois_msg);

        play_progress_->advanceProgress();
//        current_frame_->set("Frame ID: " + std::to_string(entry.getId()));
        ++data_iterator_;
    }
    else
    {
        playing_ = false;
        auto end = makeEmpty<EndOfSequenceMessage>();
        msg::publish(output_pointcloud_, end);
        msg::publish(output_mask_,       end);
        msg::publish(output_rgb_,        end);
        msg::trigger(event_finished_);
    }
}

void RGBDIDImporter::import(const boost::filesystem::path &path)
{
    if(path.empty())
        return;

    if(!bfs::exists(path))
        throw std::runtime_error("Path '" + path.string() + "' does not exist!");

    playing_ = false;

    data_.clear();

    std::map<std::string, MetaEntry> content;

    const bfs::recursive_directory_iterator end;
    bfs::recursive_directory_iterator dir_iter( path );
    while(dir_iter != end) {
        const bfs::path current_path = dir_iter->path();
        if(bfs::is_regular_file(current_path)) {
             const std::string file = current_path.filename().string();
             const std::size_t delim_pos = file.find_first_of('-');
             const std::string id = file.substr(0,delim_pos);
             const char type = file[delim_pos+1];

             std::cout << "id: " << id << " type: " << type << std::endl;

        }
        ++dir_iter;
    }
}

void RGBDIDImporter::startPlay()
{
    data_iterator_ = data_.begin();
    playing_ = true;
    ainfo << "Start playing..." << std::endl;
}

void RGBDIDImporter::triggerStartPlayEvent()
{
    const auto& events = node_modifier_->getEvents();
    auto itr = std::find_if(events.begin(), events.end(),
                            [](const EventPtr& event) { return event->getLabel() == "start play"; });
    if (itr == events.end())
        throw std::runtime_error("Unable to find 'start play' event");

    auto& ptr = (*itr);
    ptr->trigger();
}
