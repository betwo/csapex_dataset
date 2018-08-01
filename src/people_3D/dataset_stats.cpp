#include "dataset_stats.hpp"
#include "dataset_common.hpp"

#include <csapex/msg/io.h>
#include <csapex/msg/output.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/model/node_modifier.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/interval_parameter.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_opencv/roi_message.h>


CSAPEX_REGISTER_CLASS(csapex::dataset::PeopleDatasetStats, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex::dataset;
using namespace csapex::dataset::people;
namespace fs = boost::filesystem;

PeopleDatasetStats::PeopleDatasetStats() :
    stats_("not loaded")
{
}

void PeopleDatasetStats::setupParameters(Parameterizable& parameter)
{
    addParameter(param::factory::declareTrigger("load",
                                                         param::ParameterDescription("Load dataset and generate stats")),
                 std::bind(&PeopleDatasetStats::load, this));
    addParameter(param::factory::declareDirectoryInputPath("path",
                                                                    param::ParameterDescription("Path to dataset"),
                                                                    ""),
                 [this](param::Parameter* param) { path_ = param->as<std::string>(); load();});

    range_ = param::factory::declareInterval("range",
                                                      param::ParameterDescription("Range for samples"),
                                                      0, 100000, 0, 100000, 1);
    addParameter(range_);
}

void PeopleDatasetStats::setup(NodeModifier& node_modifier)
{
    out_stats_ = node_modifier.addOutput<std::string>("stats");
}

void PeopleDatasetStats::process()
{
    msg::publish(out_stats_, stats_);
}

void PeopleDatasetStats::load()
{
    if (!fs::is_directory(path_))
    {
        stats_ = "not loaded";
        return;
    }
    std::vector<MetaEntry> entries;
    load_dataset(path_, entries);
    auto begin = entries.begin();
    auto end = entries.begin();

    auto range = std::dynamic_pointer_cast<param::IntervalParameter>(range_);

    std::advance(begin, range->lower<int>());
    std::advance(end, std::min<std::size_t>(entries.size(), range->upper<int>()));

    const std::size_t frames = std::distance(begin, end);
    std::map<int, std::size_t> labels;
    for (auto itr = begin; itr != end; ++itr)
    {
        const MetaEntry& entry = *itr;
        for (const ROIType& roi : entry.rois)
            labels[roi.value.classification()] += 1;
    }
    const std::size_t total_labels = std::accumulate(labels.begin(), labels.end(), 0,
                                               [](std::size_t prev, const std::pair<int, std::size_t>& entry) { return prev + entry.second; });
    const double labels_per_frame = static_cast<double>(total_labels) / static_cast<double>(frames);
//    const auto duration_minmax = std::minmax_element(begin, end, [](const MetaEntry& a, const MetaEntry& b) { return a.timestamp < b.timestamp; });
//    const ulong duration_s = (duration_minmax.second->timestamp - duration_minmax.first->timestamp) % 60;
//    const ulong duration_min = (duration_minmax.second->timestamp - duration_minmax.first->timestamp) / 60;

    std::ostringstream os;
    os << "Frames:       " << frames << std::endl;
    os << "Labels:       " << total_labels << std::endl;
    os << "Labels/Frame: " << labels_per_frame << std::endl;
    for (const auto& entry : labels)
        os << "Labels [" << entry.first << "]: " << entry.second << std::endl;
//    os << "Duration:     " << duration_min << "min" << duration_s << "s" << std::endl;
    stats_ = os.str();
}


