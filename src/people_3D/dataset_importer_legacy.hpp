#pragma once

/// PROJECT
#include <csapex/model/tickable_node.h>
#include <csapex/model/connector_type.h>
#include <csapex_opencv/roi_message.h>

/// SYSTEM
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <random>
#include <atomic>

namespace bfs = boost::filesystem;

namespace csapex
{
namespace dataset
{

class PeopleDatasetImporterLegacy : public csapex::TickableNode
{
    struct DataSetEntry
    {
        std::string                                            id;
        bfs::path                                              path_rgb;
        bfs::path                                              path_pcl;
        bfs::path                                              path_depth;
        cv::Size                                               size_depth;
        std::vector<csapex::connection_types::RoiMessage>      rois;
        ulong                                                  ts;

        bool static compare(const DataSetEntry &a,
                            const DataSetEntry &b)
        {
            return a.id < b.id;
        }
    };

    typedef std::vector<DataSetEntry> DataSet;

public:
    PeopleDatasetImporterLegacy();

    virtual void setup(csapex::NodeModifier &node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;
    virtual void tick() override;

private:
    /// I/O
    csapex::Output  *out_pointcloud_;
    csapex::Output  *out_depth_image_;
    csapex::Output  *out_bgr_image_;
    csapex::Output  *out_rois_;
    csapex::Event   *finished_;
    /// PAREMTERS
    enum Ratio {Ratio_1_to_2, Ratio_1_to_1, Ratio_free};
    csapex::param::Parameter::Ptr import_path_;
    csapex::param::Parameter::Ptr import_;
    csapex::param::Parameter::Ptr use_random_;
    csapex::param::Parameter::Ptr random_seed_;
    csapex::param::Parameter::Ptr interval_;
    csapex::param::Parameter::Ptr play_;
    csapex::param::Parameter::Ptr play_btn_;
    csapex::param::Parameter::Ptr play_progress_;
    csapex::param::Parameter::Ptr prep_progress_;
    csapex::param::Parameter::Ptr non_human_rois_;
    csapex::param::Parameter::Ptr ratio_;
    csapex::param::Parameter::Ptr kls;

    bfs::path root_path_;
    enum ContentType {DEPTH, PCL, VISUAL, ROI};
    enum ClassificationType {BACKGROUND = 0, HUMAN = 1, HUMAN_PART = 2};
    cv::Scalar colors_[3] = {cv::Scalar(0,0,255), cv::Scalar(255), cv::Scalar(0, 255, 255)};
    const std::map<ContentType, bfs::path> dir_structure_;
    const std::map<ContentType, bfs::path> file_types_;

    DataSet          dataset_;
    DataSet          play_set_;
    std::size_t      play_pos_;
    bool             lets_play_;
    bool             stride_;
    bool             hold_;


    std::minstd_rand random_;


    void checkDirectoryStructure() const;
    void import();
    void resetPlaySet();
    void updateHz();
    void createEntry(const std::string &id,
                     const boost::filesystem::path &rois_path,
                     DataSetEntry &entry);

    void play();
    void prepareThePlaySet();
};
}
}
