#pragma once

#include <csapex_opencv/roi_message.h>

#include <unordered_map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/filesystem.hpp>
#include <boost/variant.hpp>

namespace csapex
{
namespace dataset
{
namespace people
{

enum ClassificationType
{
    BACKGROUND = 0,
    HUMAN = 1,
    HUMAN_PART = 2,
    UNKNOWN = 3,
};
const std::unordered_map<int, cv::Scalar> ClassificationColors{
    { BACKGROUND, cv::Scalar(0, 0, 255) },
    { HUMAN, cv::Scalar(0, 255, 0) },
    { HUMAN_PART, cv::Scalar(255, 0, 0) },
    { UNKNOWN, cv::Scalar(0, 128, 128) },
};

/*
id: this_is_the_id
heigth: 480
width: 640
rois:
  - img_id: this_is_the_id
    ts: 1339072457
    x_d: 244
    y_d: 115
    w_d: 141
    h_d: 297
    x_rgb: 244
    y_rgb: 115
    w_rgb: 141
    h_rgb: 297
    class: 1
ts: 1339072457
*/
typedef csapex::connection_types::RoiMessage ROIType;

struct MetaEntry
{

    std::string                 id;
    boost::filesystem::path     path_visual;
    boost::filesystem::path     path_pcl;
    boost::filesystem::path     path_depth;
    boost::filesystem::path     path_roi;
    cv::Size                    size;
    std::vector<ROIType>        rois;
    ulong                       timestamp;
};

inline bool operator<(const MetaEntry &a,
               const MetaEntry &b)
{
    return a.id < b.id;
}

struct Entry
{
    using PCLType = boost::variant<pcl::PointCloud<pcl::PointXYZI>, pcl::PointCloud<pcl::PointXYZRGB>>;
    MetaEntry meta;
    cv::Mat   visual;
    cv::Mat   depth;
    PCLType   pointcloud;
};

void save_fill_meta(Entry& entry, const std::string& directory);
void save(const Entry& entry);
void read_to(const MetaEntry& entry, cv::Mat& visual, cv::Mat& depth, pcl::PointCloud<pcl::PointXYZI>& cloud);
void load_dataset(const std::string& directory, std::vector<MetaEntry>& entries);
MetaEntry load_entry(const std::string& directory, const std::string& id);

}
}
}
