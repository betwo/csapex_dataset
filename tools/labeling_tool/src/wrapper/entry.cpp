#include "entry.hpp"

#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>
#include <pcl/io/pcd_io.h>
#include "src/converter/converter.hpp"

namespace fs = boost::filesystem;

namespace wrapper
{

DatasetEntry::DatasetEntry() :
    Entry()
{

}

std::shared_ptr<DatasetEntry> DatasetEntry::create(const std::string &directory, const std::string &id)
{
    std::shared_ptr<DatasetEntry> entry = std::make_shared<DatasetEntry>();
    entry->changed_ = false;
    entry->id_ = id;
    entry->directory_ = directory;

    const fs::path RGB_FILE   = fs::path(directory) / "visual" / (id + ".ppm");
    const fs::path DEPTH_FILE = fs::path(directory) / "depth" / (id + ".yaml.tar.gz");
//    const fs::path PCL_FILE   = fs::path(directory) / "pointcloud" / (id + ".pcd");
    const fs::path ROI_FILE   = fs::path(directory) / "roi" / (id + ".yaml");

    /*
    {
        cv::FileStorage fs(DEPTH_FILE.string(), cv::FileStorage::READ);
        fs["data"] >> entry->depth_image_;
        fs.release();
    }
    {
        entry->intensity_image_ = cv::imread(RGB_FILE.string(), -1);
    }
    */
    {
        YAML::Node file = YAML::LoadFile(ROI_FILE.string());
        YAML::Node rois = file["rois"];
        for (YAML::Node node : rois)
        {
            ROI roi;
            roi.timestamp = node["ts"].as<double>();
            roi.label = node["vis"].as<int>();
            roi.depth = {node["x_d"].as<int>(), node["y_d"].as<int>(), node["w_d"].as<int>(), node["h_d"].as<int>()};
            roi.rgb = {node["x_rgb"].as<int>(), node["y_rgb"].as<int>(), node["w_rgb"].as<int>(), node["h_rgb"].as<int>()};
            entry->rois_.push_back(roi);
            entry->original_rois_.push_back(roi);
        }
    }

    return entry;
}

void DatasetEntry::save(const std::string &directory) const
{
    if (!changed_)
        return;

    const fs::path ROI_FILE   = fs::path(directory) / (id_ + ".yaml");

    std::cout << "saving " << ROI_FILE << std::endl;

    const cv::Mat depth = depth_image();

    YAML::Emitter yaml;
    yaml << YAML::BeginMap;
    yaml << "img_d_h" << depth.rows;
    yaml << "img_d_w" << depth.cols;
    yaml << "rois" << YAML::BeginSeq;
    for (const ROI& roi : rois_)
    {
        yaml << YAML::BeginMap;
        yaml << "ts" << roi.timestamp;
        yaml << "img_id" << id_;
        yaml << "x_d" << roi.depth.x;
        yaml << "y_d" << roi.depth.y;
        yaml << "h_d" << roi.depth.height;
        yaml << "w_d" << roi.depth.width;
        yaml << "x_rgb" << roi.rgb.x;
        yaml << "y_rgb" << roi.rgb.y;
        yaml << "h_rgb" << roi.rgb.height;
        yaml << "w_rgb" << roi.rgb.width;
        yaml << "vis" << roi.label;
        yaml << YAML::EndMap;
    }
    yaml << YAML::EndSeq;
    if (rois_.empty())
        yaml << "ts" << (ulong) 0;
    else
        yaml << "ts" << (ulong) (rois_.front().timestamp * 1e6);
    yaml << YAML::EndMap;

    std::ofstream out(ROI_FILE.string());
    out << yaml.c_str();
    out.close();
}

const cv::Mat DatasetEntry::depth_image() const
{
    const fs::path DEPTH_FILE = fs::path(directory_) / "depth" / (id_ + ".yaml.tar.gz");

    cv::Mat ret;

    cv::FileStorage fs(DEPTH_FILE.string(), cv::FileStorage::READ);
    fs["data"] >> ret;
    fs.release();

    return ret;
}

const cv::Mat DatasetEntry::intensity_image() const
{
    const fs::path RGB_FILE   = fs::path(directory_) / "visual" / (id_ + ".ppm");
    return cv::imread(RGB_FILE.string(), -1);
}

const pcl::PointCloud<pcl::PointXYZ> DatasetEntry::point_cloud() const
{
    const fs::path PCL_FILE   = fs::path(directory_) / "pointcloud" / (id_ + ".pcd");
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::io::loadPCDFile(PCL_FILE.string(), cloud);
    return cloud;
}

bool operator==(const ROI& left, const ROI& right)
{
    return std::tie(left.timestamp, left.label, left.depth, left.rgb) == std::tie(right.timestamp, right.label, right.depth, right.rgb);
}

void DatasetEntry::set_rois(const std::vector<ROI> &rois)
{
    if (!(changed_ = !(original_rois_ == rois)))
        return;
    Entry::set_rois(rois);
}

}
