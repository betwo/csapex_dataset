#include "entry.hpp"
#include <yaml-cpp/yaml.h>
#include <pcl/io/pcd_io.h>
#include <boost/make_shared.hpp>

using namespace csapex::dataset::sand;
namespace bfs = boost::filesystem;

Annotation::Annotation(boost::filesystem::path path) :
        path_(std::move(path))
{
    YAML::Node root = YAML::LoadFile(path_.string());
    if (!root.IsMap())
        throw std::runtime_error("Invalid annotation file (missing root): " + path_.string());

    timestamp_ = root["timestamp"].as<uint64_t>();
    frame_ = root["frame"].as<std::string>();
    width_ = root["width"].as<int>();
    height_ = root["height"].as<int>();

    YAML::Node regions = root["regions"];
    for (auto node : regions)
    {
        Region region;
        region.x = node["x"].as<int>();
        region.y = node["y"].as<int>();
        region.width = node["width"].as<int>();
        region.height = node["height"].as<int>();
        region.clazz = node["class"].as<int>();
        regions_.push_back(region);
    }
}

Entry::Entry(uint64_t id, boost::filesystem::path pointcloud_path, boost::filesystem::path annotation_path) :
        id_(id),
        pointcloud_path_(std::move(pointcloud_path)),
        annotation_path_(std::move(annotation_path))
{
    if (!bfs::exists(pointcloud_path_))
        throw std::runtime_error("Pointcloud file does not exist: " + pointcloud_path_.string());

    if (!bfs::exists(annotation_path_))
        throw std::runtime_error("Annotation file does not exist: " + annotation_path_.string());
}

const Annotation& Entry::getAnnotation() const
{
    if (!annotation_)
        annotation_.reset(new Annotation(annotation_path_));

    return *annotation_;
}

const std::string& Entry::getFrame() const
{
    return getAnnotation().getFrame();
}

uint64_t Entry::getTimestamp() const
{
    return getAnnotation().getTimestamp();
}

template<typename PointT>
typename pcl::PointCloud<PointT>::ConstPtr Entry::getPointcloud() const
{
    auto pointcloud = boost::make_shared<pcl::PointCloud<PointT>>();
    pcl::io::loadPCDFile(pointcloud_path_.string(), *pointcloud);
    return pointcloud;
}

void Entry::getPointcloud(pcl::PCLPointCloud2& pointcloud) const
{
    pcl::io::loadPCDFile(pointcloud_path_.string(), pointcloud);
}

