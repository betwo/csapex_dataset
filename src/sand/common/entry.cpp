#include "entry.hpp"
#include <pcl/io/pcd_io.h>
#include <boost/make_shared.hpp>

using namespace csapex::dataset::sand;
namespace bfs = boost::filesystem;

Entry::Entry(uint64_t id, const boost::filesystem::path& pointcloud_path, const boost::filesystem::path& annotation_path) :
        id_(id),
        pointcloud_path_(pointcloud_path),
        annotation_path_(annotation_path)
{}

const Annotation& Entry::getAnnotation() const
{
    if (!annotation_)
        annotation_ = Annotation::load(annotation_path_);

    return annotation_.get();
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

namespace YAML
{
Node convert<Entry>::encode(const Entry& entry)
{
    Node node;
    node["id"] = entry.id_;
    node["pointcloud"] = entry.pointcloud_path_.string();
    node["annotation"] = entry.annotation_path_.string();
    return node;
}

bool convert<Entry>::decode(const Node& node, csapex::dataset::sand::Entry& entry)
{
    if (!node.IsMap())
        return false;

    entry.id_ = node["id"].as<uint64_t>();
    entry.pointcloud_path_ = node["pointcloud"].as<std::string>();
    entry.annotation_path_ = node["annotation"].as<std::string>();
    return true;
}
}
