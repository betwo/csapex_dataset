#pragma once

#include "annotation.hpp"
#include <boost/optional.hpp>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>

namespace csapex { namespace dataset { namespace sand {

class Entry
{
    friend class YAML::convert<Entry>;

public:
    Entry() = default;
    Entry(uint64_t id, const boost::filesystem::path& pointcloud_path, const boost::filesystem::path& annotation_path);

    uint64_t getId() const { return id_; }

    const Annotation& getAnnotation() const;

    template<typename PointT>
    typename pcl::PointCloud<PointT>::ConstPtr getPointcloud() const;
    void getPointcloud(pcl::PCLPointCloud2& pointcloud) const;
    const boost::filesystem::path& getPointCloudPath() const { return pointcloud_path_; }

private:
    uint64_t id_;
    boost::filesystem::path pointcloud_path_;
    boost::filesystem::path annotation_path_;

    mutable boost::optional<Annotation> annotation_;
};

}}}

namespace YAML {
template<>
struct convert<csapex::dataset::sand::Entry>
{
    static Node encode(const csapex::dataset::sand::Entry& entry);
    static bool decode(const Node& node, csapex::dataset::sand::Entry& entry);
};
}

