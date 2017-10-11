#pragma once

#include <boost/filesystem/path.hpp>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>

namespace csapex { namespace dataset { namespace sand {

class Annotation
{
public:
    struct Region
    {
        int x;
        int y;
        int width;
        int height;
        int clazz;
    };
public:
    explicit Annotation(boost::filesystem::path path);

    uint64_t getTimestamp() const { return timestamp_; }
    const std::string& getFrame() const { return frame_; }
    const std::vector<Region>& getRegions() const { return regions_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }

private:
    const boost::filesystem::path path_;

    uint64_t timestamp_;
    std::string frame_;
    int width_;
    int height_;
    std::vector<Region> regions_;
};

class Entry
{
public:
    Entry(uint64_t id, boost::filesystem::path pointcloud_path, boost::filesystem::path annotation_path);

    uint64_t getId() const { return id_; }
    const std::string& getFrame() const;
    uint64_t getTimestamp() const;

    const Annotation& getAnnotation() const;

    template<typename PointT>
    typename pcl::PointCloud<PointT>::ConstPtr getPointcloud() const;

    void getPointcloud(pcl::PCLPointCloud2& pointcloud) const;

private:
    const uint64_t id_;
    const boost::filesystem::path pointcloud_path_;
    const boost::filesystem::path annotation_path_;

    mutable std::unique_ptr<Annotation> annotation_;
};

}}}
