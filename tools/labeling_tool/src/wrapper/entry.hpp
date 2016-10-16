#pragma once

#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/PointCloud2.h>

namespace wrapper
{
struct ROI
{
    double timestamp;
    cv::Rect depth;
    cv::Rect rgb;
    int label;
};

class Entry
{
public:
    virtual void save(const std::string& directory) const = 0;

    virtual const std::string& id() const = 0;
    virtual const cv::Mat depth_image() const = 0;
    virtual const cv::Mat intensity_image() const = 0;
    virtual const pcl::PointCloud<pcl::PointXYZ> point_cloud() const = 0;

    virtual const std::vector<ROI> rois() const { return rois_; }
    virtual void set_rois(const std::vector<ROI>& rois) { rois_ = rois; }

protected:
    std::vector<ROI> rois_;
};

typedef std::vector<std::shared_ptr<Entry>> QueryResult;

class DatasetEntry : public Entry
{
public:
    DatasetEntry();

    static std::shared_ptr<DatasetEntry> create(const std::string& directory, const std::string& id);

    void save(const std::string &directory) const;

    const std::string& id() const override { return id_; }
    const cv::Mat depth_image() const override;
    const cv::Mat intensity_image() const override;
    const pcl::PointCloud<pcl::PointXYZ> point_cloud() const override;

    void set_rois(const std::vector<ROI>& rois) override;

private:
    std::vector<ROI> original_rois_;
    bool changed_;
    std::string directory_;
    std::string id_;
};

}
