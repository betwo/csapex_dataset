#pragma once

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>

namespace conversion
{

template<typename PointT = pcl::PointXYZI>
inline pcl::PointCloud<PointT> ros_depth_to_pcl(sensor_msgs::PointCloud2::ConstPtr msg)
{
    pcl::PointCloud<PointT> pcl;
    pcl::fromROSMsg(*msg, pcl);
    return pcl;
}

template<typename PointT>
inline cv::Mat pcl_to_depth_image(const pcl::PointCloud<PointT>& pcl)
{
    cv::Mat img(pcl.height, pcl.width, CV_32FC1);
    for (int y = 0; y < img.rows; ++y)
        for (int x = 0; x < img.cols; ++x)
        {
            const PointT& pt = pcl.at(x, y);

            img.at<float>(y, x) = pt.x;
        }

    return img;
}

template<typename PointT>
inline cv::Mat pcl_to_intensity_image(const pcl::PointCloud<PointT>& pcl);

template<>
inline cv::Mat pcl_to_intensity_image(const pcl::PointCloud<pcl::PointXYZI>& pcl)
{
    cv::Mat img(pcl.height, pcl.width, CV_32FC1);
    for (int y = 0; y < img.rows; ++y)
        for (int x = 0; x < img.cols; ++x)
        {
            const pcl::PointXYZI& pt = pcl.at(x, y);

            img.at<float>(y, x) = pt.intensity;
        }

    return img;
}

template<>
inline cv::Mat pcl_to_intensity_image(const pcl::PointCloud<pcl::PointXYZRGB>& pcl)
{
    cv::Mat img(pcl.height, pcl.width, CV_8UC3);
    for (int y = 0; y < img.rows; ++y)
        for (int x = 0; x < img.cols; ++x)
        {
            const pcl::PointXYZRGB& pt = pcl.at(x, y);
            const uint32_t rgb = *reinterpret_cast<const int*>(&pt.rgb);
            uint8_t r = (rgb >> 16) & 0x0000ff;
            uint8_t g = (rgb >> 8)  & 0x0000ff;
            uint8_t b = (rgb)       & 0x0000ff;

            img.at<cv::Vec3b>(y, x) = cv::Vec3b(b, g, r);
        }

    return img;
}

inline cv::Mat for_display(const cv::Mat& input)
{
    cv::Mat out;

    double max;
    cv::minMaxLoc(input, nullptr, &max);
    input.convertTo(out, CV_8UC1, 255 / max);

    return out;
}

}
