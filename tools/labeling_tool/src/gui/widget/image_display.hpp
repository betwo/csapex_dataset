#pragma once

#include <QWidget>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "wrapper/dataset.hpp"

namespace gui
{

class ImageDisplay : public QWidget
{
    Q_OBJECT
private:
    struct ROIWrapper
    {
        interface::Dataset::ROI roi;
        bool deleted = false;
        bool changed = false;
        bool created = false;
        bool confirmed = true;
        QRectF draw_region;
    };

public:
    ImageDisplay(QWidget* parent);

    void assign(const cv::Mat& image, const interface::Dataset::ROIList& rois);
    void assign(const pcl::PointCloud<pcl::PointXYZ>& cloud);
    void assign(const std::string& id, double timestamp);
    void set_confirm_deleted(bool value);
    void set_merge_rois(bool value);
    void set_keep_new(bool value);
    void set_show_points(bool value);
    void set_fotonic_flip(bool value);
    interface::Dataset::ROIList get();

private:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

    void rescale(const QSize &newsize);
    QRectF create_region(const cv::Rect& region);

    enum SelectType { NONE, OBJECT, CORNER, EDGE };
    std::pair<SelectType, std::pair<std::vector<ROIWrapper>::reverse_iterator, int>> select(const QPoint& point);


private:
    std::string id_;
    double timestamp_;

    enum class Mode { NONE, DRAW, MOVE_OBJECT, MOVE_EDGE, MOVE_CORNER, RESIZE };
    Mode mode_ = Mode::NONE;
    bool partial_ = false;
    QPoint last_point_;
    std::vector<ROIWrapper>::reverse_iterator selected_object_;
    int selected_object_additional_;
    QPoint draw_start_point_;

    QImage orignal_image_;
    double scale_;
    QImage display_image_;
    std::vector<ROIWrapper> display_rois;

    bool confirm_deleted_ = false;
    std::vector<QPoint> deleted_points_;

    bool merge_rois_ = false;
    std::vector<QRectF> merged_rois_;

    bool keep_new_ = false;
    cv::Mat valid_points_;
    cv::Mat norm_depth_points_;
    bool show_points_ = true;
    bool fotoinic_flip_ = false;
};

}
