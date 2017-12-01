#include "image_display.hpp"

#include <QEvent>
#include <QMouseEvent>
#include <QPainter>
#include <queue>

namespace gui
{

ImageDisplay::ImageDisplay(QWidget *parent) :
    QWidget(parent),
    mode_{Mode::NONE}
{

}

std::pair<ImageDisplay::SelectType, std::pair<std::vector<ImageDisplay::ROIWrapper>::reverse_iterator, int>>
    ImageDisplay::select(const QPoint &point)
{
    const auto check_corner = [&point](const QPointF& corner)
    {
        static const float MARGIN = 10;
        QRectF region{corner.x() - MARGIN / 2, corner.y() - MARGIN / 2, MARGIN, MARGIN};
        return region.contains(point);
    };

//    const auto check_edge = [&](const QPointF& a, const QPointF& b)
//    {
//        static const float MARGIN = 10;
//        QRectF region;
//        if (std::abs(a.y() - b.y()) < 1)
//            region = {a.x(), a.y() - MARGIN / 2, a.x() - b.x(), MARGIN};
//        else
//            region = {a.x() - MARGIN / 2, a.y(), MARGIN, a.y() - b.y()};
//        return region.contains(point);
//    };

    for (auto itr = display_rois.rbegin();
         itr != display_rois.rend();
         ++itr)
    {
        const ROIWrapper& roi = *itr;

        if (roi.deleted)
            continue;

        if (check_corner(roi.draw_region.topLeft()))
            return {ImageDisplay::SelectType::CORNER, {itr, 1}};
        if (check_corner(roi.draw_region.topRight()))
            return {ImageDisplay::SelectType::CORNER, {itr, 2}};
        if (check_corner(roi.draw_region.bottomRight()))
            return {ImageDisplay::SelectType::CORNER, {itr, 3}};
        if (check_corner(roi.draw_region.bottomLeft()))
            return {ImageDisplay::SelectType::CORNER, {itr, 4}};

        /*
        if (check_edge(roi.draw_region.topLeft(), roi.draw_region.topRight()))
            return {ImageDisplay::SelectType::EDGE, {itr, 1}};
        if (check_edge(roi.draw_region.topRight(), roi.draw_region.bottomRight()))
            return {ImageDisplay::SelectType::EDGE, {itr, 2}};
        if (check_edge(roi.draw_region.bottomRight(), roi.draw_region.bottomLeft()))
            return {ImageDisplay::SelectType::EDGE, {itr, 3}};
        if (check_edge(roi.draw_region.bottomLeft(), roi.draw_region.topLeft()))
            return {ImageDisplay::SelectType::EDGE, {itr, 4}};
        */

        if (roi.draw_region.contains(point))
            return {ImageDisplay::SelectType::OBJECT, {itr, 0}};
    }

    return {ImageDisplay::SelectType::NONE, {display_rois.rend(), 0}};
}

void ImageDisplay::mousePressEvent(QMouseEvent *event)
{
    switch (event->button())
    {
    case Qt::MouseButton::LeftButton:
    {
        auto selected = select(event->pos());
        switch (selected.first)
        {
        case ImageDisplay::SelectType::OBJECT:
        case ImageDisplay::SelectType::EDGE:
        case ImageDisplay::SelectType::CORNER:
            if (event->modifiers() & Qt::ShiftModifier)
            {
                ROIWrapper& wrapper = *selected.second.first;
                if (wrapper.roi.visibility == interface::Dataset::Visibility::HUMAN)
                    wrapper.roi.visibility = interface::Dataset::Visibility::HUMAN_PART;
                else
                    wrapper.roi.visibility = interface::Dataset::Visibility::HUMAN;
                wrapper.changed = true;
            }
            else
            {
                mode_ = selected.first == SelectType::OBJECT ? Mode::MOVE_OBJECT :
                                                               (selected.first == SelectType::EDGE ? Mode::MOVE_EDGE :
                                                                                                     Mode::MOVE_CORNER);

                last_point_ = event->pos();
                selected_object_ = selected.second.first;
                selected_object_additional_ = selected.second.second;
            }
            break;
        case ImageDisplay::SelectType::NONE:
        default:
            mode_ = Mode::NONE;
            break;
        }
        break;
    }
    case Qt::MouseButton::RightButton:
    {
        mode_ = Mode::DRAW;
        partial_ = (event->modifiers() & Qt::ShiftModifier) != 0;
        draw_start_point_ = event->pos();
        last_point_ = event->pos();
        break;
    }
    case Qt::MouseButton::MiddleButton:
    {
        auto selected = select(event->pos());
        if (selected.first != SelectType::NONE)
        {
            if (selected.second.first->confirmed)
            {
                selected.second.first->deleted = true;
                if (confirm_deleted_)
                    deleted_points_.push_back(event->pos());
            }
            else
                selected.second.first->confirmed = true;
        }

        mode_ = Mode::NONE;
        break;
    }
    default:
        mode_ = Mode::NONE;
        break;
    }

    this->repaint();
}

void ImageDisplay::mouseReleaseEvent(QMouseEvent *event)
{
    switch(mode_)
    {
    case Mode::DRAW:
    {
        QRectF rect{draw_start_point_, event->pos()};
        if (rect.height() < 20 || rect.width() < 20)
            break;

        ROIWrapper wrapper;
        wrapper.created = true;
        wrapper.draw_region = std::move(rect);
        wrapper.roi.image_id = id_;
        wrapper.roi.timestamp = timestamp_;
        if (partial_)
            wrapper.roi.visibility = interface::Dataset::Visibility::HUMAN_PART;
        else
            wrapper.roi.visibility = interface::Dataset::Visibility::HUMAN;
        display_rois.push_back(wrapper);
        break;
    }
    }

    mode_ = Mode::NONE;

    this->repaint();
}

void ImageDisplay::mouseMoveEvent(QMouseEvent *event)
{
    switch (mode_)
    {
    case Mode::DRAW:
    {
        last_point_ = event->pos();
        break;
    }
    case Mode::MOVE_CORNER:
    case Mode::MOVE_EDGE:
    case Mode::MOVE_OBJECT:
    {
        QPoint delta = event->pos() - last_point_;
        last_point_ = event->pos();

        if (mode_ == Mode::MOVE_OBJECT)
            selected_object_->draw_region.translate(delta);
        else if (mode_ == Mode::MOVE_CORNER)
            switch (selected_object_additional_) {
            case 1: selected_object_->draw_region.setTopLeft(selected_object_->draw_region.topLeft() + delta); break;
            case 2: selected_object_->draw_region.setTopRight(selected_object_->draw_region.topRight() + delta); break;
            case 3: selected_object_->draw_region.setBottomRight(selected_object_->draw_region.bottomRight() + delta); break;
            case 4: selected_object_->draw_region.setBottomLeft(selected_object_->draw_region.bottomLeft() + delta); break;
            default: break;
            }
        else
            switch (selected_object_additional_) {
            case 1: selected_object_->draw_region.setTopLeft(selected_object_->draw_region.topLeft() + QPoint(0, delta.y())); break;
            case 2: selected_object_->draw_region.setTopRight(selected_object_->draw_region.topRight() + QPoint(delta.x(), 0)); break;
            case 3: selected_object_->draw_region.setBottomRight(selected_object_->draw_region.bottomRight() + QPoint(0, delta.y())); break;
            case 4: selected_object_->draw_region.setBottomLeft(selected_object_->draw_region.bottomLeft() + QPoint(delta.x(), 0)); break;
            default: break;
            }
        selected_object_->changed = selected_object_->created ? false : true;
        break;
    }
    }

    this->repaint();
}

void ImageDisplay::resizeEvent(QResizeEvent *event)
{
    rescale(event->size());
}

void ImageDisplay::rescale(const QSize &newsize)
{
    scale_ = std::max((double)orignal_image_.width() / newsize.width(),
                      (double)orignal_image_.height() / newsize.height());

    display_image_ = orignal_image_.scaled(newsize, Qt::KeepAspectRatio);
}

QRectF ImageDisplay::create_region(const cv::Rect &region)
{
    return QRectF{region.x / scale_, region.y / scale_, region.width / scale_, region.height / scale_};
}

void ImageDisplay::assign(const std::string &id, double timestamp)
{
    this->id_ = id;
    this->timestamp_ = timestamp;
}

void ImageDisplay::assign(const pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    const auto is_finite = [](const pcl::PointXYZ& point)
    {
        return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    };
    const auto is_zero = [](const pcl::PointXYZ& point)
    {
        return point.x == 0 && point.y == 0 && point.z == 0;
    };
    const auto is_fotonic_zero = [](const pcl::PointXYZ& point)
    {
        static const double eps = 0.001;
        return std::abs(point.x) < eps && std::abs(point.y - -0.23) < eps && std::abs(point.z - 0.1) < eps;
    };
    const auto depth = [](const pcl::PointXYZ &point)
    {
        return sqrt(point.x * point.x +
                    point.y * point.y +
                    point.z * point.z);
    };

    valid_points_ = cv::Mat(cloud.height, cloud.width, CV_8UC1, cv::Scalar(1));
    norm_depth_points_ = cv::Mat(cloud.height, cloud.width, CV_32FC1, cv::Scalar(0));
    for (int y = 0; y < cloud.height; ++y)
        for (int x = 0; x < cloud.width; ++x)
        {
            int ry = y;
            if (fotoinic_flip_) {
                ry = cloud.height - y - 1;
            }

            valid_points_.at<uint8_t>(y, x) = is_finite(cloud.at(x, ry)) && !is_zero(cloud.at(x, ry)) && (!fotoinic_flip_ || !is_fotonic_zero(cloud.at(x, ry)));


            if(valid_points_.at<uint8_t>(y,x) == 1) {
                double d = depth(cloud.at(x,ry));
                if(d < 12.0)
                    norm_depth_points_.at<float>(y,x) = d;
                else
                    valid_points_.at<uint8_t>(y,x) = 0;
            }
        }
        cv::normalize(norm_depth_points_, norm_depth_points_, 0.0, 1.0, CV_MINMAX);
}

void ImageDisplay::assign(const cv::Mat& image, const interface::Dataset::ROIList& rois)
{
    orignal_image_ = QImage(image.cols, image.rows, QImage::Format::Format_ARGB32);
    for (int y = 0; y < image.rows; ++y)
        for (int x = 0; x < image.cols; ++x)
        {
            if (image.type() == CV_8UC3)
            {
                const cv::Vec3b& color = image.at<cv::Vec3b>(y, x);
                orignal_image_.setPixel(x, y, qRgba(color[2], color[1], color[0], 255));
            }
            else if (image.type() == CV_16UC1)
            {
                static const auto normalizer = 2046.0 / (255.0 * 4.0);
                const uint16_t color = image.at<uint16_t>(y, x);
                orignal_image_.setPixel(x, y, qRgba(color / normalizer, color / normalizer, color / normalizer, 255));
            }
            else
            {
                unsigned char color = image.at<uint8_t>(y, x);
                orignal_image_.setPixel(x, y, qRgba(color, color, color, 255));
            }
        }
    rescale(this->size());

    display_rois.erase(std::remove_if(display_rois.begin(), display_rois.end(),
                                      [this](const ROIWrapper& wrap) { return keep_new_ ? !wrap.created : true; }),
                       display_rois.end());
    for (const interface::Dataset::ROI& roi : rois)
    {
        ROIWrapper wrapper;
        wrapper.roi = roi;
        wrapper.draw_region = create_region(wrapper.roi.region);

        if (confirm_deleted_)
            for (const QPoint& pt : deleted_points_)
                if (wrapper.draw_region.contains(pt))
                    wrapper.confirmed = false;

        display_rois.push_back(wrapper);
    }

    this->repaint();
}

interface::Dataset::ROIList ImageDisplay::get()
{
    deleted_points_.clear();

    interface::Dataset::ROIList rois;
    for (ROIWrapper& wrapper : display_rois)
    {
        if (wrapper.deleted || !wrapper.confirmed)
        {
            if (!confirm_deleted_)
                deleted_points_.push_back(QPoint(wrapper.draw_region.center().x(), wrapper.draw_region.center().y()));
            continue;
        }

        if (!wrapper.changed && !wrapper.created)
            rois.push_back(wrapper.roi);
        else
        {
            wrapper.roi.region = cv::Rect{wrapper.draw_region.x() * scale_,
                    wrapper.draw_region.y() * scale_,
                    wrapper.draw_region.width() * scale_,
                    wrapper.draw_region.height() * scale_};
            rois.push_back(wrapper.roi);
        }
    }    
    return rois;
}

namespace color {
static const cv::Point3f red(0.f,0.f,255.f);  /// P0
static const cv::Point3f green(0.f,255.f,0.f);  /// P1
static const cv::Point3f blue(255.f,0.f,0.f);  /// p3
static const cv::Point3f fac1  (blue - 2 * green + red);
static const cv::Point3f fac2  (-2*blue + 2 * green);

inline QColor bezierColor(const float value,
                          const float alpha = 0.25)
{
    cv::Point3f  col = fac1 * value * value + fac2 * value + blue;
    return  QColor(std::floor(col.x + .5), std::floor(col.y + .5), std::floor(col.z + .5), alpha * 255);
}

}


void ImageDisplay::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.drawImage(0, 0, display_image_);

    for (const QPoint& pt : deleted_points_)
    {
        p.setPen(QPen(Qt::red, 2));
        p.drawEllipse(pt, 4, 4);
    }

    for (const ROIWrapper& roi : display_rois)
    {
        if (roi.deleted)
            continue;

        if (roi.confirmed)
            if (roi.roi.visibility == interface::Dataset::Visibility::HUMAN_PART)
                p.setPen(QPen(Qt::yellow, 2));
            else
                p.setPen(QPen(Qt::blue, 2));
        else
            p.setPen(QPen(Qt::red, 2));
        p.drawRect(roi.draw_region);

        if (show_points_)
        {
            /// p.setPen(QPen(QBrush(QColor(0, 255, 0, 24)), 1));
            for (int y = roi.draw_region.y(); y < roi.draw_region.y() + roi.draw_region.height(); ++y)
                for (int x = roi.draw_region.x(); x < roi.draw_region.x() + roi.draw_region.width(); ++x)
                {
                    if (valid_points_.at<uint8_t>(y * scale_, x * scale_) == 1) {
                        p.setPen(QPen(QBrush(color::bezierColor(norm_depth_points_.at<float>(y * scale_, x * scale_))), 1));
                        p.drawEllipse(QPoint(x, y), 1, 1);
                    }
                }
        }
    }

    if (mode_ == Mode::DRAW)
    {
        QRectF rect{draw_start_point_, last_point_};
        if (partial_)
            p.setPen(QPen(Qt::yellow, 2, Qt::DashLine));
        else
            p.setPen(QPen(Qt::blue, 2, Qt::DashLine));

        p.drawRect(rect);

        if (show_points_)
        {
            p.setPen(QPen(QBrush(QColor(0, 255, 0, 24)), 1));
            for (int y = draw_start_point_.y(); y < last_point_.y(); ++y)
                for (int x = draw_start_point_.x(); x < last_point_.x(); ++x)
                {
                    if (valid_points_.at<uint8_t>(y * scale_, x * scale_) == 1)
                        p.drawEllipse(QPoint(x, y), 1, 1);
                }
        }
    }
}

void ImageDisplay::set_confirm_deleted(bool value)
{
    confirm_deleted_ = value;
    deleted_points_.clear();
}

void ImageDisplay::set_merge_rois(bool value)
{
    merge_rois_ = value;
}

void ImageDisplay::set_keep_new(bool value)
{
    keep_new_ = value;
}

void ImageDisplay::set_show_points(bool value)
{
    show_points_ = value;
    this->repaint();
}

void ImageDisplay::set_fotonic_flip(bool value)
{
    fotoinic_flip_ = value;
}

}
