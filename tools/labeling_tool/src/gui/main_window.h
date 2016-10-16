#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>
#include "widget/image_display.hpp"

#include <memory>
#include <map>
#include <rosbag/view.h>
#include "wrapper/dataset.hpp"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void load_dataset();
    void save();
    void prev_image();
    void next_image();
    void seek_image(int idx);
    void reload_options();

private:
    void update_image();

private:
    Ui::MainWindow *ui;
    gui::ImageDisplay* display_;

    interface::Dataset _dataset;
    std::map<std::string, interface::Dataset::ROIList> _changed_rois;
    std::vector<std::string> _image_ids;
    std::vector<std::string>::const_iterator _image_itr;
    std::vector<std::string>::const_iterator _image_prev_itr;

    std::unique_ptr<interface::Dataset::Entry> _image_entry;
};

#endif // MAIN_WINDOW_H
