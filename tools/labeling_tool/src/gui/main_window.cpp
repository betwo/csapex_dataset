#include "main_window.h"
#include "ui_main_window.h"

#include <QFileDialog>
#include <QInputDialog>
#include <iostream>
#include "converter/converter.hpp"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    display_ = new gui::ImageDisplay(ui->box_images);
    ui->box_images->layout()->addWidget(display_);
}

void MainWindow::load_dataset()
{
    QString dir = QFileDialog::getExistingDirectory(0, "Open dataset", QDir::currentPath());
    if (!dir.isEmpty())
    {
        _dataset.load(dir.toStdString(), interface::Dataset::ContentType(interface::Dataset::CONTENT_ALL_VISUAL | interface::Dataset::CONTENT_POINTCLOUD));

        _image_ids = _dataset.get_chronological_keys();
        _image_itr = _image_ids.begin();
        _image_prev_itr = _image_ids.end();

        ui->progressBar->setMaximum(_image_ids.size() - 1);
        ui->total_label->setText(QString::fromStdString(std::to_string(_image_ids.size() - 1)));
        ui->roiOverview->clear();
        std::for_each(_image_ids.begin(), _image_ids.end(),
                      [this](const std::string& id)
                      {
                          ui->roiOverview->addItem(QString::fromStdString(id));
                      });

        update_image();
    }
}

void MainWindow::next_image()
{
    auto next = std::next(_image_itr);
    if (next != _image_ids.end())
    {
        _image_prev_itr = _image_itr;
        _image_itr = next;

        update_image();
    }
}

void MainWindow::prev_image()
{
    auto prev = std::prev(_image_itr);
    if (prev != _image_ids.begin())
    {
        _image_prev_itr = _image_itr;
        _image_itr = prev;

        update_image();
    }
}

void MainWindow::seek_image(int idx)
{
    _image_prev_itr = _image_itr;
    _image_itr = std::next(_image_ids.begin(), idx);
    update_image();
}

void MainWindow::update_image()
{
    if (_image_prev_itr != _image_ids.end())
        _changed_rois[*_image_prev_itr] = display_->get();

    const interface::Dataset::EntryReference& ref = _dataset.get_entry(*_image_itr);

    auto index = std::distance(_image_ids.cbegin(), _image_itr);
    ui->progressBar->setValue(index);
    ui->index_label->setText(QString::fromStdString(std::to_string(index)));

    _image_entry.reset(interface::Dataset::Entry::create(ref));

    display_->assign(_image_entry->pointcloud);

    auto changed = _changed_rois.find(ref.image_id);
    if (changed != _changed_rois.end())
        display_->assign(_image_entry->visual_image, changed->second);
    else
        display_->assign(_image_entry->visual_image, _image_entry->roi_list);
}

void MainWindow::reload_options()
{
//    display_->set_confirm_deleted(ui->copyConfirmCheckBox->isChecked());
    display_->set_keep_new(ui->keepNewCheckBox->isChecked());
    display_->set_show_points(ui->showPointsCheckBox->isChecked());
    display_->set_fotonic_flip(ui->fotonicFlipCheckBox->isChecked());
}

void MainWindow::save()
{
    for (const std::pair<std::string, interface::Dataset::ROIList>& pair : _changed_rois)
    {
        const interface::Dataset::EntryReference& ref = _dataset.get_entry(pair.first);
        interface::Dataset::Entry* entry = interface::Dataset::Entry::create(ref);
        entry->roi_list = pair.second;
        std::cout << "Saving: " << entry->image_id << std::endl;
        entry->save();
    }
    _changed_rois.clear();
}


MainWindow::~MainWindow()
{
    delete ui;
}

