#include "dataset_common.hpp"

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>

namespace fs = boost::filesystem;

void csapex::dataset::people::save_fill_meta(Entry& entry, const std::string& directory)
{
    entry.meta.path_visual = fs::path(directory) / "visual" / (entry.meta.id + ".ppm");
    entry.meta.path_depth  = fs::path(directory) / "depth" / (entry.meta.id + ".yaml.tar.gz");
    entry.meta.path_pcl    = fs::path(directory) / "pointcloud" / (entry.meta.id + ".pcd");
    entry.meta.path_roi    = fs::path(directory) / "roi" / (entry.meta.id + ".yaml");

    entry.meta.size.width = entry.visual.cols;
    entry.meta.size.height = entry.visual.rows;
}

void csapex::dataset::people::save(const Entry& entry)
{
    {
        fs::create_directories(entry.meta.path_depth.parent_path());

        cv::FileStorage depth(entry.meta.path_depth.string(), cv::FileStorage::WRITE);
        depth << "data" << entry.depth;
        depth.release();
    }
    {
        fs::create_directories(entry.meta.path_visual.parent_path());

        cv::imwrite(entry.meta.path_visual.string(), entry.visual);
    }
    {
        fs::create_directories(entry.meta.path_pcl.parent_path());

        pcl::io::savePCDFileBinaryCompressed(entry.meta.path_pcl.string(), entry.pointcloud);
    }
    {
        fs::create_directories(entry.meta.path_roi.parent_path());

        YAML::Emitter yaml;
        yaml << YAML::BeginMap;
        yaml << "img_d_w" << entry.meta.size.width;
        yaml << "img_d_h" << entry.meta.size.height;
        yaml << "rois" << YAML::BeginSeq;
        for (const ROIType& roi : entry.meta.rois)
        {
            yaml << YAML::BeginMap;
            yaml << "ts" << roi.stamp_micro_seconds;
            yaml << "img_id" << entry.meta.id;
            yaml << "x_d" << roi.value.rect().x;
            yaml << "y_d" << roi.value.rect().y;
            yaml << "w_d" << roi.value.rect().width;
            yaml << "h_d" << roi.value.rect().height;
            yaml << "x_rgb" << roi.value.rect().x;
            yaml << "y_rgb" << roi.value.rect().y;
            yaml << "w_rgb" << roi.value.rect().width;
            yaml << "h_rgb" << roi.value.rect().height;
            yaml << "vis" << roi.value.classification();
            yaml << YAML::EndMap;
        }
        yaml << YAML::EndSeq;
        yaml << "ts" << entry.meta.timestamp;
        yaml << YAML::EndMap;

        std::ofstream out(entry.meta.path_roi.string());
        out << yaml.c_str();
        out.close();
    }
}

void csapex::dataset::people::load_dataset(const std::string &directory, std::vector<MetaEntry> &entries)
{
    fs::path roi_dir = fs::path(directory) / "roi";

    for (fs::directory_iterator itr(roi_dir); itr != fs::directory_iterator(); ++itr)
    {
        fs::path file = itr->path();
        if (fs::is_regular_file(file)
                && fs::extension(file) == ".yaml")
        {
            std::string id = fs::basename(file);

            MetaEntry entry = load_entry(directory, id);
            entries.push_back(entry);
        }
    }
}

csapex::dataset::people::MetaEntry csapex::dataset::people::load_entry(const std::string &directory, const std::string &id)
{
    MetaEntry entry;
    entry.id = id;
    entry.path_visual = fs::path(directory) / "visual" / (id + ".ppm");
    entry.path_depth  = fs::path(directory) / "depth" / (id + ".yaml.tar.gz");
    entry.path_pcl    = fs::path(directory) / "pointcloud" / (id + ".pcd");
    entry.path_roi    = fs::path(directory) / "roi" / (id + ".yaml");

    if (!fs::exists(entry.path_visual))
        throw std::runtime_error("Inconsistent content for '" + id + "' - missing visual");
    if (!fs::exists(entry.path_depth))
        throw std::runtime_error("Inconsistent content for '" + id + "' - missing depth");
    if (!fs::exists(entry.path_pcl))
        throw std::runtime_error("Inconsistent content for '" + id + "' - missing pcl");
    if (!fs::exists(entry.path_roi))
        throw std::runtime_error("Inconsistent content for '" + id + "' - missing roi");

    YAML::Node yaml = YAML::LoadFile(entry.path_roi.string());
    if (!yaml.IsMap())
        throw std::runtime_error("Inconsistent content for '" + id + "' - roi yaml invalid");

//    entry.timestamp = yaml["ts"].as<ulong>();
    entry.timestamp = std::stoull(id );
    entry.size.width = yaml["img_d_w"].as<int>();
    entry.size.height = yaml["img_d_h"].as<int>();

    cv::Rect crop_mask(0, 0, entry.size.width, entry.size.height);

    YAML::Node rois = yaml["rois"];
    if (!rois.IsSequence())
        throw std::runtime_error("Inconsistent content for '" + id + "' - roi yaml invalid");

    for (YAML::iterator itr = rois.begin(); itr != rois.end(); ++itr)
    {
        const YAML::Node& node = *itr;
        if (!node.IsMap())
            throw std::runtime_error("Inconsistent content for '" + id + "' - roi yaml invalid");

        cv::Rect rect;
        rect.x = node["x_d"].as<int>();
        rect.y = node["y_d"].as<int>();
        rect.width = node["w_d"].as<int>();
        rect.height = node["h_d"].as<int>();

        rect = rect & crop_mask;
        if (rect.height <= 0
                || rect.width <= 0)
            continue;

        int vis = node["vis"].as<int>();
        if (vis != HUMAN
                && vis != HUMAN_PART)
            continue;

        ROIType roi;
        roi.value.setRect(rect);
        roi.value.setClassification(vis);
        auto cl = ClassificationColors.find(vis);
        if (cl != ClassificationColors.end())
            roi.value.setColor(cl->second);
        else
            roi.value.setColor(ClassificationColors.at(UNKNOWN));

        entry.rois.push_back(roi);
    }

    return entry;
}

void csapex::dataset::people::read_to(const MetaEntry &entry, cv::Mat &visual, cv::Mat &depth, pcl::PointCloud<pcl::PointXYZI> &cloud)
{
    cv::FileStorage depth_fs(entry.path_depth.string(), cv::FileStorage::READ);
    depth_fs["data"] >> depth;
    depth_fs.release();

    visual = cv::imread(entry.path_visual.string(), cv::IMREAD_UNCHANGED);

    pcl::io::loadPCDFile<pcl::PointXYZI>(entry.path_pcl.string(), cloud);
}
