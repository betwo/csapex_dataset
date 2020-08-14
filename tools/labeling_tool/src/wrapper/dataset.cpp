#include "dataset.hpp"

#include <fstream>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace wrapper
{

void Dataset::open(const std::string &directory)
{
    this->directory_ = directory;

    const fs::path DIR = fs::path(directory) / "roi";

    for (fs::directory_iterator itr(DIR); itr != fs::directory_iterator(); ++itr)
    {
        const fs::path file = *itr;
        if (file.string().find(".yaml") != std::string::npos)
            this->ids_.push_back(file.stem().string());
    }

    std::sort(ids_.begin(), ids_.end());
}

QueryResult Dataset::query() const
{
    QueryResult res;

    for (const std::string& id : ids_)
    {
        std::cout << "loading " << id << std::endl;
        auto entry = DatasetEntry::create(directory_, id);
        res.emplace_back(entry);
    }

    return res;
}

}

#include <functional>
#include <boost/algorithm/string.hpp>
#include <yaml-cpp/yaml.h>
#include <pcl/io/pcd_io.h>

namespace interface
{

Dataset::Entry* Dataset::Entry::create(const EntryReference &ref)
{
    Dataset::Entry* entry = new Dataset::Entry();
    entry->reference = ref;
    entry->image_id = ref.image_id;

    if (!ref.depth_path.empty())
    {
        cv::FileStorage fs(ref.depth_path.string(), cv::FileStorage::READ);
        fs["data"] >> entry->depth_image;
        fs.release();
    }

    if (!ref.visual_path.empty())
    {
        entry->visual_image = cv::imread(ref.visual_path.string(), -1);
        if(entry->visual_image.type() == CV_16UC1) {
            cv::normalize(entry->visual_image, entry->visual_image, 0, 255, cv::NORM_MINMAX, CV_8U);
            cv::equalizeHist(entry->visual_image, entry->visual_image);
            cv::cvtColor(entry->visual_image, entry->visual_image, cv::COLOR_GRAY2BGR);
        }
    }

    if (!ref.pointcloud_path.empty())
    {
        pcl::io::loadPCDFile(ref.pointcloud_path.string(), entry->pointcloud);
    }

    if (!ref.roi_path.empty())
    {
        YAML::Node file = YAML::LoadFile(ref.roi_path.string());

        entry->timestamp = file["ts"].as<ulong>();

        YAML::Node rois = file["rois"];
        for (YAML::Node node : rois)
        {
            ROI roi;
            roi.image_id = ref.image_id;
            roi.timestamp = node["ts"].as<double>();
            roi.visibility = static_cast<Visibility>(node["vis"].as<int>());
            roi.region = {node["x_d"].as<int>(), node["y_d"].as<int>(), node["w_d"].as<int>(), node["h_d"].as<int>()};
            entry->roi_list.push_back(roi);
        }
    }

    return entry;
}

void Dataset::Entry::save()
{
    YAML::Emitter yaml;
    yaml << YAML::BeginMap;
    yaml << "img_d_w" << visual_image.cols;
    yaml << "img_d_h" << visual_image.rows;
    yaml << "rois" << YAML::BeginSeq;
    for (const ROI& roi : roi_list)
    {
        yaml << YAML::BeginMap;
        yaml << "ts" << roi.timestamp;
        yaml << "img_id" << roi.image_id;
        yaml << "x_d" << roi.region.x;
        yaml << "y_d" << roi.region.y;
        yaml << "w_d" << roi.region.width;
        yaml << "h_d" << roi.region.height;
        yaml << "x_rgb" << roi.region.x;
        yaml << "y_rgb" << roi.region.y;
        yaml << "w_rgb" << roi.region.width;
        yaml << "h_rgb" << roi.region.height;
        yaml << "vis" << static_cast<int>(roi.visibility);
        yaml << YAML::EndMap;
    }
    yaml << YAML::EndSeq;
    yaml << "ts" << (ulong) (timestamp);
    yaml << YAML::EndMap;

    std::ofstream out(reference.roi_path.string());
    out << yaml.c_str();
    out.close();
}

static const std::map<Dataset::ContentType, Dataset::ContentTypeInfo> CONTENTTYPE_TO_SUBDIR{
    {Dataset::ContentType::CONTENT_ROI, {"roi", ".yaml", [](Dataset::EntryReference& ref, const boost::filesystem::path& path) {ref.roi_path = path;}} },
    {Dataset::ContentType::CONTENT_DEPTH, {"depth", ".yaml.tar.gz", [](Dataset::EntryReference& ref, const boost::filesystem::path& path) {ref.depth_path = path;}}},
    {Dataset::ContentType::CONTENT_VISUAL, {"visual", ".ppm", [](Dataset::EntryReference& ref, const boost::filesystem::path& path) {ref.visual_path = path;}}},
    {Dataset::ContentType::CONTENT_POINTCLOUD, {"pointcloud", ".pcd", [](Dataset::EntryReference& ref, const boost::filesystem::path& path) {ref.pointcloud_path = path;}}},
};

Dataset::Dataset()
{

}

bool Dataset::load(const boost::filesystem::path &dataset_dir, ContentType types)
{
    this->dataset_dir = dataset_dir;

    if (types == 0)
        throw std::exception();

    entry_references.clear();

    for (auto type_info : CONTENTTYPE_TO_SUBDIR)
    {
        if ((types & type_info.first) != 0)
            load_content(dataset_dir, type_info.second);
    }

    std::transform(entry_references.begin(), entry_references.end(), std::ostream_iterator<std::string>(std::cout, "\n"),
                   [](const std::pair<std::string, EntryReference>& pair)
    {
        return pair.first;
    });
}

void Dataset::load_content(const boost::filesystem::path &base_dir, const ContentTypeInfo &loader)
{
    const boost::filesystem::path dir = base_dir / loader.subdir;

    for (boost::filesystem::directory_iterator itr(dir); itr != boost::filesystem::directory_iterator(); ++itr)
    {
        const boost::filesystem::path& path = *itr;

        if (boost::algorithm::ends_with(path.string(), loader.file_type))
        {
            std::string image_id = path.filename().string();
            image_id = image_id.substr(0, image_id.size() - loader.file_type.length());

            EntryReference& ref = entry_references[image_id];
            ref.image_id = image_id;

            loader.assign(ref, path);
        }
    }
}

const Dataset::EntryReferenceList& Dataset::get_entries() const
{
    return entry_references;
}

const Dataset::EntryReference& Dataset::get_entry(const std::string& key) const
{
    static const EntryReference EMPTY{};

    auto itr = entry_references.find(key);
    if (itr == entry_references.end())
        return EMPTY;
    else
        return itr->second;
}

std::vector<std::string> Dataset::get_chronological_keys() const
{
    std::vector<std::string> keys;

    std::transform(entry_references.begin(), entry_references.end(), std::back_inserter(keys), [](const EntryReferenceList::value_type& pair) { return pair.first; });
    std::sort(keys.begin(), keys.end());

    return keys;
}

}
