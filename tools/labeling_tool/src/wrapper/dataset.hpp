#pragma once

#include <string>
#include <vector>
#include <memory>

#include <boost/filesystem.hpp>

#include "entry.hpp"

namespace wrapper
{
class Dataset
{
public:
    void open(const std::string& directory);

    QueryResult query() const;

private:
    std::string directory_;
    std::vector<std::string> ids_;
};
}


namespace interface
{

class Dataset
{
public:
    struct EntryReference
    {
        std::string image_id;

        boost::filesystem::path depth_path;
        boost::filesystem::path visual_path;
        boost::filesystem::path pointcloud_path;
        boost::filesystem::path roi_path;
    };

    enum Visibility
    {
        BACKGROUND = 0,
        HUMAN = 1,
        HUMAN_PART = 2,
        UNKNOWN = 3,
    };

    struct ROI
    {
        std::string image_id;
        double timestamp;
        cv::Rect region;
        Visibility visibility;
    };

    typedef std::vector<ROI> ROIList;

    struct Entry
    {
        std::string image_id;
        ulong timestamp;
        cv::Mat depth_image;
        cv::Mat visual_image;
        pcl::PointCloud<pcl::PointXYZ> pointcloud;
        ROIList roi_list;

        EntryReference reference;

        static Entry* create(const EntryReference& ref);
        void save();
    };

    typedef std::map<std::string, EntryReference> EntryReferenceList;

    enum ContentType
    {
        CONTENT_ROI         = 0x1,
        CONTENT_DEPTH       = 0x2,
        CONTENT_VISUAL      = 0x4,
        CONTENT_POINTCLOUD  = 0x8,

        CONTENT_ALL         = CONTENT_ROI | CONTENT_DEPTH | CONTENT_VISUAL | CONTENT_POINTCLOUD,
        CONTENT_ALL_DEPTH   = CONTENT_ROI | CONTENT_DEPTH,
        CONTENT_ALL_VISUAL  = CONTENT_ROI | CONTENT_VISUAL,
    };

    struct ContentTypeInfo
    {
        std::string subdir;
        std::string file_type;
        std::function<void(Dataset::EntryReference&, const boost::filesystem::path&)> assign;
    };

    Dataset();

    bool load(const boost::filesystem::path& dataset_dir, ContentType types = ContentType::CONTENT_ALL);
    bool save();

    const EntryReferenceList& get_entries() const;
    const EntryReference& get_entry(const std::string& key) const;
    std::vector<std::string> get_chronological_keys() const;


private:
    void load_content(const boost::filesystem::path& base_dir, const ContentTypeInfo& loader);

private:
    boost::filesystem::path dataset_dir;
    EntryReferenceList entry_references;
};

}
