#include "dataset.hpp"
#include <yaml-cpp/yaml.h>
#include <boost/filesystem/operations.hpp>

using namespace csapex::dataset::sand;
namespace bfs = boost::filesystem;

Dataset::Dataset(boost::filesystem::path index_path) :
        index_path_(std::move(index_path))
{
    YAML::Node root = YAML::LoadFile(index_path_.string());
    if (!root.IsMap())
        throw std::runtime_error("Invalid dataset index file (missing root): " + index_path_.string());

    bfs::path pointcloud_root_path;
    bfs::path annotation_root_path;
    {
        YAML::Node path_roots = root["roots"];
        if (!path_roots.IsMap())
            throw std::runtime_error("Invalid dataset index file (missing path roots): " + index_path_.string());

        YAML::Node pointcloud_root = path_roots["pointcloud"];
        if (!pointcloud_root.IsScalar())
            throw std::runtime_error("Invalid dataset index file (missing pointcloud root): " + index_path_.string());

        YAML::Node annotation_root = path_roots["annotation"];
        if (!annotation_root.IsScalar())
            throw std::runtime_error("Invalid dataset index file (missing annotation root): " + index_path_.string());
        
        pointcloud_root_path = pointcloud_root.as<std::string>();
        annotation_root_path = annotation_root.as<std::string>();

        if (pointcloud_root_path.is_relative())
            pointcloud_root_path = index_path_.parent_path() / pointcloud_root_path;

        if (annotation_root_path.is_relative())
            annotation_root_path = index_path_.parent_path() / annotation_root_path;

        if (!bfs::is_directory(pointcloud_root_path))
            throw std::runtime_error("Pointcloud root path does not exist: " + pointcloud_root_path.string());
        if (!bfs::is_directory(annotation_root_path))
            throw std::runtime_error("Annotation root path does not exist: " + annotation_root_path.string());
    }
    
    YAML::Node index = root["index"];
    for (auto node : index)
    {
        auto id = node["id"].as<uint64_t>();
        auto pointcloud_path = bfs::path(node["pointcloud"].as<std::string>());
        auto annotation_path = bfs::path(node["annotation"].as<std::string>());

        if (pointcloud_path.is_relative())
            pointcloud_path = pointcloud_root_path / pointcloud_path;
        if (annotation_path.is_relative())
            annotation_path = annotation_root_path / annotation_path;

        entries_.emplace_back(id, pointcloud_path, annotation_path);
    }
}

const Entry* Dataset::findById(uint64_t id) const
{
    auto itr = std::find_if(begin(), end(), [&id](const Entry& entry) { return entry.getId() == id; });
    if (itr == end())
        return nullptr;
    return &(*itr);
}

void Dataset::add(const Entry& entry)
{
    entries_.push_back(entry);
}
