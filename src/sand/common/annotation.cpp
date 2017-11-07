#include "annotation.hpp"
#include <fstream>

using namespace csapex::dataset::sand;
namespace bfs = boost::filesystem;

Annotation Annotation::load(boost::filesystem::path path)
{
    YAML::Node node = YAML::LoadFile(path.string());

    Annotation annotation = node.as<Annotation>();
    annotation.path_ = path;

    return annotation;
}

void Annotation::save(const Annotation& annotation)
{
    return save(annotation, annotation.path_);
}

void Annotation::save(const Annotation& annotation, const boost::filesystem::path& path)
{
    YAML::Node node;
    node = annotation;

    std::ofstream file(path.string());
    file << node;
}


namespace YAML
{
Node convert<Annotation::Class>::encode(const Annotation::Class& clazz)
{
    return Node(static_cast<int>(clazz));
}

bool convert<Annotation::Class>::decode(const Node& node, Annotation::Class& clazz)
{
    if (!node.IsScalar())
        return false;

    clazz = static_cast<Annotation::Class>(node.as<int>());
    return true;
}

Node convert<Annotation::Region>::encode(const Annotation::Region& region)
{
    Node node;
    node["x"] = region.x;
    node["y"] = region.y;
    node["width"] = region.width;
    node["height"] = region.height;
    node["class"] = region.clazz;
    return node;
}

bool convert<Annotation::Region>::decode(const Node& node, Annotation::Region& region)
{
    if (!node.IsMap())
        return false;

    region.x = node["x"].as<int>();
    region.y = node["y"].as<int>();
    region.width = node["width"].as<int>();
    region.height = node["height"].as<int>();
    region.clazz = node["class"].as<Annotation::Class>();
    return true;
}

Node convert<Annotation>::encode(const Annotation& annotation)
{
    Node node;
    node["timestamp"] = annotation.timestamp_;
    node["frame"] = annotation.frame_;
    node["width"] = annotation.width_;
    node["height"] = annotation.height_;
    node["regions"] = annotation.regions_;
    return node;
}

bool convert<Annotation>::decode(const Node& node, Annotation& annotation)
{
    if (!node.IsMap())
        return false;

    annotation.timestamp_ = node["timestamp"].as<uint64_t>();
    annotation.frame_ = node["frame"].as<std::string>();
    annotation.width_ = node["width"].as<int>();
    annotation.height_ = node["height"].as<int>();
    annotation.regions_ = node["regions"].as<std::vector<Annotation::Region>>();
    return true;
}
}
