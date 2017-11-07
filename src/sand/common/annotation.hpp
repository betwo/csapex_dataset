#pragma once

#include <boost/filesystem/path.hpp>
#include <yaml-cpp/yaml.h>

namespace csapex { namespace dataset { namespace sand {

class Annotation
{
    friend class YAML::convert<Annotation>;
public:
    enum Class
    {
        CLASS_BACKGROUND = -1,
        CLASS_UNKNOWN = 0,
        CLASS_PERSON = 1,
        CLASS_PARTIAL_PERSON = 2,
    };

    struct Region
    {
        int x;
        int y;
        int width;
        int height;
        Class clazz;
    };

public:
    uint64_t getTimestamp() const { return timestamp_; }
    const std::string& getFrame() const { return frame_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }

    const std::vector<Region>& getRegions() const { return regions_; }
    std::vector<Region>& getRegions() { return regions_; }

public:
    static Annotation load(boost::filesystem::path path);
    static void save(const Annotation& annotation);
    static void save(const Annotation& annotation, const boost::filesystem::path& path);

private:
    boost::filesystem::path path_;

    uint64_t timestamp_;
    std::string frame_;
    int width_;
    int height_;
    std::vector<Region> regions_;
};

}}}

namespace YAML {
template<>
struct convert<csapex::dataset::sand::Annotation::Class>
{
    static Node encode(const csapex::dataset::sand::Annotation::Class& clazz);
    static bool decode(const Node& node, csapex::dataset::sand::Annotation::Class& clazz);
};

template<>
struct convert<csapex::dataset::sand::Annotation::Region>
{
    static Node encode(const csapex::dataset::sand::Annotation::Region& clazz);
    static bool decode(const Node& node, csapex::dataset::sand::Annotation::Region& clazz);
};

template<>
struct convert<csapex::dataset::sand::Annotation>
{
    static Node encode(const csapex::dataset::sand::Annotation& annoation);
    static bool decode(const Node& node, csapex::dataset::sand::Annotation& annoation);
};
}
