#ifndef READ_INRIA_HPP
#define READ_INRIA_HPP

#include <fstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

namespace csapex
{
namespace dataset
{
namespace people
{

struct Instance {
    enum Label {NEGATIVE = -1, POSITIVE = 1};

    std::string             path;
    std::vector<cv::Rect>   rois;
    std::vector<cv::Point>  centers;
    Label                   label;

    typedef std::map<std::string, Instance> Set;

};

bool fileExists(const std::string &path)
{
    std::ifstream in(path);
    bool found = in.is_open();
    in.close();
    return found;
}

void split(const std::string &str,
               const char delimiter,
               std::vector<std::string> &tokens)
{
  std::stringstream ss(str);
  std::string tok;

  while(std::getline(ss, tok, delimiter)) {
    tokens.emplace_back(tok);
  }
}

bool readTuple(const std::string &str,
               std::pair<int,int> &tuple)
{
    if(str.empty())
        return false;

    std::string str_tuple = str;
    std::size_t brace = str_tuple.find('(');
    str_tuple.replace(brace, 1, "");
    brace = str_tuple.find(')');
    str_tuple.replace(brace, 1, "");
    std::size_t comma = str_tuple.find(',');
    str_tuple.replace(comma, 1, " ");
    std::stringstream ss;
    ss << str_tuple;
    if(!ss)
        return false;
    ss >> tuple.first;
    if(!ss)
        return false;
    ss >> tuple.second;

    return true;
}

std::size_t readAnnotations(std::ifstream &in_ann,
                            std::vector<cv::Point> &centers,
                            std::vector<cv::Rect> &rois)
{
    bool found_center = false;
    bool found_bounding = false;
    std::size_t found = 0;
    std::string line;
    std::pair<int,int> center;
    std::pair<int,int> bb_min;
    std::pair<int,int> bb_max;
    while(std::getline(in_ann, line)) {
        std::vector<std::string> tokens;
        split(line, ' ', tokens);
        if(tokens.empty())
            continue;

        if(tokens.front() == "Center") {
            tokens.clear();
            split(line, ':', tokens);
            if(readTuple(tokens.back(), center)) {
                found_center = true;
            } else {
                std::cerr << "Couldn't read tuple!" << std::endl;
            }
        } else if(tokens.front() == "Bounding") {
            tokens.clear();
            split(line, ':', tokens);
            std::string bb = tokens.back();
            tokens.clear();
            split(bb, '-', tokens);
            if(readTuple(tokens.front(), bb_min) &&
                    readTuple(tokens.back(), bb_max)) {
                found_bounding = true;
            } else {
                std::cerr << "Couldn't read bounding box!" << std::endl;
            }
        }
        if(found_center && found_bounding) {
            centers.push_back(cv::Point(center.first, center.second));
            rois.push_back(cv::Rect(bb_min.first, bb_min.second,
                                    bb_max.first - bb_min.first,
                                    bb_max.second - bb_min.second));
            ++found;
            found_center = false;
            found_bounding = false;
        }
    }
    return found;
}

std::size_t readPos(std::ifstream &in_pos,
                    std::ifstream &in_ann,
                    const std::string &root_path,
                    std::map<std::string, Instance> &lst)
{
    std::size_t found = 0;
    /// 1. load all the positive example images
    std::string line;
    while(std::getline(in_pos, line)) {
        Instance i;
        i.path = root_path + line;
        std::vector<std::string> tokens;
        split(line, '/', tokens);
        std::string name = tokens.back();
        tokens.clear();
        split(name, '.', tokens);
        std::string id = tokens.front();
        lst[id] = i;
    }

    /// 2. load all the annotations belonging to that
    while(std::getline(in_ann, line)) {
        std::vector<std::string> tokens;
        split(line, '/', tokens);
        std::string name = tokens.back();
        tokens.clear();
        split(name, '.', tokens);
        std::string id = tokens.front();

        line = root_path + line;
        std::ifstream in_rois(line);
        if(!in_rois.is_open()) {
            std::cerr << "annotation [" << line << "] not found, dropping set entry!" << std::endl;
            lst.erase(id);
        } else {
            if(lst.find(id) == lst.end()) {
                std::cerr << "Found annotations for non-existing file!" << std::endl;
                std::cerr << id << std::endl;
                continue ;
            }

            std::vector<cv::Rect> &rois = lst[id].rois;
            std::vector<cv::Point> &centers = lst[id].centers;
            found += readAnnotations(in_rois, centers, rois);
            lst[id].label = Instance::POSITIVE;
        }
    }
    return found;
}

std::size_t readNeg(std::ifstream &in_neg,
                    std::map<std::string, Instance> &lst,
                    const std::string &root_path,
                    const cv::Size &roi_size)
{
    /// 1. load all the negative example images
    std::string line;
    while(std::getline(in_neg, line)) {
        Instance i;
        i.path = root_path + line;
        std::vector<std::string> tokens;
        split(line, '/', tokens);
        std::string name = tokens.back();
        tokens.clear();
        split(name, '.', tokens);
        std::string id = tokens.front();
        lst[id] = i;
    }

    /// 2. generate 10 negative rois per image
    std::default_random_engine generator;
    std::size_t found = 0;
    for(auto &e : lst) {
        cv::Mat img = cv::imread(e.second.path);
        cv::Size interval = cv::Size(img.cols - roi_size.width - 1,
                                    img.rows - roi_size.height - 1);

        std::uniform_int_distribution<int> distribution_x(0, interval.width);
        std::uniform_int_distribution<int> distribution_y(0, interval.height);
        for(std::size_t i = 0 ; i < 10 ; ++i) {
            cv::Rect roi(distribution_x(generator),
                         distribution_y(generator),
                         roi_size.width,
                         roi_size.height);
            e.second.rois.push_back(roi);
            ++found;
        }
        e.second.label = Instance::NEGATIVE;
    }
    return found;
}

void readFolder(const std::string &path_src_folder,
                Instance::Set &pos_samples,
                Instance::Set &neg_samples,
                std::size_t &pos_sample_count,
                std::size_t &neg_sample_count,
                const cv::Size &neg_window_size = cv::Size(64, 128),
                const bool debug = false)
{
    std::string path = path_src_folder;
    if(path.back() != '/') {
        path += "/";
    }

    std::ifstream in_neg(path + "neg.lst");
    std::ifstream in_pos(path + "pos.lst");
    std::ifstream in_ann(path + "annotations.lst");
    if(!in_neg.is_open()) {
        throw std::runtime_error("File 'neg.lst' not found in supplied source folder!");
    }
    if(!in_pos.is_open()) {
        throw std::runtime_error("File 'pos.lst' not found in supplied source folder!");
    }
    if(!in_ann.is_open()) {
        throw std::runtime_error("File 'annotations.lst' not found in supplied source folder!");
    }

    std::string root_path = ""; /// the root of the complete inria set.
    root_path = path.substr(0, path.size() - 1);
    std::size_t pos = root_path.find_last_of('/');
    root_path = root_path.substr(0, pos);
    root_path += "/";

    pos_sample_count = readPos(in_pos, in_ann, root_path, pos_samples);
    neg_sample_count = readNeg(in_neg, neg_samples, root_path, neg_window_size);

    if(debug) {
        std::cout << "Found '" << pos_sample_count << "' positive and '" << neg_sample_count << "' negative instances." << std::endl;
    }
}

inline std::vector<std::size_t> randomVector(const std::size_t size)
{
    std::vector<std::size_t> random_vector(size);
    for(std::size_t i = 0 ; i < size ; ++i){
        random_vector[i] = i;
    }

    std::random_shuffle(random_vector.begin(), random_vector.end());
    return random_vector;
}

}

}

}

#endif // READ_INRIA_HPP
