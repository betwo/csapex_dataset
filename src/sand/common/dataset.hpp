#pragma once

#include "entry.hpp"

namespace csapex { namespace dataset { namespace sand {

class Dataset
{
public:
    using iterator = std::vector<Entry>::iterator;
    using const_iterator = std::vector<Entry>::const_iterator;

public:
    Dataset() = default;
    explicit Dataset(boost::filesystem::path index_path);

    std::size_t size() const { return entries_.size(); }
    iterator begin() { return entries_.begin(); }
    iterator end() { return entries_.end(); }
    const_iterator begin() const { return entries_.begin(); }
    const_iterator end() const { return entries_.end(); }

    const Entry& operator[](std::size_t index) const { return entries_[index]; }

    const Entry* findById(uint64_t id) const;
    void add(const Entry& entry);

private:
    boost::filesystem::path index_path_;
    std::vector<Entry> entries_;
};

}}}
