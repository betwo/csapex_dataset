#pragma once

#include <csapex/model/tickable_node.h>

namespace csapex
{
namespace dataset
{

class PeopleDatasetStats : public csapex::TickableNode
{
public:
   PeopleDatasetStats();

   void setupParameters(csapex::Parameterizable& parameter) override;
   void setup(csapex::NodeModifier& node_modifier) override;
   virtual void tick() override;

private:
   void load();

private:
   csapex::Output* out_stats_;

   std::string stats_;
   std::string path_;
   csapex::param::Parameter::Ptr range_;
};

}
}
