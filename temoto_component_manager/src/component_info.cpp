#include "temoto_core/common/tools.h"
#include "temoto_component_manager/component_info.h"
#include "ros/ros.h"

namespace temoto_component_manager
{
using namespace temoto_core;

ComponentInfo::ComponentInfo(std::string component_name)
{
  //set the component to current namespace
  temoto_namespace_ = common::getTemotoNamespace();
  component_name_ = component_name;
}

/* * * * * * * * * * * *
   *     GETTERS
   * * * * * * * * * * * */

// Get the temoto namespace where this component is defined
std::string ComponentInfo::getTemotoNamespace() const
{
  return temoto_namespace_;
}

/// Get name
std::string ComponentInfo::getName() const
{
  return component_name_;
}

// Get input topics
const std::vector<StringPair>& ComponentInfo::getInputTopics() const
{
  return input_topics_.getInputTopics();
}

std::vector<diagnostic_msgs::KeyValue> ComponentInfo::getInputTopicsAsKeyVal() const
{
  return input_topics_.inputTopicsAsKeyValues();
}

// Get output topics
const std::vector<StringPair>& ComponentInfo::getOutputTopics() const
{
  return output_topics_.getOutputTopics();
}

std::vector<diagnostic_msgs::KeyValue> ComponentInfo::getOutputTopicsAsKeyVal() const
{
  return output_topics_.outputTopicsAsKeyValues();
}

// Get required parameters
const std::vector<StringPair>& ComponentInfo::getRequiredParameters() const
{
  return required_parameters_.getInputTopics();
}

std::vector<diagnostic_msgs::KeyValue> ComponentInfo::getRequiredParametersAsKeyVal() const
{
  return input_topics_.inputTopicsAsKeyValues();
}

// Get topic by type
std::string ComponentInfo::getTopicByType(const std::string& type, const std::vector<StringPair>& topics)
{
  // Loop over the topics and check the type match. Return the topic if the types amatch
  for(auto&topic : topics)
  {
    if(topic.first == type)
    {
      return topic.second;
    }
  }

  return std::string();
}

// Get output topic
std::string ComponentInfo::getInputTopic(const std::string& type)
{
  return getTopicByType(type, input_topics_.getInputTopics());
}

// Get output topic
std::string ComponentInfo::getOutputTopic(const std::string& type)
{
  return getTopicByType(type, output_topics_.getOutputTopics());
}

// Get component type
std::string ComponentInfo::getType() const
{
  return component_type_;
}

// Get component package name
std::string ComponentInfo::getPackageName() const
{
  return package_name_;
}

// Get executable
std::string ComponentInfo::getExecutable() const
{
  return executable_;
}

// Get description
std::string ComponentInfo::getDescription() const
{
  return description_;
}

// Get reliability
float ComponentInfo::getReliability() const
{
  return reliability_.getReliability();
}

// Is local
bool ComponentInfo::isLocal() const
{
  return getTemotoNamespace() == common::getTemotoNamespace();
}

// Get advertised
bool ComponentInfo::getAdvertised() const
{
  return advertised_;
}

// To string
std::string ComponentInfo::toString() const
{
  std::string ret;
  ret += "COMPONENT: " + getName() + "\n";
  ret += "  temoto_namespace : " + getTemotoNamespace() + "\n";
  ret += "  type             : " + getType() + "\n";
  ret += "  package name     : " + getPackageName() + "\n";
  ret += "  executable       : " + getExecutable() + "\n";
  ret += "  description      : " + getDescription() + "\n";
  ret += "  reliability      : " + std::to_string(getReliability()) + "\n";

  // Print out the input topics
  if (!getInputTopics().empty())
  {
    ret += "  output_topics \n";
    for (auto& topic : getInputTopics())
    {
      ret += "    " + topic.first + " : " + topic.second + "\n";
    }
  }

  // Print out the output topics
  if (!getOutputTopics().empty())
  {
    ret += "  output_topics \n";
    for (auto& topic : getOutputTopics())
    {
      ret += "    " + topic.first + " : " + topic.second + "\n";
    }
  }

  return ret;
}

/* * * * * * * * * * * *
 *     SETTERS
 * * * * * * * * * * * */

void ComponentInfo::setTemotoNamespace(std::string temoto_namespace)
{
  temoto_namespace_ = temoto_namespace;
}

void ComponentInfo::setName(std::string name)
{
  component_name_ = name;
}

void ComponentInfo::addTopicIn(StringPair topic)
{
  input_topics_.addInputTopic(topic.first, topic.second);
}

void ComponentInfo::addTopicOut(StringPair topic)
{
  output_topics_.addOutputTopic(topic.first, topic.second);
}

void ComponentInfo::addRequiredParameter(temoto_core::StringPair required_parameter)
{
  required_parameters_.addInputTopic(required_parameter.first, required_parameter.second);
}

void ComponentInfo::setType(std::string component_type)
{
  component_type_ = component_type;
}

void ComponentInfo::setPackageName(std::string package_name)
{
  package_name_ = package_name;
}

void ComponentInfo::setExecutable(std::string executable)
{
  executable_ = executable;
}

void ComponentInfo::setDescription(std::string description)
{
  description_ = description;
}

void ComponentInfo::setAdvertised(bool advertised)
{
  advertised_ = advertised;
}

void ComponentInfo::adjustReliability(float reliability)
{
  reliability_.adjustReliability(reliability);
}

void ComponentInfo::resetReliability(float reliability)
{
  reliability_.resetReliability(reliability);
}

}  // ComponentManager namespace
