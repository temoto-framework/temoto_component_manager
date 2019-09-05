/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2019 TeMoto Telerobotics
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* Author: Robert Valner */

#ifndef TEMOTO_COMPONENT_MANAGER__COMPONENT_INFO_H
#define TEMOTO_COMPONENT_MANAGER__COMPONENT_INFO_H

#include "temoto_core/common/temoto_log_macros.h"
#include "temoto_core/common/topic_container.h" // temoto_core::StringPair
#include "temoto_core/common/reliability.h"
#include <string>
#include <vector>
#include <map>
#include <ctype.h>
#include <memory> // shared_ptr
#include <yaml-cpp/yaml.h>

namespace temoto_component_manager
{

class ComponentInfo
{
public:

  /**
   * @brief ComponentInfo
   * @param component_name
   */
  ComponentInfo(std::string component_name = "A noname component");
  
  // To string
  std::string toString() const;

  /* * * * * * * * * * * *
   *     GETTERS
   * * * * * * * * * * * */

  // Get the temoto namespace where this component is defined
  std::string getTemotoNamespace() const;

  /// Get name
  std::string getName() const;

  // Get input topics
  const std::vector<temoto_core::StringPair>& getInputTopics() const;
  std::vector<diagnostic_msgs::KeyValue> getInputTopicsAsKeyVal() const;

  // Get output topics
  const std::vector<temoto_core::StringPair>& getOutputTopics() const;
  std::vector<diagnostic_msgs::KeyValue> getOutputTopicsAsKeyVal() const;

  // Get output topics
  const std::vector<temoto_core::StringPair>& getRequiredParameters() const;
  std::vector<diagnostic_msgs::KeyValue> getRequiredParametersAsKeyVal() const;

  // Get topic by type
  std::string getTopicByType(const std::string& type, const std::vector<temoto_core::StringPair>& topics);

  // Get input topic
  std::string getInputTopic(const std::string& type);

  // Get output topic
  std::string getOutputTopic(const std::string& type);

  // Get required parameter
  std::string getRequiredParameter(const std::string& type);
  

  // Get component type
  std::string getType() const;

  // Get component package name
  std::string getPackageName() const;

  // Get executable
  std::string getExecutable() const;

  // Get description
  std::string getDescription() const;

  // Get reliability
  float getReliability() const;

  // Is local
  bool isLocal() const;

  // Get advertised
  bool getAdvertised() const;


  /* * * * * * * * * * * *
   *     SETTERS
   * * * * * * * * * * * */

  void setTemotoNamespace(std::string temoto_namespace);

  void setName(std::string name);

  void addTopicIn(temoto_core::StringPair topic);

  void addTopicOut(temoto_core::StringPair topic);

  void addRequiredParameter(temoto_core::StringPair required_parameter);

  void setType(std::string component_type);

  void setPackageName(std::string package_name);

  void setExecutable(std::string executable);

  void setDescription(std::string description);

  void setAdvertised(bool advertised);

  void adjustReliability(float reliability);

  void resetReliability(float reliability);


private:
  
  std::string temoto_namespace_;
  std::string component_name_;
  std::string component_type_;
  std::string package_name_;
  std::string executable_;
  std::string description_;
  temoto_core::Reliability reliability_;
  temoto_core::TopicContainer input_topics_;
  temoto_core::TopicContainer output_topics_;
  temoto_core::TopicContainer required_parameters_;
  bool advertised_ = false;
};

typedef std::shared_ptr<ComponentInfo> ComponentInfoPtr;
typedef std::vector<ComponentInfoPtr> ComponentInfoPtrs;

// TODO: Not sure how to declare operators in a header, implement them in src
//       and not brake everything (linking problems in places where
//       component_info.cpp has to be linked)
static bool operator==(const ComponentInfo& s1, const ComponentInfo& s2)
{
  // Check the namespace, executable and name of the package
  if (s1.getTemotoNamespace() != s2.getTemotoNamespace() ||
      s1.getExecutable() != s2.getExecutable() ||
      s1.getPackageName() != s2.getPackageName())
  {
    return false;
  }

  // Check the size of input and output topics
  if (s1.getInputTopics().size() != s2.getInputTopics().size() ||
      s1.getOutputTopics().size() != s2.getOutputTopics().size())
  {
    return false;
  }

  // Check the input topics
  auto input_topics_2_copy = s2.getInputTopics();
  for (auto& input_topic_1 : s1.getInputTopics())
  {
    bool topic_found = false;
    for (auto it=input_topics_2_copy.begin(); it!=input_topics_2_copy.end(); it++)
    {
      if (input_topic_1.first == it->first)
      {
        topic_found = true;
        input_topics_2_copy.erase(it);
        break;
      }
    }

    if (!topic_found)
    {
      return false;
    }
  }

  // Check the output topics
  auto output_topics_2_copy = s2.getOutputTopics();
  for (auto& output_topic_1 : s1.getOutputTopics())
  {
    bool topic_found = false;
    for (auto it=output_topics_2_copy.begin(); it!=output_topics_2_copy.end(); it++)
    {
      if (output_topic_1.first == it->first)
      {
        topic_found = true;
        output_topics_2_copy.erase(it);
        break;
      }
    }

    if (!topic_found)
    {
      return false;
    }
  }

  // The component infos are equal
  return true;
}
} // namespace temoto_component_manager

namespace YAML
{
template <>
struct convert<temoto_component_manager::ComponentInfo>
{
  static Node encode(const temoto_component_manager::ComponentInfo& component)
  {
    Node node;
    node["component_name"] = component.getName();
    node["component_type"] = component.getType();
    node["package_name"] = component.getPackageName();
    node["executable"] = component.getExecutable();
    node["description"] = component.getDescription();
    node["reliability"] = component.getReliability();

    Node input_topics_node;
    for (auto& topics : component.getInputTopics())
    {
      input_topics_node[topics.first] = topics.second;
    }
    node["input_topics"] = input_topics_node;

    Node output_topics_node;
    for (auto& topics : component.getOutputTopics())
    {
      output_topics_node[topics.first] = topics.second;
    }
    node["output_topics"] = output_topics_node;

    Node required_parameters_node;
    for (auto& parameters : component.getRequiredParameters())
    {
      required_parameters_node[parameters.first] = parameters.second;
    }
    node["required_parameters"] = required_parameters_node;

    return node;
  }

  static bool decode(const Node& node, temoto_component_manager::ComponentInfo& component)
  {
    if (!node.IsMap() || node.size() < 5)
    {
      return false;
    }

    uint8_t code;

    // Convert the compulsory fields
    try
    {
      code = 1;
      component.setName(node["component_name"].as<std::string>());

      code = 2;
      component.setType(node["component_type"].as<std::string>());

      code = 3;
      component.setPackageName(node["package_name"].as<std::string>());

      code = 4;
      component.setExecutable(node["executable"].as<std::string>());
    }
    catch (YAML::InvalidNode e)
    {
      // print out the error message
      switch(code)
      {
        case 5:
          std::cout << "Something is wrong with the 'input_topics'\n";
          break;

        case 6:
          std::cout << "Something is wrong with the 'output_topics'\n";
          break;
      }

      return false;
    }

    /*
     * These fields are optional
     */

    // Get the input_topics
    try
    {
      Node input_topics_node = node["input_topics"];
      for (YAML::const_iterator node_it = input_topics_node.begin()
          ; node_it != input_topics_node.end()
          ; ++node_it)
      {
        component.addTopicIn({node_it->first.as<std::string>(), node_it->second.as<std::string>()});
      }
    }
    catch (YAML::InvalidNode e)
    {
    }

    // Get the output_topics
    try
    {
      Node output_topics_node = node["output_topics"];
      for (YAML::const_iterator node_it = output_topics_node.begin()
          ; node_it != output_topics_node.end()
          ; ++node_it)
      {
        component.addTopicOut({node_it->first.as<std::string>(), node_it->second.as<std::string>()});
      }
    }
    catch(YAML::InvalidNode e)
    {
    }

    // Get the required_parameters
    try
    {
      Node required_parameters_node = node["required_parameters"];
      for (YAML::const_iterator node_it = required_parameters_node.begin()
          ; node_it != required_parameters_node.end()
          ; ++node_it)
      {
        component.addRequiredParameter({node_it->first.as<std::string>(), node_it->second.as<std::string>()});
      }
    }
    catch(YAML::InvalidNode e)
    {
    }

    // Get the description
    try
    {
      component.setDescription(node["description"].as<std::string>());
    }
    catch (YAML::InvalidNode e)
    {
    }

    // Get the reliability
    try
    {
      component.resetReliability(node["reliability"].as<float>());
    }
    catch (YAML::InvalidNode e)
    {
    }

    return true;
  }
};
}
#endif
