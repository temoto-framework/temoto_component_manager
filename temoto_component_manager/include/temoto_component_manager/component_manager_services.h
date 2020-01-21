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

#ifndef TEMOTO_COMPONENT_MANAGER__COMPONENT_MANAGER_SERVICES_H
#define TEMOTO_COMPONENT_MANAGER__COMPONENT_MANAGER_SERVICES_H

#include <string>
#include "temoto_core/trr/resource_registrar_services.h"
#include "temoto_component_manager/ListComponents.h"
#include "temoto_component_manager/ListPipes.h"
#include "temoto_component_manager/LoadComponent.h"
#include "temoto_component_manager/LoadPipe.h"
#include "temoto_component_manager/Component.h"
#include "temoto_component_manager/Pipe.h"

namespace temoto_component_manager
{
  // TODO: Change the srv_name to something more reasonable
  namespace srv_name
  {
    const std::string MANAGER = "component_manager";
    const std::string SERVER = "load_component";
    const std::string SYNC_TOPIC = "/temoto_component_manager/"+MANAGER+"/sync";

    const std::string MANAGER_2 = "component_manager_pipe";
    const std::string PIPE_SERVER = "load_pipe";

    const std::string LIST_COMPONENTS_SERVER = "list_components_server";
    const std::string LIST_PIPES_SERVER = "list_pipes_server";
  }
}

/**
 * @brief operator ==
 * @param r1
 * @param r2
 * @return
 */
static bool operator==(const temoto_component_manager::LoadComponent::Request& r1,
                       const temoto_component_manager::LoadComponent::Request& r2)
{
  // First check the basics
  if (r1.component_type != r2.component_type ||
      r1.package_name != r2.package_name ||
      r1.executable != r2.executable)
  {
    return false;
  }

  // Check the size of the requested output topics
  if (r1.output_topics.size() != r2.output_topics.size())
  {
    return false;
  }

  // Check the requested output topics
  auto output_topics_2_copy = r2.output_topics;
  for (auto& output_topic_1 : r1.output_topics)
  {
    bool topic_found = false;
    for (auto it=output_topics_2_copy.begin(); it!=output_topics_2_copy.end(); it++)
    {
      if (output_topic_1.key == it->key &&
          output_topic_1.value == it->value)
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

/**
 * @brief 
 * 
 * @param r1 
 * @param r2 
 * @return true 
 * @return false 
 */
static bool operator==(const temoto_component_manager::LoadPipe::Request& r1,
                       const temoto_component_manager::LoadPipe::Request& r2)
{
  if (r1.pipe_category != r2.pipe_category)
  {
    return false;
  }
  
  /*
   * Check if the required output topics match
   */
  for (const auto& output_topic_r1 : r1.output_topics)
  {
    auto found = std::find_if(
      r2.output_topics.begin(),
      r2.output_topics.end(),
      [&] (const diagnostic_msgs::KeyValue& t2)
    { 
      return ((output_topic_r1.key == t2.key) && (output_topic_r1.value == t2.value));
    });

    if (found == r2.output_topics.end())
    {
      return false;
    }
  }

  /*
   * Check if the segment specifiers match
   */
  for (const auto& segment_specifier_r1 : r1.pipe_segment_specifiers)
  {
    bool segment_specifier_found = false;
    for (const auto& segment_specifier_r2 : r2.pipe_segment_specifiers)
    {
      // compare the segment index
      if (segment_specifier_r1.segment_index != segment_specifier_r2.segment_index)
      {
        continue;
      }

      // compare component name
      if (segment_specifier_r1.component_name != segment_specifier_r2.component_name)
      {
        continue;
      }

      // check if the number of parameters is the same
      if (segment_specifier_r1.parameters.size() != segment_specifier_r2.parameters.size())
      {
        continue;
      }

      // check the parameters individually
      auto found = std::find_if(
        segment_specifier_r1.parameters.begin(),
        segment_specifier_r1.parameters.end(),
      [&] (const diagnostic_msgs::KeyValue& parameter_r1)
      { 
        for (const auto& parameter_r2 : segment_specifier_r2.parameters)
        {
          if ((parameter_r1.key == parameter_r2.key) && (parameter_r1.value == parameter_r2.value))
          {
            return true;
          }
        }
        return false;
      });

      if (found == segment_specifier_r1.parameters.end())
      {
        continue;
      }

      segment_specifier_found = true;
      break;
    }

    if (!segment_specifier_found)
    {
      return false;
    }
  }

  /*
   * return
   */     
  return true;
}

#endif
