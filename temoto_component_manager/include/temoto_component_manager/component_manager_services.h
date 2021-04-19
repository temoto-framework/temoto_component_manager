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

#ifndef TEMOTO_COMPONENT_MANAGER__COMPONENT_MANAGER_SERVICES_H
#define TEMOTO_COMPONENT_MANAGER__COMPONENT_MANAGER_SERVICES_H

#include <string>
#include "temoto_core/trr/resource_registrar_services.h"
#include "temoto_resource_registrar/temoto_error.h"
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
    const std::string PIPE_SERVER = "load_pipe";
    const std::string SYNC_TOPIC = "/temoto_component_manager/"+MANAGER+"/sync";

    const std::string LIST_COMPONENTS_SERVER = "list_components_server";
    const std::string LIST_PIPES_SERVER = "list_pipes_server";
  }
}

static bool operator==(const temoto_component_manager::LoadComponent::Request& r1,
                       const temoto_component_manager::LoadComponent& r2)
{
  /*
   * This operator assumes that the response portion of previous queries (r2) includes
   * all the details regarding the component that was provided per request. Thus ambiguous
   * queries can be compared against previous queries.
   */ 

  if (r1.component_type != r2.request.component_type)
  {
    return false;
  }

  if (!r1.package_name.empty() && r1.package_name != r2.response.package_name)
  {
    return false;
  }

  if (!r1.executable.empty() && r1.executable != r2.response.executable)
  {
    return false;
  }

  /*
   * Check the input topics
   */ 
  if (r1.input_topics.size() > r2.response.input_topics.size())
  {
    return false;
  }

  auto r2_input_topics_copy = r2.response.input_topics;
  for(const auto& r1_input_topic : r1.input_topics)
  {
    bool topic_found = false;
    for (auto it=r2_input_topics_copy.begin(); it!=r2_input_topics_copy.end(); it++)
    {
      if (r1_input_topic.key == it->key && r1_input_topic.value.empty())
      {
        topic_found = true;
        r2_input_topics_copy.erase(it);
        break;
      }
      else if (r1_input_topic.key == it->key && !r1_input_topic.value.empty())
      {
        // If the keys are equal but the topics are different this request is unique
        if (r1_input_topic.value == it->value)
        {
          topic_found = true;
          r2_input_topics_copy.erase(it);
          break;
        }
        else
        {
          return false;
        }
      }
    }
    if (!topic_found)
    {
      return false;
    }
  }

  /*
   * Check the output topics
   */ 
  if (r1.output_topics.size() > r2.response.output_topics.size())
  {
    return false;
  }

  auto r2_output_topics_copy = r2.response.output_topics;
  for(const auto& r1_output_topic : r1.output_topics)
  {
    bool topic_found = false;
    for (auto it=r2_output_topics_copy.begin(); it!=r2_output_topics_copy.end(); it++)
    {
      if (r1_output_topic.key == it->key && r1_output_topic.value.empty())
      {
        topic_found = true;
        r2_output_topics_copy.erase(it);
        break;
      }
      else if (r1_output_topic.key == it->key && !r1_output_topic.value.empty())
      {
        // If the keys are equal but the topics are different this request is unique
        if (r1_output_topic.value == it->value)
        {
          topic_found = true;
          r2_output_topics_copy.erase(it);
          break;
        }
        else
        {
          return false;
        }
      }
    }
    if (!topic_found)
    {
      return false;
    }
  }

  /*
   * Check the parameters
   */
  if (r1.required_parameters.size() > r2.response.required_parameters.size())
  {
    return false;
  }

  auto r2_parameters_copy = r2.response.required_parameters;
  for(const auto& r1_parameter : r1.required_parameters)
  {
    bool parameter_found = false;
    for (auto it=r2_parameters_copy.begin(); it!=r2_parameters_copy.end(); it++)
    {
      if (r1_parameter.key == it->key && r1_parameter.value == it->value)
      {
        parameter_found = true;
        r2_parameters_copy.erase(it);
        break;
      }
    }
    if (!parameter_found)
    {
      return false;
    }
  }

  // The component requests are equal
  return true;
}

static bool operator==(const temoto_component_manager::LoadPipe::Request& r1,
                       const temoto_component_manager::LoadPipe& r2)
{
  if (r1.pipe_category != r2.request.pipe_category)
  {
    return false;
  }
  
  /*
   * Check if the required output topics match
   */
  for (const auto& output_topic_r1 : r1.output_topics)
  {
    auto found = std::find_if(
      r2.request.output_topics.begin(),
      r2.request.output_topics.end(),
      [&] (const diagnostic_msgs::KeyValue& t2)
    { 
      return ((output_topic_r1.key == t2.key) && (output_topic_r1.value == t2.value));
    });

    if (found == r2.request.output_topics.end())
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
    for (const auto& segment_specifier_r2 : r2.request.pipe_segment_specifiers)
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
