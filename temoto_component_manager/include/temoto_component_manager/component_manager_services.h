#ifndef TEMOTO_COMPONENT_MANAGER__COMPONENT_MANAGER_SERVICES_H
#define TEMOTO_COMPONENT_MANAGER__COMPONENT_MANAGER_SERVICES_H

#include <string>
#include "temoto_core/rmp/resource_manager_services.h"
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
  return( (r1.pipe_category == r2.pipe_category) &&
          (r1.detection_method == r2.detection_method));
  /*
   * TODO: Add more comparison metrics
   */ 
}

#endif
