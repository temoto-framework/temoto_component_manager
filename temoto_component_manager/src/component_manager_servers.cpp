#include "ros/package.h"
#include "temoto_component_manager/component_manager_servers.h"
#include <algorithm>
#include <utility>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <regex>

namespace temoto_component_manager
{
using namespace temoto_core;

ComponentManagerServers::ComponentManagerServers(BaseSubsystem *b, ComponentInfoRegistry *cir)
  : BaseSubsystem(*b, __func__)
  , cir_(cir)
  , resource_manager_(srv_name::MANAGER, this)
{
  // Start the server
  resource_manager_.addServer<LoadComponent>( srv_name::SERVER
                                            , &ComponentManagerServers::loadComponentCb
                                            , &ComponentManagerServers::unloadComponentCb);
  // Register callback for status info
  resource_manager_.registerStatusCb(&ComponentManagerServers::statusCb);


  TEMOTO_INFO("Component manager is ready.");
}

ComponentManagerServers::~ComponentManagerServers()
{
}

void ComponentManagerServers::statusCb(temoto_core::ResourceStatus& srv)
{

  TEMOTO_DEBUG("Received a status message.");

  // If local component failed, adjust package reliability and advertise to other managers via
  // synchronizer.
  if (srv.request.status_code == rmp::status_codes::FAILED)
  {
    auto it = allocated_components_.find(srv.request.resource_id);
    if (it != allocated_components_.end())
    {
      if(it->second.isLocal())
      {
        TEMOTO_WARN("Local component failure detected, adjusting reliability.");
        it->second.adjustReliability(0.0);
        cir_->updateLocalComponent(it->second);
      }
      else
      {
        TEMOTO_WARN("Remote component failure detected, doing nothing (component will be updated via synchronizer).");
      }
    }
  }
}

bool ComponentManagerServers::listComponentsCb( ListComponents::Request& req
                                              , ListComponents::Response& res)
{
  // std::vector <std::string> devList;

  // Find the devices with the required type
  for (const auto& component : cir_->getLocalComponents())
  {
    if (component.getType() == req.type)
    {
      res.list.push_back(component.getName());
    }
  }

  return true;
}

// TODO: rename "loadComponentCb" to "loadComponentCb"
void ComponentManagerServers::loadComponentCb( LoadComponent::Request& req
                                             , LoadComponent::Response& res)
{
  TEMOTO_INFO_STREAM("- - - - - - - - - - - - -\n"
                     << "Received a request to load a component: \n" << req << std::endl);

  // Try to find suitable candidate from local components
  std::vector<ComponentInfo> l_cis;
  std::vector<ComponentInfo> r_cis;

  bool got_local_components = cir_->findLocalComponents(req, l_cis);
  bool got_remote_components = cir_->findRemoteComponents(req, r_cis);

  // Find the most reliable global component but do not forward the requests
  // that originate from other namespaces
  bool prefer_remote = false;
  if (got_local_components && got_remote_components
      && (req.rmp.temoto_namespace == common::getTemotoNamespace()))
  {
    if (l_cis.at(0).getReliability() < r_cis.at(0).getReliability())
    {
      prefer_remote = true;
    }
  }

  if (got_local_components && !prefer_remote)
  {
    // Loop over suitable components
    for (ComponentInfo& ci : l_cis)
    {
      // Check if this component is already in use (providing some other types of data)
      // If thats the case, then set up a topic remapper
      ComponentInfo ci_in_use;
      if (checkIfInUse(ci, ci_in_use))
      {
        // query the component from er_manager
      }

      // Try to run the component via local Resource Manager
      temoto_er_manager::LoadExtResource load_er_msg;
      load_er_msg.request.action = temoto_er_manager::action::ROS_EXECUTE;
      load_er_msg.request.package_name = ci.getPackageName();
      load_er_msg.request.executable = ci.getExecutable();

      // Remap the input topics if requested
      processTopics(req.input_topics, res.input_topics, load_er_msg, ci, "in");

      // Remap the output topics if requested
      processTopics(req.output_topics, res.output_topics, load_er_msg, ci, "out");

      TEMOTO_INFO( "ComponentManagerServers found a suitable local component: '%s', '%s', '%s', reliability %.3f"
                 , load_er_msg.request.action.c_str()
                 , load_er_msg.request.package_name.c_str()
                 , load_er_msg.request.executable.c_str()
                 , ci.getReliability());

      try
      {
        resource_manager_.call<temoto_er_manager::LoadExtResource>( temoto_er_manager::srv_name::MANAGER
                                                     , temoto_er_manager::srv_name::SERVER
                                                     , load_er_msg
                                                     , rmp::FailureBehavior::NONE);

        TEMOTO_DEBUG("Call to ProcessManager was sucessful.");

        // Fill out the response about which particular component was chosen
        res.package_name = ci.getPackageName();
        res.executable = ci.getExecutable();
        res.rmp = load_er_msg.response.rmp;

        ci.adjustReliability(1.0);
        cir_->updateLocalComponent(ci);
        allocated_components_.emplace(res.rmp.resource_id, ci);

        return;
      }
      catch(error::ErrorStack& error_stack)
      {
        if (error_stack.front().code != static_cast<int>(error::Code::SERVICE_REQ_FAIL))
        {
          ci.adjustReliability(0.0);
          cir_->updateLocalComponent(ci);
        }
        SEND_ERROR(error_stack);
      }
    }
  }

  if (got_remote_components)
  {
    // Loop over suitable components
    for (ComponentInfo& ci : r_cis)
    {
      // remote component candidate was found, forward the request to the remote component manager
      LoadComponent load_component_msg;
      load_component_msg.request.component_type = ci.getType();
      load_component_msg.request.package_name = ci.getPackageName();
      load_component_msg.request.executable = ci.getExecutable();
      load_component_msg.request.input_topics = req.input_topics;
      load_component_msg.request.output_topics = req.output_topics;

      TEMOTO_INFO( "Component Manager is forwarding request: '%s', '%s', '%s', reliability %.3f"
                 , ci.getType().c_str()
                 , ci.getPackageName().c_str()
                 , ci.getExecutable().c_str()
                 , ci.getReliability());

      try
      {
        resource_manager_.call<LoadComponent>( srv_name::MANAGER
                                             , srv_name::SERVER
                                             , load_component_msg
                                             , rmp::FailureBehavior::NONE
                                             , ci.getTemotoNamespace());

        TEMOTO_DEBUG("Call to remote ComponentManagerServers was sucessful.");
        res = load_component_msg.response;
        allocated_components_.emplace(res.rmp.resource_id, ci);
      }
      catch(error::ErrorStack& error_stack)
      {
        throw FORWARD_ERROR(error_stack);
      }
      return;
    }
  }
  else
  {
    // no suitable local nor remote component was found
    throw CREATE_ERROR(error::Code::COMPONENT_NOT_FOUND, "ComponentManagerServers did not find a suitable component.");
  }
}

// TODO: rename "unloadComponentCb" to "unloadComponentCb"
void ComponentManagerServers::unloadComponentCb( LoadComponent::Request& req
                                               , LoadComponent::Response& res)
{
  TEMOTO_DEBUG("received a request to stop component with id '%ld'", res.rmp.resource_id);
  allocated_components_.erase(res.rmp.resource_id);
  return;
}

void ComponentManagerServers::processTopics( std::vector<diagnostic_msgs::KeyValue>& req_topics
                                           , std::vector<diagnostic_msgs::KeyValue>& res_topics
                                           , temoto_er_manager::LoadExtResource& load_er_msg
                                           , ComponentInfo& component_info
                                           , std::string direction)
{
  /*
   * Find out it this is a launch file or not. Remapping is different
   * for executable types (launch files or executables)
   */
  bool isLaunchFile;
  std::regex rx(".*\\.launch$");
  isLaunchFile = std::regex_match(component_info.getExecutable(), rx);

  // Work with input or output topics
  std::vector<StringPair> component_info_topics;

  if (direction == "in")
  {
    component_info_topics = component_info.getInputTopics();
  }
  else if (direction == "out")
  {
    component_info_topics = component_info.getOutputTopics();
  }

  // If no topics were requested, then return a list of all topics this component publishes
  if (req_topics.empty())
  {
    for (const auto& output_topic : component_info_topics)
    {
      diagnostic_msgs::KeyValue topic_msg;
      topic_msg.key = output_topic.first;
      topic_msg.value = common::getAbsolutePath(output_topic.second);
      res_topics.push_back(topic_msg);
    }
    return;
  }

  // Remap the input topics if requested
  for (auto& req_topic : req_topics)
  {
    // And return the input topics via response
    diagnostic_msgs::KeyValue res_topic;
    res_topic.key = req_topic.key;
    std::string default_topic;

    if (direction == "in")
    {
      default_topic = component_info.getInputTopic(req_topic.key);
    }
    else if (direction == "out")
    {
      default_topic = component_info.getOutputTopic(req_topic.key);
    }

    if (req_topic.value != "")
    {
      res_topic.value = common::getAbsolutePath(req_topic.value);

      // Remap depending wether it is a launch file or excutable
      std::string remap_arg;

      if (isLaunchFile)
      {
        remap_arg = req_topic.key + ":=" + req_topic.value;
      }
      else
      {
        remap_arg = default_topic + ":=" + req_topic.value;
      }

      load_er_msg.request.args += remap_arg + " ";
    }
    else
    {
      res_topic.value = common::getAbsolutePath(default_topic);
    }

    // Add the topic to the response message
    res_topics.push_back(res_topic);
  }
}

bool ComponentManagerServers::checkIfInUse(const ComponentInfo& ci_to_check, ComponentInfo& ci_return) const
{
  for (auto const& ac : allocated_components_)
  {
    if (ac.second.getPackageName() != ci_to_check.getPackageName())
    {
      return false;
    }

    if (ac.second.getExecutable() != ci_to_check.getExecutable())
    {
      return false;
    }

    ci_return = ac.second;
  }
  return true;
}

}  // component_manager namespace
