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
  , resource_manager_1_(srv_name::MANAGER, this)
  , resource_manager_2_(srv_name::MANAGER_2, this)
{
  // Start the component server
  resource_manager_1_.addServer<LoadComponent>( srv_name::SERVER
                                              , &ComponentManagerServers::loadComponentCb
                                              , &ComponentManagerServers::unloadComponentCb);

  resource_manager_2_.addServer<LoadPipe>( srv_name::PIPE_SERVER
                                         , &ComponentManagerServers::loadPipeCb
                                         , &ComponentManagerServers::unloadPipeCb);

  // Register callback for status info
  resource_manager_1_.registerStatusCb(&ComponentManagerServers::statusCb1);
  resource_manager_2_.registerStatusCb(&ComponentManagerServers::statusCb2);

  TEMOTO_INFO("Component manager is ready.");
}

ComponentManagerServers::~ComponentManagerServers()
{
}

/*
 * ComponentManagerServers::statusCb1
 */
void ComponentManagerServers::statusCb1(temoto_core::ResourceStatus& srv)
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

/*
 * ComponentManagerServers::statusCb2
 */
void ComponentManagerServers::statusCb2(temoto_core::ResourceStatus& srv)
{
  TEMOTO_DEBUG("Received a status message.");

  // If local sensor failed, adjust package reliability and advertise to other managers via
  // synchronizer.
  if (srv.request.status_code == temoto_core::rmp::status_codes::FAILED)
  {
    TEMOTO_DEBUG("A resource, that a running pipe depends on, has failed");

    int val = srv.request.resource_id;

    auto it = std::find_if(allocated_pipes_hack_.begin(), allocated_pipes_hack_.end(),
              [val](const std::pair<int, std::pair<PipeInfo, std::vector<int>>>& pair_in)
              {
                for (const auto& client_id : pair_in.second.second)
                {
                  if (client_id == val)
                  {
                    return true;
                  }
                }
                return false;
              });

    if (it != allocated_pipes_hack_.end())
    {
      TEMOTO_INFO("Tracker of type '%s' (pipe size: %d) has stopped working",
                   it->second.first.getType().c_str(),
                   it->second.first.getPipeSize());

      // Reduce the reliability of the pipe
      it->second.first.reliability_.adjustReliability(0);
      cir_->updatePipe(it->second.first);
    }
  }
}

/*
 * ComponentManagerServers::listComponentsCb
 */
bool ComponentManagerServers::listComponentsCb( ListComponents::Request& req
                                              , ListComponents::Response& res)
{
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

/*
 * ComponentManagerServers::loadComponentCb
 */
void ComponentManagerServers::loadComponentCb( LoadComponent::Request& req
                                             , LoadComponent::Response& res)
{
  TEMOTO_DEBUG_STREAM("Received a request to load a component: \n" << req << std::endl);

  // Try to find suitable candidate from local components
  std::vector<ComponentInfo> l_cis;
  std::vector<ComponentInfo> r_cis;

  bool got_local_components = cir_->findLocalComponents(req, l_cis);
  bool got_remote_components = cir_->findRemoteComponents(req, r_cis);

  // Find the most reliable global component but do not forward the requests
  // that originate from other namespaces
  bool prefer_remote = false;
  if (got_local_components
      && got_remote_components
      && (req.rmp.temoto_namespace == common::getTemotoNamespace()))
  {
    /*
     * TODO: This section should be expanded with more coosing metrics.
     * Currently only reliabilites are compared but cost of communication
     * delays, etc. should also be considered when a component is chosen
     * from a remote namespace.
     */  
    if (l_cis.at(0).getReliability() < r_cis.at(0).getReliability())
    {
      prefer_remote = true;
    }
  }

  if (got_local_components && !prefer_remote)
  {
    /*
     * Check if the requested component is already in use  but providing some other 
     * types of data (compared to current request) If thats the case, then set up 
     * a topic remapper
     * 
     * TODO: Add a feature (boolean) to allow components not to be "reused" like that
     */ 
    temoto_er_manager::LoadExtResource load_er_msg;
    temoto_core::temoto_id::ID alloc_comp_id = checkIfInUse(l_cis);

    if (alloc_comp_id != temoto_core::temoto_id::UNASSIGNED_ID)
    {
      load_er_msg = allocated_ext_resources_.at(alloc_comp_id);
      ComponentInfo& alloc_comp_info = allocated_components_.at(alloc_comp_id);

      try
      {
        /*
         * Load the component. Even though the component is already loaded, the query is still sent
         * to External Resource Manager with the same parameters - Why? Because in that case the 
         * External Resource Manager does not start another component but rather just increases
         * it's use count.
         */ 
        resource_manager_1_.call<temoto_er_manager::LoadExtResource>( temoto_er_manager::srv_name::MANAGER
                                                      , temoto_er_manager::srv_name::SERVER
                                                      , load_er_msg
                                                      , rmp::FailureBehavior::NONE);

        /*
         * Set up the topics that are returned to the client
         */

        // Input topics 
        for (const auto& input_topic : req.input_topics)
        {
          if (input_topic.value.empty())
          {
            // If the client did not ask the data to be remapped, then return the default topic
            diagnostic_msgs::KeyValue in_tpc;
            in_tpc.key = input_topic.key;
            in_tpc.value = alloc_comp_info.getInputTopic(input_topic.key);
            res.input_topics.push_back(in_tpc);
          }
          else
          {
            // Set up the remapper
            temoto_er_manager::LoadExtResource load_er_msg_remapper;
            load_er_msg_remapper.request.action = temoto_er_manager::action::ROS_EXECUTE;
            load_er_msg_remapper.request.package_name = "topic_tools";
            load_er_msg_remapper.request.executable = "relay";
            load_er_msg_remapper.request.args = alloc_comp_info.getInputTopic(input_topic.key) + " " + input_topic.value;
            
            resource_manager_1_.call<temoto_er_manager::LoadExtResource>( temoto_er_manager::srv_name::MANAGER
                                                      , temoto_er_manager::srv_name::SERVER
                                                      , load_er_msg_remapper
                                                      , rmp::FailureBehavior::NONE);

            res.input_topics.push_back(input_topic);
          }
        }

        // Output topics 
        for (const auto& output_topic : req.output_topics)
        {
          if (output_topic.value.empty())
          {
            // If the client did not ask the data to be remapped, then return the default topic
            diagnostic_msgs::KeyValue out_tpc;
            out_tpc.key = output_topic.key;
            out_tpc.value = alloc_comp_info.getOutputTopic(output_topic.key);
            res.output_topics.push_back(out_tpc);
          }
          else
          {
            // Set up the remapper
            temoto_er_manager::LoadExtResource load_er_msg_remapper;
            load_er_msg_remapper.request.action = temoto_er_manager::action::ROS_EXECUTE;
            load_er_msg_remapper.request.package_name = "topic_tools";
            load_er_msg_remapper.request.executable = "relay";
            load_er_msg_remapper.request.args = alloc_comp_info.getOutputTopic(output_topic.key) + " " + output_topic.value;

            resource_manager_1_.call<temoto_er_manager::LoadExtResource>( temoto_er_manager::srv_name::MANAGER
                                                      , temoto_er_manager::srv_name::SERVER
                                                      , load_er_msg_remapper
                                                      , rmp::FailureBehavior::NONE);

            res.output_topics.push_back(output_topic);
          }
        }

        res.package_name = alloc_comp_info.getPackageName();
        res.executable = alloc_comp_info.getExecutable();
        //res.rmp = load_er_msg.response.rmp;

        return;
      }
      catch(error::ErrorStack& error_stack)
      {
        // TODO: Currently component reuse is enforced
        throw FORWARD_ERROR(error_stack);
      }
    }

    /*
     * Loop through suitable local component candidates
     */ 
    for (ComponentInfo& ci : l_cis)
    {
      // Try to run the component via local Resource Manager
      temoto_er_manager::LoadExtResource load_er_msg;
      load_er_msg.request.action = temoto_er_manager::action::ROS_EXECUTE;
      load_er_msg.request.package_name = ci.getPackageName();
      load_er_msg.request.executable = ci.getExecutable();

      // Remap the input topics if requested
      processTopics(req.input_topics, res.input_topics, load_er_msg, ci, "in");

      // Remap the output topics if requested
      processTopics(req.output_topics, res.output_topics, load_er_msg, ci, "out");
            
      TEMOTO_DEBUG( "Found a suitable local component: '%s', '%s', '%s', reliability %.3f"
                 , load_er_msg.request.action.c_str()
                 , load_er_msg.request.package_name.c_str()
                 , load_er_msg.request.executable.c_str()
                 , ci.getReliability());

      try
      {
        resource_manager_1_.call<temoto_er_manager::LoadExtResource>( temoto_er_manager::srv_name::MANAGER
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
        allocated_ext_resources_.emplace(res.rmp.resource_id, load_er_msg);

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

  /*
   * Go through suitable remote component candidates
   */ 
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
        resource_manager_1_.call<LoadComponent>( srv_name::MANAGER
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

/*
 * ComponentManagerServers::unloadComponentCb
 */
void ComponentManagerServers::unloadComponentCb( LoadComponent::Request& req
                                               , LoadComponent::Response& res)
{
  TEMOTO_DEBUG("received a request to stop component with id '%ld'", res.rmp.resource_id);
  allocated_components_.erase(res.rmp.resource_id);
  allocated_ext_resources_.erase(res.rmp.resource_id);
  return;
}

/*
 * ComponentManagerServers::loadPipeCb
 */ 
void ComponentManagerServers::loadPipeCb(LoadPipe::Request& req, LoadPipe::Response& res)
{
  TEMOTO_INFO_STREAM("Received a request: \n" << req << std::endl);

  PipeInfos pipes;

  if (!cir_->findPipes(req, pipes))
  {
    throw CREATE_ERROR(temoto_core::error::Code::NO_TRACKERS_FOUND, "No pipes found for the requested category");
  }

  TEMOTO_DEBUG_STREAM("Found the requested pipe category.");

  /*
   * Loop over all possible tracking methods until somethin starts to work
   */
  for (PipeInfo& pipe : pipes)
  {
    TEMOTO_DEBUG_STREAM("Trying pipe: \n" << pipe.toString().c_str());

    try
    {
      // Get the id of the pipe, if provided
      std::string pipe_id = req.pipe_id;

      // Create a new pipe id if it was not specified
      if (pipe_id.empty())
      {
        // Create a unique pipe identifier string
        pipe_id = "pipe_" + std::to_string(pipe_id_generator_.generateID())
                + "_at_" + temoto_core::common::getTemotoNamespace();
      }

      /*
       * Build the pipe based on the number of segments. If the pipe
       * contains only one segment, then there are no constraints on
       * the ouptut topic types. But if the pipe contains multiple segments
       * then each preceding segment has to provide the topics that are
       * required by the proceding segment
       */
      temoto_core::TopicContainer required_topics;

      if (pipe.getPipeSize() > 1)
      {
        // TODO: If the right hand side of the "req_tops" is directly used in the proceeding
        // for-loop, then it crashes on the second loop. Not sure why.
        const std::set<std::string>& req_tops = pipe.getSegments().at(1).required_input_topic_types_;

        // Loop over requested topics
        for (const auto& topic : req_tops)
        {
          required_topics.addOutputTopicType(topic);
        }
      }
      else
      {
        // TODO: If the right hand side of the "req_tops" is directly used in the proceeding
        // for-loop, then it crashes on the second loop. Not sure why.
        const std::set<std::string>& req_tops = pipe.getSegments().at(0).required_output_topic_types_;

        // Loop over requested topics
        for (const auto& topic : req_tops)
        {
          required_topics.addOutputTopicType(topic);
        }
      }

      // Loop over the pipe
      const std::vector<Segment>& segments = pipe.getSegments();

      // TODO: REMOVE AFTER RMP HAS THIS FUNCTIONALITY
      std::vector<int> sub_resource_ids;

      for (unsigned int i=0; i<segments.size(); i++)
      {
        // Clear out the required output topics
        required_topics.clearOutputTopics();

        // If it is not the last segment then ...
        if (i != segments.size()-1)
        {
          // ... get the requirements for the output topic types from the proceding segment
          for (const auto& topic_type : segments.at(i+1).required_input_topic_types_)
          {
            required_topics.addOutputTopic(topic_type, "/" + pipe_id + "/segment_" + std::to_string(i) + "/" + topic_type);
          }
        }
        else
        {
          // ... get the requirements for the output topics from own output topic requirements
          // TODO: throw if the "required_output_topic_types_" is empty
          for (const auto& topic_type : segments.at(i).required_output_topic_types_)
          {
            required_topics.addOutputTopic(topic_type, "/" + pipe_id + "/segment_" + std::to_string(i) + "/" + topic_type);
          }
        }

        // Compose the LoadComponent message
        temoto_component_manager::LoadComponent load_component_msg;
        load_component_msg.request.component_type = segments.at(i).segment_type_;
        load_component_msg.request.input_topics = required_topics.inputTopicsAsKeyValues();
        load_component_msg.request.output_topics = required_topics.outputTopicsAsKeyValues();

        // Call the Component Manager
        resource_manager_2_.call<temoto_component_manager::LoadComponent>(temoto_component_manager::srv_name::MANAGER,
                                                                          temoto_component_manager::srv_name::SERVER,
                                                                          load_component_msg);

        // TODO: REMOVE AFTER RMP HAS THIS FUNCTIONALITY
        sub_resource_ids.push_back(load_component_msg.response.rmp.resource_id);

        required_topics.setInputTopicsByKeyValue(load_component_msg.response.output_topics);
      }

      // Send the output topics of the last segment back via response
      res.output_topics = required_topics.outputTopicsAsKeyValues();

      // Add the pipe to allocated pipes + increase its reliability
      pipe.reliability_.adjustReliability();
      cir_->updatePipe(pipe);

      //allocated_pipes_[res.rmp.resource_id] = pipe;
      allocated_pipes_hack_[res.rmp.resource_id] = std::pair<PipeInfo, std::vector<int>>(pipe, sub_resource_ids);

      return;
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      // TODO: Make sure that send error add the name of the function where the error was sent
      SEND_ERROR(error_stack);
    }
  }

  throw CREATE_ERROR(temoto_core::error::Code::NO_TRACKERS_FOUND, "Could not find pipes for the requested category");
}

/*
 * ComponentManagerServers::unloadPipeCb
 */
void ComponentManagerServers::unloadPipeCb(LoadPipe::Request& req, LoadPipe::Response& res)
{
  // Remove the pipe from the list of allocated pipes
  auto it = allocated_pipes_hack_.find(res.rmp.resource_id);
  if (it != allocated_pipes_hack_.end())
  {
    TEMOTO_DEBUG_STREAM("Erasing a pipe from the list of allocated pipes");
    allocated_pipes_hack_.erase(it);
  }
  else
  {
    throw CREATE_ERROR(temoto_core::error::Code::RESOURCE_NOT_FOUND, "Could not unload the pipe, because"
                       " it does not exist in the list of allocated pipes");
  }
}


/*
 * ComponentManagerServers::processTopics
 */
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

temoto_core::temoto_id::ID ComponentManagerServers::checkIfInUse( const std::vector<ComponentInfo>& cis_to_check) const
{
  for (const ComponentInfo& ci : cis_to_check)
  {
    for (auto const& ac : allocated_components_)
    {
      if (ac.second.getPackageName() != ci.getPackageName())
      {
        continue;
      }

      if (ac.second.getExecutable() != ci.getExecutable())
      {
        continue;
      }

      return ac.first;
    }
  }
  return temoto_core::temoto_id::UNASSIGNED_ID;
}

}  // component_manager namespace
