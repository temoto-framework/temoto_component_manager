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

#include <algorithm>
#include <utility>
#include <fstream>
#include <regex>
#include <boost/filesystem.hpp>

#include "ros/package.h"
#include "yaml-cpp/yaml.h"
#include "temoto_component_manager/component_manager_servers.h"
#include "temoto_resource_registrar/temoto_logging.h"
#include "temoto_resource_registrar/temoto_error.h"

namespace temoto_component_manager
{
using namespace temoto_core;

ComponentManagerServers::ComponentManagerServers(BaseSubsystem *b, ComponentInfoRegistry *cir)
: BaseSubsystem(*b, __func__)
, cir_(cir)
, resource_registrar_(srv_name::MANAGER)
{ START_SPAN
  /*
   * Configure the RR catalog backup routine
   */
  std::string home_path = std::getenv("HOME");
  std::string rr_catalog_backup_path = home_path + "/.temoto/" + srv_name::MANAGER + ".rrcat";
  rr_catalog_config_.setName(srv_name::MANAGER);
  rr_catalog_config_.setLocation(rr_catalog_backup_path);
  rr_catalog_config_.setSaveOnModify(true);
  rr_catalog_config_.setEraseOnDestruct(true);
  resource_registrar_.updateConfiguration(rr_catalog_config_);

  /*
   * Add the LoadComponent server to the resource registrar
   */
  {
    auto server = std::make_unique<Ros1Server<LoadComponent>>(srv_name::SERVER
    , std::bind(&ComponentManagerServers::loadComponentCb, this, std::placeholders::_1, std::placeholders::_2)
    , std::bind(&ComponentManagerServers::unloadComponentCb, this, std::placeholders::_1, std::placeholders::_2)
    , std::bind(&ComponentManagerServers::componentStatusCb
      , this
      , std::placeholders::_1
      , std::placeholders::_2
      , std::placeholders::_3));
    resource_registrar_.registerServer(std::move(server));
  }

  /*
   * Add the LoadPipe server to the resource registrar
   */
  {
    auto server = std::make_unique<Ros1Server<LoadPipe>>(srv_name::PIPE_SERVER
    , std::bind(&ComponentManagerServers::loadPipeCb, this, std::placeholders::_1, std::placeholders::_2)
    , std::bind(&ComponentManagerServers::unloadPipeCb, this, std::placeholders::_1, std::placeholders::_2));
    resource_registrar_.registerServer(std::move(server));
  }

  resource_registrar_.init();

  /*
   * Check if this node should be recovered from a previous system failure
   */
  if (boost::filesystem::exists(rr_catalog_backup_path))
  {
    // Give the component snooper agents some time to find local components
    ros::Duration(2).sleep();
    restoreState();
  }

  /*
   * Set up simple ROS servers that do not provide any resources
   */ 
  list_components_server_ = nh_.advertiseService( srv_name::LIST_COMPONENTS_SERVER
  , &ComponentManagerServers::listComponentsCb
  , this);

  list_pipes_server_ = nh_.advertiseService( srv_name::LIST_PIPES_SERVER
  , &ComponentManagerServers::listPipesCb
  , this);

  // Register the component update callback
  cir_->registerUpdateCallback(std::bind(&ComponentManagerServers::cirUpdateCallback, this, std::placeholders::_1));                                       

  TEMOTO_INFO_("Component manager is ready.");
}

ComponentManagerServers::~ComponentManagerServers()
{
}

void ComponentManagerServers::componentStatusCb(LoadComponent::Request& req
, LoadComponent::Response& res
, temoto_resource_registrar::Status status_msg)
{
  /* TODO */
}

/*
 * ComponentManagerServers::erStatusCb
 */
void ComponentManagerServers::erStatusCb(temoto_process_manager::LoadProcess srv_msg
, temoto_resource_registrar::Status status_msg)
{
  TEMOTO_WARN_STREAM_("Received a status message: " << status_msg.message_);

  // If local component failed, adjust package reliability and advertise to other managers via
  // synchronizer.
  if (status_msg.state_ == temoto_resource_registrar::Status::State::FATAL)
  {
    std::lock_guard<std::recursive_mutex> guard_acm(allocated_components_mutex_);
    auto it = std::find_if(allocated_components_.begin()
    , allocated_components_.end()
    , [&srv_msg](const AllocCompTuple& act)
      {
        return std::get<2>(act).response.temoto_metadata.request_id == srv_msg.response.temoto_metadata.request_id;
      });

    if (it == allocated_components_.end())
    {
      return;
    }

    if (std::get<1>(*it).isLocal())
    {
      std::get<1>(*it).adjustReliability(0.0);
      cir_->updateLocalComponent(std::get<1>(*it));
      TEMOTO_DEBUG_STREAM_("Updated reliability to: " << std::get<1>(*it).getReliability());
    }
    else
    {
      TEMOTO_WARN_("Remote component failure detected, doing nothing (component will be updated via synchronizer).");
    }
  }
}

/*
 * ComponentManagerServers::pipeStatusCb
 */
void ComponentManagerServers::pipeStatusCb(LoadComponent srv_msg, temoto_resource_registrar::Status status_msg)
{
  TEMOTO_DEBUG_("Received a status message.");

  // If local sensor failed, adjust package reliability and advertise to other managers via
  // synchronizer.
  if (status_msg.state_ == temoto_resource_registrar::Status::State::FATAL)
  {
    TEMOTO_DEBUG_("A resource, that a running pipe depends on, has failed");

    auto it = std::find_if(allocated_pipes_.begin(), allocated_pipes_.end(),
    [&srv_msg](const AllocPipePair& pair_in)
    {
      for (const auto& alloc_comp : pair_in.second.second)
      {
        if (alloc_comp.response.temoto_metadata.request_id == srv_msg.response.temoto_metadata.request_id)
        {
          return true;
        }
      }
      return false;
    });

    if (it != allocated_pipes_.end())
    {
      TEMOTO_INFO_("Pipe of type '%s' (pipe size: %d) has stopped working"
      , it->second.first.getType().c_str()
      , it->second.first.getPipeSize());

      // Reduce the reliability of the pipe
      it->second.first.reliability_.adjustReliability(0);
      cir_->updatePipe(it->second.first);
    }
  }
  else if (status_msg.state_ == temoto_resource_registrar::Status::State::UPDATE)
  {
    /*
     * Everything is handled by Resource Registrar
     */ 
  }
}

/*
 * ComponentManagerServers::listComponentsCb
 */
bool ComponentManagerServers::listComponentsCb( ListComponents::Request& req, ListComponents::Response& res)
{
  // Find the devices with the required type
  for (const auto& component : cir_->getLocalComponents())
  {
    if (component.getType() == req.type || req.type.empty())
    {
      temoto_component_manager::Component comp_msg;
      comp_msg.component_name = component.getName();
      comp_msg.component_type = component.getType();
      comp_msg.package_name = component.getPackageName();
      comp_msg.temoto_namespace = component.getTemotoNamespace();
      comp_msg.executable = component.getExecutable();
      comp_msg.input_topics = component.getInputTopicsAsKeyVal();
      comp_msg.output_topics = component.getOutputTopicsAsKeyVal();
      comp_msg.required_parameters = component.getRequiredParametersAsKeyVal();
    
      res.local_components.push_back(comp_msg);
    }
  }

  for (const auto& component : cir_->getRemoteComponents())
  {
    if (component.getType() == req.type || req.type.empty())
    {
      temoto_component_manager::Component comp_msg;
      comp_msg.component_name = component.getName();
      comp_msg.component_type = component.getType();
      comp_msg.package_name = component.getPackageName();
      comp_msg.temoto_namespace = component.getTemotoNamespace();
      comp_msg.executable = component.getExecutable();
      comp_msg.input_topics = component.getInputTopicsAsKeyVal();
      comp_msg.output_topics = component.getOutputTopicsAsKeyVal();
      comp_msg.required_parameters = component.getRequiredParametersAsKeyVal();
    
      res.remote_components.push_back(comp_msg);
    }
  }
  return true;
}

/*
 * ComponentManagerServers::listPipesCb
 */
bool ComponentManagerServers::listPipesCb( ListPipes::Request& req, ListPipes::Response& res)
{
  // TODO: Find the pipes with the required type
  for (const auto& pipe_category : cir_->getPipes())
  {
    for (const auto& pipe : pipe_category.second)
    {
      Pipe pipe_msg;
      pipe_msg.pipe_type = pipe.getType();
      pipe_msg.pipe_name = pipe.getName();

      for (const auto& segment : pipe.getSegments())
      {
        PipeSegment segment_msg;
        segment_msg.segment_type = segment.segment_type_;
        segment_msg.required_parameters = std::vector<std::string>(
          segment.required_parameters_.begin(), segment.required_parameters_.end());
        pipe_msg.segments.push_back(segment_msg);
      }
      res.pipe_infos.push_back(pipe_msg);
    }
  }
  return true;
}

/*
 * ComponentManagerServers::loadComponentCb
 */
void ComponentManagerServers::loadComponentCb( LoadComponent::Request& req, LoadComponent::Response& res)
{ START_SPAN
  TEMOTO_INFO_STREAM_("Received a request to load a component: \n" << req << std::endl);

  /*
    There are two significantly different conditions how this callback is invoked by RR server:
    a) The query is completely unique, no such component is running yet.
    b) The query asks for a component that's already running, but wants the data to be published on custom topics.

    Condition a) is handled in a regular way, that is, the component is located and loaded via Process Manager.
    Condition b) is a bit more special. The running component is found and Process Manager is invoked 2 times:
      1'st call to Process Manager sets up a topic remapping node
      2'nd call to Process Manager asks to "start" the already running component in order to increase its use count.
   */

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
   && req.use_only_local_components == false
  /*&& (req.trr.temoto_namespace == common::getTemotoNamespace()) // TODO: include temoto namespace to RR2*/)
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
    TEMOTO_DEBUG_STREAM_("Found a suitable local candidate.");
    auto allocated_component = checkIfInUse(req);

    /*
     * The component is already in use but the client wants certain topics to be remapped (condition "b").
     */ 
    if (allocated_component)
    {
      TEMOTO_DEBUG_STREAM_("The given component is already in use but it is providing other data than requested."
       "Setting up a topic relay ...");

      const auto& allocated_component_res = std::get<0>(*allocated_component).response;
      temoto_core::TopicContainer alloc_comp_res_tpc_container;
      alloc_comp_res_tpc_container.setInputTopicsByKeyValue(allocated_component_res.input_topics);
      alloc_comp_res_tpc_container.setOutputTopicsByKeyValue(allocated_component_res.output_topics);

      try
      {
        resource_registrar_.call<temoto_process_manager::LoadProcess>(temoto_process_manager::srv_name::MANAGER
        , temoto_process_manager::srv_name::SERVER
        , std::get<2>(*allocated_component)
        , std::bind(&ComponentManagerServers::erStatusCb, this, std::placeholders::_1, std::placeholders::_2));

        res.package_name = std::get<0>(*allocated_component).response.package_name;
        res.executable = std::get<0>(*allocated_component).response.executable;
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
            in_tpc.value = alloc_comp_res_tpc_container.getInputTopic(input_topic.key);
            res.input_topics.push_back(in_tpc);
          }
          else
          {
            // Set up the remapper
            temoto_process_manager::LoadProcess load_er_msg_remapper;
            load_er_msg_remapper.request.action = temoto_process_manager::action::ROS_EXECUTE;
            load_er_msg_remapper.request.package_name = "topic_tools";
            load_er_msg_remapper.request.executable = "relay";
            load_er_msg_remapper.request.args = alloc_comp_res_tpc_container.getInputTopic(input_topic.key) + " " + input_topic.value;
            
            resource_registrar_.call<temoto_process_manager::LoadProcess>(temoto_process_manager::srv_name::MANAGER
            , temoto_process_manager::srv_name::SERVER
            , load_er_msg_remapper
            , std::bind(&ComponentManagerServers::erStatusCb, this, std::placeholders::_1, std::placeholders::_2));

            res.input_topics.push_back(input_topic);
          }
        }

        // Output topics
        if (req.output_topics.empty())
        {
          res.output_topics = allocated_component_res.output_topics;
          return;
        }

        for (const auto& output_topic : req.output_topics)
        {
          if (output_topic.value.empty())
          {
            // If the client did not ask the data to be remapped, then return the default topic
            diagnostic_msgs::KeyValue out_tpc;
            out_tpc.key = output_topic.key;
            out_tpc.value = alloc_comp_res_tpc_container.getOutputTopic(output_topic.key);
            res.output_topics.push_back(out_tpc);
          }
          else
          {
            // Set up the remapper
            temoto_process_manager::LoadProcess load_er_msg_remapper;
            load_er_msg_remapper.request.action = temoto_process_manager::action::ROS_EXECUTE;
            load_er_msg_remapper.request.package_name = "topic_tools";
            load_er_msg_remapper.request.executable = "relay";
            load_er_msg_remapper.request.args = alloc_comp_res_tpc_container.getOutputTopic(output_topic.key) + " " + output_topic.value;
            TEMOTO_DEBUG_STREAM_("key: " << output_topic.key << ". args: " << load_er_msg_remapper.request.args);

            resource_registrar_.call<temoto_process_manager::LoadProcess>(temoto_process_manager::srv_name::MANAGER
            , temoto_process_manager::srv_name::SERVER
            , load_er_msg_remapper
            , std::bind(&ComponentManagerServers::erStatusCb, this, std::placeholders::_1, std::placeholders::_2));

            res.output_topics.push_back(output_topic);
          }
        }
        return;
      }
      catch(resource_registrar::TemotoErrorStack& error_stack)
      {
        // TODO: Currently component reuse is enforced
        throw FWD_TEMOTO_ERRSTACK(error_stack);
      }
    }

    /*
     * The component is unique and not in use (condition "a").
     * Loop through suitable local component candidates
     */ 
    for (ComponentInfo& ci : l_cis)
    {
      // Try to run the component via local Resource Manager
      temoto_process_manager::LoadProcess load_er_msg;
      load_er_msg.request.action = temoto_process_manager::action::ROS_EXECUTE;
      load_er_msg.request.package_name = ci.getPackageName();
      load_er_msg.request.executable = ci.getExecutable();

      // Remap the input topics if requested
      processTopics(req.input_topics, res.input_topics, load_er_msg, ci, "in");

      // Remap the output topics if requested
      processTopics(req.output_topics, res.output_topics, load_er_msg, ci, "out");

      // Remap the parameters if requested
      processParameters(req.required_parameters, res.required_parameters, load_er_msg, ci);

      // Set additional arguments
      load_er_msg.request.args = load_er_msg.request.args + " " + req.additional_args; 
            
      TEMOTO_DEBUG_( "Found a suitable local component: '%s', '%s', '%s', reliability %.3f"
      , load_er_msg.request.action.c_str()
      , load_er_msg.request.package_name.c_str()
      , load_er_msg.request.executable.c_str()
      , ci.getReliability());

      try
      {
        resource_registrar_.call<temoto_process_manager::LoadProcess>(temoto_process_manager::srv_name::MANAGER
        , temoto_process_manager::srv_name::SERVER
        , load_er_msg
        , std::bind(&ComponentManagerServers::erStatusCb, this, std::placeholders::_1, std::placeholders::_2));

        TEMOTO_DEBUG_("Call to  Manager was sucessful.");

        // Fill out the response about which particular component was chosen
        res.component_name = ci.getName();
        res.package_name = ci.getPackageName();
        res.executable = ci.getExecutable();

        ci.adjustReliability(1.0);
        cir_->updateLocalComponent(ci);

        std::lock_guard<std::recursive_mutex> guard_acm(allocated_components_mutex_);

        LoadComponent load_component_srv_msg;
        load_component_srv_msg.request = req;
        load_component_srv_msg.response = res;
        allocated_components_.push_back({load_component_srv_msg, ci, load_er_msg});

        return;
      }
      catch(resource_registrar::TemotoErrorStack& error_stack)
      {
        if (error_stack.front().getOrigin() == GET_NAME)
        {
          ci.adjustReliability(0.0);
          cir_->updateLocalComponent(ci);
        }
        //SEND_ERROR(error_stack);
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
      load_component_msg.request.use_only_local_components = true;
      load_component_msg.request.component_type = ci.getType();
      load_component_msg.request.package_name = ci.getPackageName();
      load_component_msg.request.executable = ci.getExecutable();
      load_component_msg.request.input_topics = req.input_topics;
      load_component_msg.request.output_topics = req.output_topics;
      load_component_msg.request.required_parameters = req.required_parameters;

      TEMOTO_INFO_( "Component Manager is forwarding request: '%s', '%s', '%s', reliability %.3f"
      , ci.getType().c_str()
      , ci.getPackageName().c_str()
      , ci.getExecutable().c_str()
      , ci.getReliability());

      try
      {
        resource_registrar_.call<LoadComponent>("/" + ci.getTemotoNamespace() + "/" + srv_name::MANAGER
        , srv_name::SERVER
        , load_component_msg);

        TEMOTO_DEBUG_("Call to remote ComponentManagerServers was sucessful.");
        
        // Assign the current metadata (res) to the remote response
        load_component_msg.response.temoto_metadata = res.temoto_metadata;
        res = load_component_msg.response;

        std::lock_guard<std::recursive_mutex> guard_acm(allocated_components_mutex_);
        allocated_components_.push_back({load_component_msg, ci, temoto_process_manager::LoadProcess()});
      }
      catch(resource_registrar::TemotoErrorStack& error_stack)
      {
        throw FWD_TEMOTO_ERRSTACK(error_stack);
      }
      return;
    }
  }
  else
  {
    // no suitable local nor remote component was found
    throw TEMOTO_ERRSTACK("ComponentManagerServers did not find a suitable component");
  }
}

/*
 * ComponentManagerServers::unloadComponentCb
 */
void ComponentManagerServers::unloadComponentCb(LoadComponent::Request& req, LoadComponent::Response& res)
{
  // Suppress "unused parameter" compiler warnings
  (void)req;
  (void)res;
  std::lock_guard<std::recursive_mutex> guard_acm(allocated_components_mutex_);

  TEMOTO_WARN_STREAM_("received a request to stop component '" << res <<"'");
  auto it = std::find_if(allocated_components_.begin()
  , allocated_components_.end()
  , [&](const AllocCompTuple& act)
    {
      TEMOTO_WARN_STREAM_("loaded comp id: " << std::get<0>(act).response.temoto_metadata.request_id << "\n" << "reqcomp id: " << res.temoto_metadata.request_id);
      return std::get<0>(act).response.temoto_metadata.request_id == res.temoto_metadata.request_id;
    });
  
  if (it != allocated_components_.end())
  {
    allocated_components_.erase(it);
  }
  else
  {
    TEMOTO_ERROR_STREAM_("Such component has not been loaded");
  }
}

/*
 * ComponentManagerServers::loadPipeCb
 */ 
void ComponentManagerServers::loadPipeCb(LoadPipe::Request& req, LoadPipe::Response& res)
{
  TEMOTO_DEBUG_STREAM_("Received a request: \n" << req << std::endl);
  PipeInfos pipes;

  if (!cir_->findPipes(req, pipes))
  {
    throw TEMOTO_ERRSTACK("No pipes found for the requested category");
  }

  TEMOTO_DEBUG_STREAM_("Found the requested pipe category.");

  /*
   * Loop over all possible tracking methods until somethin starts to work
   */
  for (PipeInfo& pipe : pipes)
  {
    TEMOTO_DEBUG_STREAM_("Trying pipe: \n" << pipe.toString().c_str());

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

      res.pipe_id = pipe_id;

      /*
       * Build the pipe based on the number of segments. If the pipe
       * contains only one segment, then there are no constraints on
       * the ouptut topic types. But if the pipe contains multiple segments
       * then each preceding segment has to provide the topics that are
       * required by the proceding segment
       */
      temoto_core::TopicContainer previous_segment_topics;

      // Loop over the pipe
      const std::vector<Segment>& segments = pipe.getSegments();

      // TODO: REMOVE AFTER RMP HAS THIS FUNCTIONALITY
      std::vector<LoadComponent> component_srv_msgs;

      for (unsigned int i=0; i<segments.size(); i++)
      {
        // Declare a LoadComponent message
        temoto_component_manager::LoadComponent load_component_msg;
        load_component_msg.request.use_only_local_components = req.use_only_local_segments;
        load_component_msg.request.component_type = segments.at(i).segment_type_;
        temoto_core::TopicContainer required_topics;

        // Set the input topics
        for (const auto& topic_type : segments.at(i).required_input_topic_types_)
        {
          required_topics.addInputTopic(topic_type, previous_segment_topics.getOutputTopic(topic_type)); 
        }

        // Set the output topics. If it is not the last segment then ...
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

        load_component_msg.request.input_topics = required_topics.inputTopicsAsKeyValues();
        load_component_msg.request.output_topics = required_topics.outputTopicsAsKeyValues();

        // Check if any parameters were specified for this segment
        for (const auto& seg_param_spec : req.pipe_segment_specifiers)
        {
          if (seg_param_spec.segment_index == i)
          {
            load_component_msg.request.required_parameters = seg_param_spec.parameters;
            load_component_msg.request.component_name = seg_param_spec.component_name;
            break;
          }
        }

        // Call the Component Manager
        resource_registrar_.call<LoadComponent>(temoto_component_manager::srv_name::MANAGER
        , temoto_component_manager::srv_name::SERVER
        , load_component_msg
        , std::bind(&ComponentManagerServers::pipeStatusCb, this, std::placeholders::_1, std::placeholders::_2));

        // TODO: REMOVE AFTER RMP HAS THIS FUNCTIONALITY
        component_srv_msgs.push_back(load_component_msg);

        previous_segment_topics.setInputTopicsByKeyValue(load_component_msg.response.input_topics);
        previous_segment_topics.setOutputTopicsByKeyValue(load_component_msg.response.output_topics);
      }

      // Send the output topics of the last segment back via response
      res.output_topics = previous_segment_topics.outputTopicsAsKeyValues();

      // Add the pipe to allocated pipes + increase its reliability
      pipe.reliability_.adjustReliability();
      cir_->updatePipe(pipe);

      LoadPipe load_pipe_srv_msg;
      load_pipe_srv_msg.request = req;
      load_pipe_srv_msg.response = res;
      allocated_pipes_.push_back({load_pipe_srv_msg, {pipe, component_srv_msgs}});

      return;
    }
    catch (resource_registrar::TemotoErrorStack& error_stack)
    {
      // TODO: Make sure that send error add the name of the function where the error was sent
      //SEND_ERROR(error_stack);
    }
  }

  throw TEMOTO_ERRSTACK("Could not find pipes for the requested category");
}

/*
 * ComponentManagerServers::unloadPipeCb
 */
void ComponentManagerServers::unloadPipeCb(LoadPipe::Request& req, LoadPipe::Response& res)
{
  (void)req; // Suppress "unused parameter" compiler warnings

  // Remove the pipe from the list of allocated pipes
  auto it = std::find_if(allocated_pipes_.begin()
  , allocated_pipes_.end()
  , [&res](const AllocPipePair& pair_in)
    {
      return pair_in.first.response.temoto_metadata.request_id == res.temoto_metadata.request_id;
    });

  if (it != allocated_pipes_.end())
  {
    TEMOTO_DEBUG_STREAM_("Erasing a pipe from the list of allocated pipes");
    allocated_pipes_.erase(it);
  }
  else
  {
    throw TEMOTO_ERRSTACK("Could not unload the pipe, because it does not exist in the list of allocated pipes");
  }
}

/*
 * ComponentManagerServers::processTopics
 */
void ComponentManagerServers::processTopics(std::vector<diagnostic_msgs::KeyValue>& req_topics
, std::vector<diagnostic_msgs::KeyValue>& res_topics
, temoto_process_manager::LoadProcess& load_er_msg
, ComponentInfo& component_info
, std::string direction)
{ START_SPAN
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

  // First fill out the response message with the default topic names
  for (const auto& topic : component_info_topics)
  {
    diagnostic_msgs::KeyValue topic_msg;
    topic_msg.key = topic.first;
    topic_msg.value = common::getAbsolutePath(topic.second);
    res_topics.push_back(topic_msg);
  }

  // If no topics were requested, then return a list of all topics this component publishes
  if (req_topics.empty())
  {
    return; 
  }

  // Remap the input topics if requested
  for (auto& req_topic : req_topics)
  {
    // Find the topic from the filled out response message
    auto res_topic = std::find_if(res_topics.begin(), res_topics.end(),
      [req_topic](const auto rt)
      {
        if (req_topic.key == rt.key)
        {
          return true;
        }
        else
        {
          return false;
        }
      });

    if (!req_topic.value.empty())
    {
      std::string default_topic = res_topic->value;
      res_topic->value = common::getAbsolutePath(req_topic.value);
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
  }
}

/*
 * ComponentManagerServers::processParameters
 */
void ComponentManagerServers::processParameters( std::vector<diagnostic_msgs::KeyValue>& req_parameters
                                               , std::vector<diagnostic_msgs::KeyValue>& res_parameters
                                               , temoto_process_manager::LoadProcess& load_er_msg
                                               , ComponentInfo& component_info)
{ START_SPAN
  /*
   * Find out it this is a launch file or not. Remapping is different
   * for executable types (launch files or executables)
   */
  bool isLaunchFile;
  std::regex rx(".*\\.launch$");
  isLaunchFile = std::regex_match(component_info.getExecutable(), rx);
  std::vector<StringPair> component_info_parameters = component_info.getRequiredParameters();

  // If no parameters were requested, then set the default value for the parameter
  if (req_parameters.empty())
  {
    for (const auto& parameter : component_info_parameters)
    {
      diagnostic_msgs::KeyValue parameter_msg;
      parameter_msg.key = parameter.first;
      parameter_msg.value = parameter.second;
      res_parameters.push_back(parameter_msg);

      std::string remap_arg = parameter_msg.key + ":=" + parameter_msg.value;
      load_er_msg.request.args += remap_arg + " ";
    }
    return;
  }

  // Fill out the parameters
  for (auto& req_parameter : req_parameters)
  {
    // And return the input parameters via response
    diagnostic_msgs::KeyValue res_parameter;
    res_parameter.key = req_parameter.key;
    std::string default_parameter_value = component_info.getRequiredParameter(req_parameter.key);

    if (req_parameter.value != "")
    {
      std::string remap_arg = req_parameter.key + ":=" + req_parameter.value;
      load_er_msg.request.args += remap_arg + " ";
      res_parameter.value = req_parameter.value;
    }
    else
    {
      res_parameter.value = default_parameter_value;
    }

    // Add the parameter to the response message
    res_parameters.push_back(res_parameter);
  }
}

boost::optional<AllocCompTuple> ComponentManagerServers::checkIfInUse(const LoadComponent::Request& req) const
{
  std::lock_guard<std::recursive_mutex> guard_acm(allocated_components_mutex_);
  auto comp_er_pair_it = std::find_if(allocated_components_.begin()
  , allocated_components_.end()
  , [req](const AllocCompTuple& ac_tuple)
    {
      const auto& acp_req = std::get<0>(ac_tuple).request;
      const auto& acp_res = std::get<0>(ac_tuple).response;

      return (req.component_type == acp_req.component_type
      && (req.executable == acp_req.executable || req.executable == acp_res.executable)
      && (req.package_name == acp_req.package_name || req.package_name == acp_res.package_name));
    });
  
  if (comp_er_pair_it != allocated_components_.end())
  {
    return boost::optional<AllocCompTuple>(*comp_er_pair_it);
  }
  else
  {
    return boost::optional<AllocCompTuple>();
  }
}

void ComponentManagerServers::cirUpdateCallback(ComponentInfo component)
{
  // TEMOTO_DEBUG_STREAM_("A component was added or updated ...");

  // // make a copy of the allocated_components so that it will not be corrupted by other threads
  // std::map<temoto_core::temoto_id::ID, ComponentInfoResponse> allocated_components_cpy;
  // {
  //   std::lock_guard<std::recursive_mutex> guard_acm(allocated_components_mutex_);
  //   allocated_components_cpy = allocated_components_;
  // }

  // for (const auto allocated_component : allocated_components_cpy)
  // {
  //   if (component.getType() != allocated_component.second.first.getType())
  //   {
  //     continue;
  //   }
  //   if (component.getReliability() <= allocated_component.second.first.getReliability())
  //   {
  //     continue;
  //   }

  //   temoto_core::ResourceStatus status_message;
  //   status_message.request.resource_id = allocated_component.first;
  //   status_message.request.status_code = trr::status_codes::UPDATE;
  //   status_message.request.message = "A component that is currently loaded got a more reliable alternative component";
  //   resource_registrar_1_.sendStatus(status_message);
  // }
  TEMOTO_DEBUG_STREAM_("CIR update routine finished.");
}

void ComponentManagerServers::restoreState()
{
  TEMOTO_INFO_STREAM_("Restoring the RR catalog");
  resource_registrar_.loadCatalog();

  /*
   * Restore "Load Component" related datastructures
   */
  for (const auto& component_query : resource_registrar_.getServerQueries<LoadComponent>(srv_name::SERVER))
  {
    // Get the component info datastructure that corresponds to the loaded component
    ComponentInfo ci;
    if (cir_->findLocalComponent(component_query.response.component_name, ci))
    {
      TEMOTO_DEBUG_STREAM_("found the local component");
      auto erm_queries = resource_registrar_.getRosChildQueries<temoto_process_manager::LoadProcess>(component_query.response.temoto_metadata.request_id
      , temoto_process_manager::srv_name::SERVER);
    
      // If the component is local, then it also has an Process Manager query, otherwise it's a remote component
      TEMOTO_DEBUG_STREAM_("size of erm_queries: " << erm_queries.size());
      if (erm_queries.empty())
      {
        allocated_components_.push_back({component_query, ci, temoto_process_manager::LoadProcess()});
      }
      else
      {
        for (const auto& erm_query : erm_queries)
        {
          TEMOTO_DEBUG_STREAM("Process Manager query: " << erm_query.second.request);
          if (erm_query.second.request.package_name == "topic_tools" &&
              erm_query.second.request.executable == "relay")
          {
            continue;
          }
          else
          {
            allocated_components_.push_back({component_query, ci, erm_query.second});
          }
        }
      }
    }
    else
    {
      TEMOTO_ERROR_STREAM_("could not find local component");
    }
  }

  /*
   * TODO: Restore "Load Pipe" related datastructures
   */
}

}  // component_manager namespace
