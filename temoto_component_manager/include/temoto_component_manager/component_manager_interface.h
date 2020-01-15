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

#ifndef TEMOTO_COMPONENT_MANAGER__COMPONENT_MANAGER_INTERFACE_H
#define TEMOTO_COMPONENT_MANAGER__COMPONENT_MANAGER_INTERFACE_H

#include "temoto_core/common/base_subsystem.h"
#include "temoto_core/common/topic_container.h"
#include "temoto_core/trr/resource_registrar.h"

#include "temoto_component_manager/component_manager_services.h"
#include <memory> //unique_ptr

/**
 * @brief The ComponentTopicsReq class
 */
class ComponentTopicsReq : public temoto_core::TopicContainer
{
  /* DELIBERATELY EMPTY */
};

/**
 * @brief The ComponentTopicsRes class
 */
class ComponentTopicsRes : public temoto_core::TopicContainer
{
  /* DELIBERATELY EMPTY */
};

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *                          COMPONENT MANAGER INTERFACE
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

namespace temoto_component_manager
{

/**
 * @brief Exposes simplified interface to Component Manager
 * 
 */
template <class ParentSubsystem>
class ComponentManagerInterface : public temoto_core::BaseSubsystem
{
public:
  /**
   * @brief Construct a new Component Manager Interface object
   * 
   */
  ComponentManagerInterface()
  {
    class_name_ = __func__;
  }

  /**
   * @brief Initializes the ComponentManagerInterface. This function must be called before using any other methods of ComponentManagerInterface 
   * @param parent_subsystem
   */
  void initialize(ParentSubsystem* parent_subsystem)
  {
    parent_subsystem_pointer_ = parent_subsystem;
    initializeBase(parent_subsystem);
    log_group_ = "interfaces." + parent_subsystem->class_name_;
    subsystem_name_ = parent_subsystem->class_name_ + "/component_manager_interface";

    resource_registrar_ = std::unique_ptr<temoto_core::trr::ResourceRegistrar<ComponentManagerInterface>>(new temoto_core::trr::ResourceRegistrar<ComponentManagerInterface>(subsystem_name_, this));
    resource_registrar_->registerStatusCb(&ComponentManagerInterface::statusInfoCb);
  }

  /**
   * @brief Invokes a component
   * 
   * @param component_type type of the component
   * @param use_only_local_components defines whether components could be invoked from other TeMoto instances
   * For example if invoking instance of TeMoto does not have a camera component, it asks other instances for the camera
   * @return ComponentTopicsRes Contains information about the topics published by the invoked component
   */
  ComponentTopicsRes startComponent(const std::string& component_type, bool use_only_local_components = false)
  {
    try
    {
      validateInterface();
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    return startComponent(component_type, "", "", ComponentTopicsReq(), use_only_local_components);
  }

  /**
   * @brief Invokes a component
   * 
   * @param component_type type of the component
   * @param topics output topics that the requested component should provide 
   * @param use_only_local_components defines whether components could be invoked from other TeMoto instances
   * For example if invoking instance of TeMoto does not have a camera component, it asks other instances for the camera
   * @return ComponentTopicsRes Contains information about the topics published by the invoked component 
   */
  ComponentTopicsRes startComponent( const std::string& component_type
                                   , const ComponentTopicsReq& topics
                                   , bool use_only_local_components = false)
  {
    try
    {
      validateInterface();
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    return startComponent(component_type, "", "", topics, use_only_local_components);
  }

  /**
   *  @brief Invokes a component
   * 
   * @param component_type type of the component
   * @param package_name name of the component package 
   * @param ros_program_name name of the node or launch file that should be executed in the package_name
   * @param topics output topics that the requested component should provide 
   * @param use_only_local_components defines whether components could be invoked from other TeMoto instances
   * For example if invoking instance of TeMoto does not have a camera component, it asks other instances for the camera
   * @return ComponentTopicsRes Contains information about the topics published by the invoked component
   */
  ComponentTopicsRes startComponent(const std::string& component_type
                            , const std::string& package_name
                            , const std::string& ros_program_name
                            , const ComponentTopicsReq& topics
                            , bool use_only_local_components = false)
  {
    
    try
    {
      validateInterface();
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    return startComponent(component_type, "", "", topics, ComponentTopicsReq(), use_only_local_components);
  }

  /**
   * @ @brief Invokes a component
   * 
   * @param component_type type of the component
   * @param package_name name of the component package
   * @param ros_program_name name of the node or launch file that should be executed in the package_name
   * @param topics output topics that the requested component should provide. 
   * @param parameters parameter specifications for the component, e.g., fame id, etc
   * @param use_only_local_components defines whether components could be invoked from other TeMoto instances 
   * For example if invoking instance of TeMoto does not have a camera component, it asks other instances for the camera
   * @return ComponentTopicsRes Contains information about the topics published by the invoked component
   */
  ComponentTopicsRes startComponent( const std::string& component_type
                                   , const std::string& package_name
                                   , const std::string& ros_program_name
                                   , const ComponentTopicsReq& topics
                                   , const ComponentTopicsReq& parameters
                                   , bool use_only_local_components = false)
  {
    // Fill out the "StartComponentRequest" request
    temoto_component_manager::LoadComponent srv_msg;
    srv_msg.request.component_type = component_type;
    srv_msg.request.package_name = package_name;
    srv_msg.request.executable = ros_program_name;
    srv_msg.request.use_only_local_components = use_only_local_components;
    srv_msg.request.output_topics = topics.outputTopicsAsKeyValues();
    srv_msg.request.input_topics = topics.inputTopicsAsKeyValues();
    srv_msg.request.required_parameters = parameters.outputTopicsAsKeyValues();

    // Call the server    
    try
    {
      resource_registrar_->template call<LoadComponent>(srv_name::MANAGER,
                                                      srv_name::SERVER,
                                                      srv_msg);
    }
    catch(temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    allocated_components_.push_back(srv_msg);
    ComponentTopicsRes responded_topics;
    responded_topics.setOutputTopicsByKeyValue( srv_msg.response.output_topics );

    return responded_topics;
  }

  /**
   * @brief Stops the component
   * 
   * @param load_comp_req 
   */
  void stopComponent(const temoto_component_manager::LoadComponent& load_comp_msg)
  {
    // The == operator used in the lambda function is defined in
    // component manager services header
    auto found_component_it = std::find_if(
        allocated_components_.begin(),
        allocated_components_.end(),
        [&](const LoadComponent& srv_msg) -> bool{ return srv_msg.request == load_comp_msg.request; });

    if (found_component_it == allocated_components_.end())
    {
      throw CREATE_ERROR(temoto_core::error::Code::RESOURCE_UNLOAD_FAIL, "Unable to unload resource that is not "
                                                            "loaded.");
    }

    try
    {
      // do the unloading
      resource_registrar_->unloadClientResource(found_component_it->response.trr.resource_id);
      allocated_components_.erase(found_component_it);
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }
  }

  /**
   * @brief Stops the component based on its type, package name, and executable name
   * 
   * @param component_type type of the component
   * @param package_name name of the component package 
   * @param ros_program_name name of the node or launch file that should be executed in the package_name
   */
  void stopComponent(std::string component_type, std::string package_name, std::string ros_program_name)
  {
    try
    {
      validateInterface();
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    // Find all instances where request part matches of what was given and unload each resource
    temoto_component_manager::LoadComponent::Request req;
    req.component_type = component_type;
    req.package_name = package_name;
    req.executable = ros_program_name;

    stopComponent(req);
  }

  /**
   * @brief Invokes a pipe
   * 
   * @param pipe_category specifies the category of the pipe, defined in a pipes.yaml file
   * @param segment_specifiers allows to set requirements for a specific segment within a pipe
   * @param use_only_local_segments defines whether components could be invoked from other TeMoto instances
   * @return temoto_core::TopicContainer Contains information about the topics published by the last segment of the pipe
   */
  temoto_core::TopicContainer startPipe( std::string pipe_category
    , const std::vector<PipeSegmentSpecifier>& segment_specifiers = std::vector<PipeSegmentSpecifier>()
    , bool use_only_local_segments = false)
  {
    // Validate the interface
    try
    {
      validateInterface();
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    // Start filling out the LoadPipe message
    LoadPipe load_pipe_msg;
    load_pipe_msg.request.pipe_category = pipe_category;
    load_pipe_msg.request.pipe_segment_specifiers = segment_specifiers;
    load_pipe_msg.request.use_only_local_segments = use_only_local_segments;

    try
    {
      resource_registrar_->template call<LoadPipe>(srv_name::MANAGER_2,
                                                 srv_name::PIPE_SERVER,
                                                 load_pipe_msg);
      allocated_pipes_.push_back(load_pipe_msg);
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    temoto_core::TopicContainer topics_to_return;
    topics_to_return.setOutputTopicsByKeyValue(load_pipe_msg.response.output_topics);

    return topics_to_return;
  }

  /**
   * @brief Stops a pipe
   * 
   * @param pipe_category 
   * @param segment_specifiers 
   * @param use_only_local_segments 
   * @return temoto_core::TopicContainer 
   */
  temoto_core::TopicContainer stopPipe( std::string pipe_category
    , const std::vector<PipeSegmentSpecifier>& segment_specifiers = std::vector<PipeSegmentSpecifier>()
    , bool use_only_local_segments = false)
  {
    try
    {
      validateInterface();
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    // Find all instances where request part matches of what was given and unload each resource
    LoadPipe load_pipe_msg;
    load_pipe_msg.request.pipe_category = pipe_category;
    load_pipe_msg.request.pipe_segment_specifiers = segment_specifiers;
    load_pipe_msg.request.use_only_local_segments = use_only_local_segments;

    // The == operator used in the lambda function is defined in
    // component manager services header
    auto found_pipe_it = std::find_if(
        allocated_pipes_.begin(),
        allocated_pipes_.end(),
        [&](const LoadPipe& srv_msg) -> bool{ return srv_msg.request == load_pipe_msg.request; });

    if (found_pipe_it == allocated_pipes_.end())
    {
      throw CREATE_ERROR(temoto_core::error::Code::RESOURCE_UNLOAD_FAIL, "Unable to unload resource that is not "
                                                            "loaded.");
    }

    try
    {
      // Do the unloading
      resource_registrar_->unloadClientResource(found_pipe_it->response.trr.resource_id);
      allocated_pipes_.erase(found_pipe_it);
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }
  }

  /**
   * @brief Registers a custom component recovery routine
   * 
   * @param callback 
   */
  void registerComponentStatusCallback( void (ParentSubsystem::*callback )(const LoadComponent&))
  {
    component_status_callback_ = callback;
  }

  /**
   * @brief Registers a custom update routine
   * 
   * @param callback 
   */
  void registerComponentUpdateCallback( void (ParentSubsystem::*callback )(const LoadComponent&))
  {
    component_update_callback_ = callback;
  }

  /**
   * @brief Registers a custom pipe recovery routine
   * 
   * @param callback 
   */
  void registerPipeStatusCallback( void (ParentSubsystem::*callback )(const LoadPipe&))
  {
    pipe_status_callback_ = callback;
  }

  ~ComponentManagerInterface()
  {
  }

  const std::string& getName() const
  {
    return subsystem_name_;
  }

private:
  std::vector<LoadComponent> allocated_components_;
  std::vector<LoadPipe> allocated_pipes_;

  void(ParentSubsystem::*component_status_callback_)(const LoadComponent&) = NULL;
  void(ParentSubsystem::*component_update_callback_)(const LoadComponent&) = NULL;
  void(ParentSubsystem::*pipe_status_callback_)(const LoadPipe&) = NULL;

  std::unique_ptr<temoto_core::trr::ResourceRegistrar<ComponentManagerInterface>> resource_registrar_;
  ParentSubsystem* parent_subsystem_pointer_;

  /**
   * @brief validateInterface
   */
  void validateInterface()
  {
    if(!resource_registrar_)
    {
      throw CREATE_ERROR(temoto_core::error::Code::UNINITIALIZED, "Interface is not initalized.");
    }
  }

  /**
   * @brief Receives component status update messages from the Context Manager
   * 
   * @param srv 
   */
  void statusInfoCb(temoto_core::ResourceStatus& srv)
  {
    try
    {
      validateInterface();

      TEMOTO_INFO_STREAM("status info was received");
      TEMOTO_INFO_STREAM(srv.request);

      /*
      * If the status message indicates a resource failure, then find out what was
      * the exact resource and execute a recovery behaviour
      */
      if (srv.request.status_code == temoto_core::trr::status_codes::FAILED)
      {
        TEMOTO_WARN("The status info reported a resource failure.");

        /*
        * Check if the resource that failed was a component
        */
        auto component_it = std::find_if(
          allocated_components_.begin(),
          allocated_components_.end(),
          [&](const temoto_component_manager::LoadComponent& comp) -> bool {
            return comp.response.trr.resource_id == srv.request.resource_id;
          });

        if (component_it != allocated_components_.end())
        {
          TEMOTO_WARN_STREAM("Sending a request to unload the failed component ...");
          resource_registrar_->unloadClientResource(component_it->response.trr.resource_id);
          
          /*
           * Check if the owner parent_subsystem has a status routine defined
           */
          if (component_status_callback_)
          {
            TEMOTO_WARN_STREAM("Executing a custom recovery behaviour defined in parent_subsystem '" 
              << parent_subsystem_pointer_->class_name_ << "'.");
            (parent_subsystem_pointer_->*component_status_callback_)(*component_it);
            return;
          }
          else
          {
            // Execute the default behavior for component failure, which is to load a new component
            TEMOTO_DEBUG("Asking the same component again");

            // this call automatically updates the response in allocated components vec
            component_it->request.output_topics = component_it->response.output_topics;
            resource_registrar_->template call<LoadComponent>(srv_name::MANAGER,
                                                            srv_name::SERVER,
                                                            *component_it);
          }
          return;
        }
        
        /*
        * Check if the resource that failed was a pipe
        */
        auto pipe_it = std::find_if(
          allocated_pipes_.begin(), 
          allocated_pipes_.end(),
          [&](const LoadPipe& pipe) -> bool {
            return pipe.response.trr.resource_id == srv.request.resource_id;
          });

        if (pipe_it != allocated_pipes_.end())
        {
          TEMOTO_WARN("Sending a request to unload the failed pipe ...");
          resource_registrar_->unloadClientResource(pipe_it->response.trr.resource_id);

          /*
           * Check if the owner parent_subsystem has a status routine defined
           */
          if (pipe_status_callback_)
          {
            TEMOTO_WARN_STREAM("Executing a custom recovery behaviour defined in parent_subsystem '" 
              << parent_subsystem_pointer_->class_name_ << "'.");
            (parent_subsystem_pointer_->*pipe_status_callback_)(*pipe_it);
            return;
          }
          else
          {
            // ... copy the output topics from the response into the output topics
            // of the request (since the user still wants to receive the data on the same topics) ...
            pipe_it->request.output_topics = pipe_it->response.output_topics;
            pipe_it->request.pipe_id = pipe_it->response.pipe_id;

            // ... and load an alternative pipe. This call automatically
            // updates the response in allocated pipes vector
            TEMOTO_DEBUG_STREAM("Trying to load an alternative pipe");
            resource_registrar_->template call<LoadPipe>(srv_name::MANAGER_2,
                                                      srv_name::PIPE_SERVER,
                                                      *pipe_it);
          }
          return;
        }

        TEMOTO_ERROR_STREAM("Resource status arrived for a resource that does not exist.");
        // throw CREATE_ERROR(temoto_core::error::Code::RESOURCE_NOT_FOUND, "Resource status arrived for a "
        //                    "resource that does not exist.");
      }
      else if (srv.request.status_code == temoto_core::trr::status_codes::UPDATE)
      {
        TEMOTO_DEBUG_STREAM(srv.request.message);

        auto component_it = std::find_if(
          allocated_components_.begin(),
          allocated_components_.end(),
          [&](const temoto_component_manager::LoadComponent& comp) -> bool {
            return comp.response.trr.resource_id == srv.request.resource_id;
          });

        if (component_it == allocated_components_.end())
        {
          TEMOTO_ERROR_STREAM("Resource status arrived for a resource that does not exist.");
          return;
        }
        TEMOTO_DEBUG_STREAM("Executing a custom update behaviour defined in parent_subsystem '" 
          << parent_subsystem_pointer_->class_name_ << "'.");
          (parent_subsystem_pointer_->*component_update_callback_)(*component_it);
        return;
      }
      
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }
  }
};

} // namespace

#endif
