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
#include "temoto_core/rmp/resource_manager.h"

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

template <class OwnerAction>
class ComponentManagerInterface : public temoto_core::BaseSubsystem
{
public:
  /**
   * @brief ComponentManagerInterface
   */
  ComponentManagerInterface()
  {
    class_name_ = __func__;
  }

  /**
   * @brief initialize
   * @param task
   */
  void initialize(OwnerAction* task)
  {
    owner_instance_ = task;
    initializeBase(task);
    log_group_ = "interfaces." + task->getPackageName();
    name_ = task->getName() + "/component_manager_interface";

    resource_manager_ = std::unique_ptr<temoto_core::rmp::ResourceManager<ComponentManagerInterface>>(new temoto_core::rmp::ResourceManager<ComponentManagerInterface>(name_, this));
    resource_manager_->registerStatusCb(&ComponentManagerInterface::statusInfoCb);
  }

  /**
   * @brief startComponent
   * @param component_type
   * @return
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
   * @brief startComponent
   * @param component_type
   * @param topics
   * @return
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
   * @brief startComponent
   * @param algorithm_type
   * @param package_name
   * @param ros_program_name
   * @param topics
   * @return
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
      resource_manager_->template call<LoadComponent>(srv_name::MANAGER,
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
   * @brief stopComponent
   * @param component_type
   * @param package_name
   * @param ros_program_name
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

    // The == operator used in the lambda function is defined in
    // component manager services header
    auto found_component_it = std::find_if(
        allocated_components_.begin(),
        allocated_components_.end(),
        [&](const LoadComponent& srv_msg) -> bool{ return srv_msg.request == req; });

    if (found_component_it == allocated_components_.end())
    {
      throw CREATE_ERROR(temoto_core::error::Code::RESOURCE_UNLOAD_FAIL, "Unable to unload resource that is not "
                                                            "loaded.");
    }

    try
    {
      // do the unloading
      resource_manager_->unloadClientResource(found_component_it->response.rmp.resource_id);
      allocated_components_.erase(found_component_it);
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }
  }

  /**
   * @brief startPipe
   * @param pipe_category
   * @return
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
      resource_manager_->template call<LoadPipe>(srv_name::MANAGER_2,
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
   * @brief statusInfoCb
   * @param srv
   */
  void statusInfoCb(temoto_core::ResourceStatus& srv)
  {
    try
    {
      validateInterface();
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    TEMOTO_INFO_STREAM("status info was received");
    TEMOTO_INFO_STREAM(srv.request);

    /*
     * Check if the owner action has a status routine defined
     */
    if (update_callback_)
    {
      (owner_instance_->*update_callback_)(false);
      return;
    }

    /*
     * if any resource should fail, just unload it and load it again
     * there is a chance that component manager gives us better component this time
     */
    if (srv.request.status_code == temoto_core::rmp::status_codes::FAILED)
    {
      TEMOTO_WARN("Component manager interface detected a component failure. Unloading and "
                                "trying again");
      auto comp_it = std::find_if(allocated_components_.begin(), allocated_components_.end(),
                                  [&](const temoto_component_manager::LoadComponent& comp) -> bool {
                                    return comp.response.rmp.resource_id == srv.request.resource_id;
                                  });

      auto pipe_it = std::find_if(allocated_pipes_.begin(), allocated_pipes_.end(),
                                  [&](const LoadPipe& sens) -> bool {
                                    return sens.response.rmp.resource_id == srv.request.resource_id;
                                  });

      if (comp_it != allocated_components_.end())
      {
        TEMOTO_DEBUG("Unloading");
        resource_manager_->unloadClientResource(comp_it->response.rmp.resource_id);
        TEMOTO_DEBUG("Asking the same component again");

        // this call automatically updates the response in allocated components vec
        try
        {
          comp_it->request.output_topics = comp_it->response.output_topics;
          resource_manager_->template call<LoadComponent>(srv_name::MANAGER,
                                                          srv_name::SERVER,
                                                          *comp_it);
        }
        catch(temoto_core::error::ErrorStack& error_stack)
        {
          SEND_ERROR(error_stack);
        }
      }

      // If the pipe was found then ...
      else if (pipe_it != allocated_pipes_.end())
      {
        try
        {
          // ... unload it and ...
          TEMOTO_DEBUG("Unloading the pipe");
          resource_manager_->unloadClientResource(pipe_it->response.rmp.resource_id);

          // ... copy the output topics from the response into the output topics
          // of the request (since the user still wants to receive the data on the same topics) ...
          pipe_it->request.output_topics = pipe_it->response.output_topics;
          pipe_it->request.pipe_id = pipe_it->response.pipe_id;

          // ... and load an alternative pipe. This call automatically
          // updates the response in allocated pipes vector

          TEMOTO_DEBUG_STREAM("Trying to load an alternative pipe");

          resource_manager_->template call<LoadPipe>(srv_name::MANAGER_2,
                                                     srv_name::PIPE_SERVER,
                                                     *pipe_it);
        }
        catch(temoto_core::error::ErrorStack& error_stack)
        {
          throw FORWARD_ERROR(error_stack);
        }
      }

      else
      {
        throw CREATE_ERROR(temoto_core::error::Code::RESOURCE_NOT_FOUND, "Resource status arrived for a "
                           "resource that does not exist.");
      }
    }
  }

  /**
   * @brief registerUpdateCallback
   */
  void registerUpdateCallback( void (OwnerAction::*callback )(bool))
  {
    update_callback_ = callback;
  }

  ~ComponentManagerInterface()
  {
  }

  const std::string& getName() const
  {
    return name_;
  }

private:
  std::string name_;
  std::vector<temoto_component_manager::LoadComponent> allocated_components_;
  std::unique_ptr<temoto_core::rmp::ResourceManager<ComponentManagerInterface>> resource_manager_;

  void(OwnerAction::*update_callback_)(bool) = NULL;
  OwnerAction* owner_instance_;

  std::vector<LoadPipe> allocated_pipes_;

  /**
   * @brief validateInterface
   */
  void validateInterface()
  {
    if(!resource_manager_)
    {
      throw CREATE_ERROR(temoto_core::error::Code::UNINITIALIZED, "Interface is not initalized.");
    }
  }
};

} // namespace

#endif
