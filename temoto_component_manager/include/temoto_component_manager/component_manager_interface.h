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

#ifndef TEMOTO_COMPONENT_MANAGER__COMPONENT_MANAGER_INTERFACE_H
#define TEMOTO_COMPONENT_MANAGER__COMPONENT_MANAGER_INTERFACE_H

#include "temoto_core/common/base_subsystem.h"
#include "temoto_core/common/topic_container.h"
#include "rr/ros1_resource_registrar.h"

#include "temoto_component_manager/component_manager_services.h"
#include <memory>
#include <ctime>
#include <functional>

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
class ComponentManagerInterface : public temoto_core::BaseSubsystem
{
public:
  /**
   * @brief Construct a new Component Manager Interface object
   * 
   */
  ComponentManagerInterface(bool initialize_interface = false)
  : unique_suffix_(std::to_string(createID()))
  , has_owner_(false)
  , initialized_(false)
  {
    class_name_ = __func__;
    if (initialize_interface)
    {
      initialize();
    }
  }

  void initialize(const BaseSubsystem& owner)
  {
    if (!initialized_)
    {
      initializeBase(owner);
      log_group_ = "interfaces." + owner.subsystem_name_;
      rr_name_ = owner.class_name_ + "/" + class_name_ + "_" + unique_suffix_;
      has_owner_ = true;
      initialize();
    }
    else
    {
      TEMOTO_WARN_STREAM("The Component Manager Interface is already initialized");
    }
  }

  void initialize()
  {
    if (!initialized_)
    {
      if (!has_owner_)
      {
        rr_name_ = class_name_ + "_" + unique_suffix_;
      }
      resource_registrar_ = std::make_unique<temoto_resource_registrar::ResourceRegistrarRos1>(rr_name_);
      resource_registrar_->init();
      initialized_ = true;
    }
    else
    {
      TEMOTO_WARN_STREAM("The External Resource Manager interface is already initialized");
    }
  }

  unsigned int createID()
  {
    std::srand(std::time(nullptr));
    return std::rand();
  }

  ListComponents::Response listComponents(const std::string& component_type = "")
  {
    ListComponents msg;
    msg.request.type = component_type;

    if (!client_list_components_.call(msg))
    {
      throw CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL, "Service call returned false.");
    }
    else
    {
      return msg.response;
    }
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
  , bool use_only_local_components = false
  , std::string temoto_namespace = "")
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

    return startComponent(srv_msg, temoto_namespace);
  }

  ComponentTopicsRes startComponent( temoto_component_manager::LoadComponent& load_component_srv_msg
                                   , std::string temoto_namespace = "")
  {
    #ifdef enable_tracing
    std::unique_ptr<opentracing::Span> tracing_span;

    if (resource_registrar_->statusCallbackActive())
    {
      temoto_core::StringMap parent_context = resource_registrar_->getStatusCallbackSpanContext();
      TextMapCarrier carrier(parent_context);
      auto span_context_maybe = TRACER->Extract(carrier);
      tracing_span = TRACER->StartSpan(this->class_name_ + "::" + __func__, {opentracing::ChildOf(span_context_maybe->get())});
    }
    else
    {
      tracing_span = TRACER->StartSpan(this->class_name_ + "::" + __func__);
    }
    #endif

    if(temoto_namespace.empty())
    {
      temoto_namespace = temoto_core::common::getTemotoNamespace();
    }

    // Call the server    
    try
    {
      #ifdef enable_tracing
      /*
       * If tracing is enabled:
       * Propagate the context of the span to the invoked subroutines
       * TODO: this segment of code will crash if the tracer is uninitialized
       */ 
      temoto_core::StringMap local_span_context;
      TextMapCarrier carrier(local_span_context);
      auto err = TRACER->Inject(tracing_span->context(), carrier);
      
      resource_registrar_->template call<LoadComponent>( srv_name::MANAGER
      , srv_name::SERVER
      , load_component_srv_msg
      , temoto_core::trr::FailureBehavior::NONE
      , temoto_namespace
      , local_span_context);

      #else

      resource_registrar_->call<LoadComponent>(srv_name::MANAGER
      , srv_name::SERVER
      , load_component_srv_msg
      , std::bind(&ComponentManagerInterface::componentStatusCb, this, std::placeholders::_1, std::placeholders::_2));

      #endif
    }
    catch(temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    allocated_components_.insert({load_component_srv_msg.response.temotoMetadata.requestId, load_component_srv_msg});
    ComponentTopicsRes responded_topics;
    responded_topics.setOutputTopicsByKeyValue(load_component_srv_msg.response.output_topics);

    return responded_topics;
  }

  /**
   * @brief Stops the component
   * 
   * @param load_comp_req 
   */
  void stopComponent(const LoadComponent& load_comp_msg)
  try
  {
    auto resource_id = allocated_components_.at(load_comp_msg.response.temotoMetadata.requestId).response.temotoMetadata.requestId;
    TEMOTO_WARN_STREAM("unloading " << load_comp_msg.request << "\n" << load_comp_msg.response << "\n" << " with id: " << resource_id);
    resource_registrar_->unload(srv_name::MANAGER, resource_id);
    allocated_components_.erase(load_comp_msg.response.temotoMetadata.requestId);
  }
  catch(temoto_core::error::ErrorStack& error_stack)
  {
    throw FORWARD_ERROR(error_stack);
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
    auto load_comp_msg = std::find_if(allocated_components_.begin()
    , allocated_components_.end()
    , [&](const std::pair<std::string, LoadComponent>& lsm)
      {
        return lsm.second.request.component_type == component_type
        && lsm.second.request.package_name == package_name
        && lsm.second.request.executable == ros_program_name;
      });

    if (load_comp_msg != allocated_components_.end())
    {
      stopComponent(load_comp_msg->second);
    }
    else
    {
      throw CREATE_ERROR(temoto_core::error::Code::RESOURCE_NOT_FOUND
      , "Could not find a component '%s'."
      , component_type.c_str());
    }
  }

  /**
   * @brief Invokes a pipe
   * 
   * @param pipe_category specifies the category of the pipe, defined in a pipes.yaml file
   * @param segment_specifiers allows to set requirements for a specific segment within a pipe
   * @param use_only_local_segments defines whether components could be invoked from other TeMoto instances
   * @return temoto_core::TopicContainer Contains information about the topics published by the last segment of the pipe
   */
  temoto_core::TopicContainer startPipe(std::string pipe_category
  , const std::vector<PipeSegmentSpecifier>& segment_specifiers = std::vector<PipeSegmentSpecifier>()
  , bool use_only_local_segments = false)
  {
    // Start filling out the LoadPipe message
    LoadPipe load_pipe_msg;
    load_pipe_msg.request.pipe_category = pipe_category;
    load_pipe_msg.request.pipe_segment_specifiers = segment_specifiers;
    load_pipe_msg.request.use_only_local_segments = use_only_local_segments;

    return startPipe(load_pipe_msg);
  }

  temoto_core::TopicContainer startPipe(LoadPipe& load_pipe_msg, std::string temoto_namespace = "")
  {
    #ifdef enable_tracing
    std::unique_ptr<opentracing::Span> tracing_span;

    if (resource_registrar_->statusCallbackActive())
    {
      temoto_core::StringMap parent_context = resource_registrar_->getStatusCallbackSpanContext();
      TextMapCarrier carrier(parent_context);
      auto span_context_maybe = TRACER->Extract(carrier);
      tracing_span = TRACER->StartSpan(this->class_name_ + "::" + __func__, {opentracing::ChildOf(span_context_maybe->get())});
    }
    else
    {
      tracing_span = TRACER->StartSpan(this->class_name_ + "::" + __func__);
    }
    #endif

    if(temoto_namespace.empty())
    {
      temoto_namespace = temoto_core::common::getTemotoNamespace();
    }

    TEMOTO_DEBUG_STREAM("Loading a pipe of type '" << load_pipe_msg.request.pipe_category << "' ...");
    try
    {
      #ifdef enable_tracing
      /*
       * If tracing is enabled:
       * Propagate the context of the span to the invoked subroutines
       * TODO: this segment of code will crash if the tracer is uninitialized
       */ 
      temoto_core::StringMap local_span_context;
      TextMapCarrier carrier(local_span_context);
      auto err = TRACER->Inject(tracing_span->context(), carrier);
      
      resource_registrar_->template call<LoadPipe>( srv_name::MANAGER_2
      , srv_name::PIPE_SERVER
      , load_pipe_msg
      , temoto_core::trr::FailureBehavior::NONE
      , temoto_namespace
      , local_span_context);

      #else
      // If tracing is not enabled
      resource_registrar_->call<LoadPipe>(srv_name::MANAGER
      , srv_name::PIPE_SERVER
      , load_pipe_msg
      , std::bind(&ComponentManagerInterface::pipeStatusCb, this, std::placeholders::_1, std::placeholders::_2));

      #endif

      allocated_pipes_.insert({load_pipe_msg.response.temotoMetadata.requestId, load_pipe_msg});
      temoto_core::TopicContainer topics_to_return;
      topics_to_return.setOutputTopicsByKeyValue(load_pipe_msg.response.output_topics);
      return topics_to_return;
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }
  }

  // void reloadPipe( std::string pipe_category
  //   , const std::vector<PipeSegmentSpecifier>& segment_specifiers = std::vector<PipeSegmentSpecifier>()
  //   , bool use_only_local_segments = false)
  // {
  //   TEMOTO_DEBUG_STREAM("Reloading a pipe of type '" << pipe_category << "' ...");

  //   LoadPipe load_pipe_msg;
  //   if (!findPipe(load_pipe_msg, pipe_category, segment_specifiers, use_only_local_segments))
  //   {
  //     throw CREATE_ERROR(temoto_core::error::Code::RESOURCE_UNLOAD_FAIL,
  //       "Unable to reload a resource that is not loaded.");
  //   }

  //   // Stop the pipe
  //   stopPipe(pipe_category, segment_specifiers, use_only_local_segments);

  //   // Reload the pipe
  //   load_pipe_msg.request.pipe_id = load_pipe_msg.response.pipe_id;
  //   startPipe(load_pipe_msg);
  // }

  /**
   * @brief 
   * 
   * @param load_pipe_msg 
   */
  void stopPipe(const LoadPipe& load_pipe_msg)
  try
  {
    auto resource_id = allocated_pipes_.at(load_pipe_msg.response.temotoMetadata.requestId).response.temotoMetadata.requestId;
    resource_registrar_->unload(srv_name::MANAGER, resource_id);
    allocated_components_.erase(load_pipe_msg.response.temotoMetadata.requestId);
  }
  catch(temoto_core::error::ErrorStack& error_stack)
  {
    throw FORWARD_ERROR(error_stack);
  }

  /**
   * @brief Registers a custom component recovery routine
   * 
   * @param callback 
   */
  void registerComponentStatusCallback(std::function<void(LoadComponent, temoto_resource_registrar::Status)> callback)
  {
    user_component_status_callback_ = callback;
  }

  /**
   * @brief Registers a custom pipe recovery routine
   * 
   * @param callback 
   */
  void registerPipeStatusCallback( std::function<void(LoadPipe, temoto_resource_registrar::Status)> callback)
  {
    user_pipe_status_callback_ = callback;
  }

  ~ComponentManagerInterface()
  {
  }

  const std::string& getName() const
  {
    return rr_name_;
  }

private:

  /**
   * @brief 
   * 
   * @param srv_msg 
   * @param status_msg 
   */
  void componentStatusCb(LoadComponent srv_msg, temoto_resource_registrar::Status status_msg)
  try
  {
    TEMOTO_DEBUG_STREAM("status info was received about component:\n" << srv_msg.request);

    /*
     * Check if the owner has a status routine defined
     */
    if (user_component_status_callback_)
    {
      TEMOTO_DEBUG_STREAM("Invoking user-registered status callback");
      user_component_status_callback_(srv_msg, status_msg);
      return;
    }
    else
    {
      auto local_srv_msg = std::find_if(allocated_components_.begin()
      , allocated_components_.end()
      , [&srv_msg](const std::pair<std::string, LoadComponent>& lsm)
        {
          return lsm.second.response.temotoMetadata.requestId == srv_msg.response.temotoMetadata.requestId;
        });

      if (local_srv_msg == allocated_components_.end())
      {
        throw CREATE_ERROR(temoto_core::error::Code::RESOURCE_NOT_FOUND
        , "Could not find a resource with id: '%s'."
        , srv_msg.response.temotoMetadata.requestId.c_str());
      }

      TEMOTO_DEBUG_STREAM("Unloading the failed resource");
      resource_registrar_->unload(srv_name::MANAGER
      , srv_msg.response.temotoMetadata.requestId);

      LoadComponent new_srv_msg;
      new_srv_msg.request = srv_msg.request;

      TEMOTO_DEBUG_STREAM("Asking the same resource again");
      resource_registrar_->call<LoadComponent>(srv_name::MANAGER
      , srv_name::SERVER
      , new_srv_msg);

      allocated_components_[local_srv_msg->first] = new_srv_msg;
    }
  }
  catch (temoto_core::error::ErrorStack& error_stack)
  {
    throw FORWARD_ERROR(error_stack);
  }

  /**
   * @brief 
   * 
   * @param srv_msg 
   * @param status_msg 
   */
  void pipeStatusCb(LoadPipe srv_msg, temoto_resource_registrar::Status status_msg)
  try
  {
    /*
     * Check if the owner has a status routine defined
     */
    if (user_pipe_status_callback_)
    {
      TEMOTO_DEBUG_STREAM("Invoking user-registered status callback");
      user_pipe_status_callback_(srv_msg, status_msg);
      return;
    }
    else
    {
      auto local_srv_msg = std::find_if(allocated_pipes_.begin()
      , allocated_pipes_.end()
      , [&srv_msg](const std::pair<std::string, LoadPipe>& lsm)
        {
          return lsm.second.response.temotoMetadata.requestId == srv_msg.response.temotoMetadata.requestId;
        });

      if (local_srv_msg == allocated_pipes_.end())
      {
        throw CREATE_ERROR(temoto_core::error::Code::RESOURCE_NOT_FOUND
        , "Could not find a resource with id: '%s'."
        , srv_msg.response.temotoMetadata.requestId.c_str());
      }

      TEMOTO_DEBUG_STREAM("Unloading the failed resource");
      resource_registrar_->unload(srv_name::MANAGER
      , srv_msg.response.temotoMetadata.requestId);

      LoadPipe new_srv_msg;
      new_srv_msg.request = srv_msg.request;
      new_srv_msg.request.output_topics = srv_msg.response.output_topics;
      new_srv_msg.request.pipe_id = srv_msg.response.pipe_id;

      TEMOTO_DEBUG_STREAM("Asking the same resource again");
      resource_registrar_->call<LoadPipe>(srv_name::MANAGER
      , srv_name::PIPE_SERVER
      , new_srv_msg);

      allocated_pipes_[local_srv_msg->first] = new_srv_msg;
    }
  }
  catch (temoto_core::error::ErrorStack& error_stack)
  {
    throw FORWARD_ERROR(error_stack);
  }

  bool findPipe(LoadPipe& lp_return_msg
  , const std::string& pipe_category
  , const std::vector<PipeSegmentSpecifier>& segment_specifiers = std::vector<PipeSegmentSpecifier>()
  , bool use_only_local_segments = false) const
  {
    // Find all instances where request part matches of what was given and unload each resource
    LoadPipe load_pipe_msg;
    load_pipe_msg.request.pipe_category = pipe_category;
    load_pipe_msg.request.pipe_segment_specifiers = segment_specifiers;
    load_pipe_msg.request.use_only_local_segments = use_only_local_segments;

    // The == operator used in the lambda function is defined in
    // component manager services header
    auto found_pipe_it = std::find_if(allocated_pipes_.begin()
    , allocated_pipes_.end()
    , [&](const std::pair<std::string, LoadPipe>& srv_msg)
      { 
        return srv_msg.second.request == load_pipe_msg.request; 
      });

    if (found_pipe_it != allocated_pipes_.end())
    {
      lp_return_msg = found_pipe_it->second;
      return true;
    }
    else
    {
      return false;
    }
  }

  /*
   * Class members
   */
  ros::NodeHandle nh_;
  ros::ServiceClient client_list_components_;

  std::string rr_name_;
  std::string unique_suffix_;
  bool has_owner_;
  bool initialized_;
  std::unique_ptr<temoto_resource_registrar::ResourceRegistrarRos1> resource_registrar_;

  std::map<std::string, LoadComponent> allocated_components_;
  std::map<std::string, LoadPipe> allocated_pipes_;

  std::function<void(LoadComponent, temoto_resource_registrar::Status)> user_component_status_callback_ = NULL;
  std::function<void(LoadPipe, temoto_resource_registrar::Status)> user_pipe_status_callback_ = NULL;
};

} // namespace

#endif
