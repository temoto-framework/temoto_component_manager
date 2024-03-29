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

#ifndef TEMOTO_COMPONENT_MANAGER__COMPONENT_MANAGER_SERVERS_H
#define TEMOTO_COMPONENT_MANAGER__COMPONENT_MANAGER_SERVERS_H

#include "rr/ros1_resource_registrar.h"
#include "temoto_core/common/base_subsystem.h"
#include "temoto_core/common/temoto_id.h"
#include "temoto_component_manager/component_info_registry.h"
#include "temoto_component_manager/component_manager_services.h"
#include "temoto_process_manager/process_manager_services.hpp"
#include "std_msgs/String.h"
#include <mutex>
#include <boost/optional.hpp>
#include <tuple>

namespace temoto_component_manager
{
typedef std::tuple<LoadComponent, ComponentInfo, temoto_process_manager::LoadProcess> AllocCompTuple;
typedef std::pair<LoadPipe, std::pair<PipeInfo, std::vector<LoadComponent>>> AllocPipePair;

/**
 * @brief The ComponentManagerServers contains all Component Manager related ROS services.
 */
class ComponentManagerServers : public temoto_core::BaseSubsystem
{
public:

  /**
   * @brief Constructor.
   * @param b pointer to parent class (each subsystem in TeMoto inherits the base subsystem class).
   * @param sid pointer to Component Info Database.
   */
  ComponentManagerServers( temoto_core::BaseSubsystem* b, ComponentInfoRegistry* cir );

  /**
   * @brief ~ComponentManagerServers
   */
  ~ComponentManagerServers();

  /**
   * @brief getName
   * @return Name of this subsystem.
   */
  const std::string& getName() const
  {
    return subsystem_name_;
  }

  /**
   * @brief TODO:test
   * 
   * @param test 
   */
  void cirUpdateCallback(ComponentInfo component);
    
private:

  bool listComponentsCb(ListComponents::Request& req, ListComponents::Response& res);
  bool listPipesCb(ListPipes::Request& req, ListPipes::Response& res);

  /**
   * @brief Callback to a service that executes/runs a requested device
   * and sends back the topic that the device is publishing to
   * @param req
   * @param res
   */
  void loadComponentCb(LoadComponent::Request& req, LoadComponent::Response& res);

  /**
   * @brief Called when a component is unloaded.
   * @param req
   * @param res
   */
  void unloadComponentCb(LoadComponent::Request& req, LoadComponent::Response& res);
  
  /**
   * @brief 
   * 
   * @param req 
   * @param res 
   * @param status_msg 
   */
  void componentStatusCb(LoadComponent::Request& req
  , LoadComponent::Response& res
  , temoto_resource_registrar::Status status_msg);

  /**
   * @brief Callback to pipe setup service
   * 
   * @param req 
   * @param res 
   */
  void loadPipeCb(LoadPipe::Request& req, LoadPipe::Response& res);

  /**
   * @brief Callback for pipe unloading routines
   * 
   * @param req 
   * @param res 
   */
  void unloadPipeCb(LoadPipe::Request& req, LoadPipe::Response& res);

  /**
   * @brief Called when component status update information is received.
   * 
   * @param srv_msg 
   * @param status_msg 
   */
  void erStatusCb(temoto_process_manager::LoadProcess srv_msg, temoto_resource_registrar::Status status_msg);

  /**
   * @brief 
   * 
   * @param srv_msg 
   * @param status_msg 
   */
  void pipeStatusCb(LoadComponent srv_msg, temoto_resource_registrar::Status status_msg);

  /**
   * @brief A function that helps to manage component topic related information.
   * @param req_topics Topics that were requested.
   * @param res_topics Response part for the requested topics.
   * @param load_er_msg If topic name remapping is required, then the remapped topic names
   * are placed into this message structure (used later for invoking the component via External
   * Resource Manager)
   * @param component_info Data structure that contains information about the particular component.
   * @param direction Specifies whether input or output topics are managed.
   */
  void processTopics( std::vector<diagnostic_msgs::KeyValue>& req_topics
  , std::vector<diagnostic_msgs::KeyValue>& res_topics
  , temoto_process_manager::LoadProcess& load_er_msg
  , ComponentInfo& component_info
  , std::string direction);

  void processParameters( std::vector<diagnostic_msgs::KeyValue>& req_parameters
  , std::vector<diagnostic_msgs::KeyValue>& res_parameters
  , temoto_process_manager::LoadProcess& load_er_msg
  , ComponentInfo& component_info);

  /**
   * @brief Restores the state of the Component Manager via RR Catalog
   * 
   */
  void restoreState();

  /**
   * @brief Checks if given component is already in use
   * 
   * @param ci_to_check 
   * @return temoto_core::temoto_id::ID 
   */
  boost::optional<AllocCompTuple> checkIfInUse(const LoadComponent::Request& req) const;

  ros::NodeHandle nh_;
  ros::ServiceServer list_components_server_;
  ros::ServiceServer list_pipes_server_;

  /// Pointer to a central Component Info Registry object.
  ComponentInfoRegistry* cir_;

  temoto_resource_registrar::ResourceRegistrarRos1 resource_registrar_;
  temoto_resource_registrar::Configuration rr_catalog_config_;

  /// Generates unique id's for the pipes
  temoto_core::temoto_id::IDManager pipe_id_generator_;

  /// List of allocated components
  std::vector<AllocCompTuple> allocated_components_;
  mutable std::recursive_mutex allocated_components_mutex_;

  /// List of allocated pipes
  std::vector<AllocPipePair> allocated_pipes_;
  mutable std::recursive_mutex allocated_pipes_mutex_;

}; // ComponentManagerServers

} // component_manager namespace

#endif
