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

#ifndef TEMOTO_COMPONENT_MANAGER__COMPONENT_MANAGER_SERVERS_H
#define TEMOTO_COMPONENT_MANAGER__COMPONENT_MANAGER_SERVERS_H

#include "temoto_core/common/base_subsystem.h"
#include "temoto_core/common/temoto_id.h"
#include "temoto_core/rmp/resource_manager.h"
#include "temoto_component_manager/component_info_registry.h"
#include "temoto_component_manager/component_manager_services.h"
#include "temoto_er_manager/temoto_er_manager_services.h"
#include "std_msgs/String.h"

namespace temoto_component_manager
{

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
   * @param srv
   */
  void statusCb1(temoto_core::ResourceStatus& srv);

  /**
   * @brief Called when component status update information is received.
   * @param srv
   */
  void statusCb2(temoto_core::ResourceStatus& srv);

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
                    , temoto_er_manager::LoadExtResource& load_er_msg
                    , ComponentInfo& component_info
                    , std::string direction);

  void processParameters( std::vector<diagnostic_msgs::KeyValue>& req_parameters
                        , std::vector<diagnostic_msgs::KeyValue>& res_parameters
                        , temoto_er_manager::LoadExtResource& load_er_msg
                        , ComponentInfo& component_info);

  /**
   * @brief Checks if given component is already in use
   * 
   * @param ci_to_check 
   * @return temoto_core::temoto_id::ID 
   */
  temoto_core::temoto_id::ID checkIfInUse( const std::vector<ComponentInfo>& cis_to_check) const;

  ros::NodeHandle nh_;
  ros::ServiceServer list_components_server_;
  ros::ServiceServer list_pipes_server_;

  /// Pointer to a central Component Info Registry object.
  ComponentInfoRegistry* cir_;

  /// Resource Management object which handles resource requests and status info propagation.
  temoto_core::rmp::ResourceManager<ComponentManagerServers> resource_manager_1_;

  /*
   * TODO: The second manager is used for making RMP calls within the same manager. If the same
   * resouce manager is used for calling servers managed by the same manager, the calls will lock
   */
  temoto_core::rmp::ResourceManager<ComponentManagerServers> resource_manager_2_;

  /// Generates unique id's for the pipes
  temoto_core::temoto_id::IDManager pipe_id_generator_;

  /*
   * TODO: A DATA STRUCTURE THAT IS A TEMPORARY HACK UNTIL RMP IS IMPROVED
   */
  std::map<int, std::pair<PipeInfo, std::vector<int>>>  allocated_pipes_hack_;

  /// List of allocated components
  std::map<temoto_core::temoto_id::ID, ComponentInfo> allocated_components_;
  std::map<temoto_core::temoto_id::ID, temoto_er_manager::LoadExtResource> allocated_ext_resources_;

}; // ComponentManagerServers

} // component_manager namespace

#endif
