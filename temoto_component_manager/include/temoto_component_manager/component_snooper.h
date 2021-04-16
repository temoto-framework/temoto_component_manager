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

#ifndef TEMOTO_COMPONENT__MANAGER_COMPONENT_SNOOPER_H
#define TEMOTO_COMPONENT__MANAGER_COMPONENT_SNOOPER_H

#include "temoto_core/common/base_subsystem.h"
#include "temoto_core/trr/config_synchronizer.h"
#include "temoto_component_manager/component_info_registry.h"
#include "temoto_action_engine/action_engine.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

namespace temoto_component_manager
{

/**
 * @brief Component Snooper is responsible of finding component devices and updating other component snoopers
 * about available components (in a system where multiple instances of temoto are running).
 * Component Snooper uses snooping agents (temoto actions) to discover component devices.
 */
class ComponentSnooper : public temoto_core::BaseSubsystem
{
  /**
   * @brief Defines what kind of data type is used in component info synchronization messages.
   */
  typedef std_msgs::String PayloadType;

public:

  /**
   * @brief Constructor
   * @param b Pointer to the parent subsystem that embeds this object.
   * @param cir Pointer to the Component Info Registry.
   */
  ComponentSnooper( temoto_core::BaseSubsystem* b, ComponentInfoRegistry* cir, const std::string& config_base_path);

  /**
   * @brief Advertises a component to other component snoopers.
   * @param si Component to advertise.
   */
  void advertiseComponent(ComponentInfo& si) const;

  /**
   * @brief Advertises all components in the local system.
   */
  void advertiseLocalComponents() const;

  /**
   * @brief Executes the component snooping process. Utilizes Temoto actions as snooping agents.
   */
  void startSnooping();

  /**
   * @brief A helper function that is used for converting component yaml descriptions to component info
   * objects.
   * @param config Component Info in a yaml node format.
   * @return
   */
  std::vector<ComponentInfoPtr> parseComponents(const YAML::Node& config);

  /// Destructor
  ~ComponentSnooper();

private:

  /**
   * @brief A callback function that is called when other instance of temoto has advertised
   * its components.
   * @param msg Incoming message
   * @param payload Data portion of the message
   */
  void syncCb(const temoto_core::ConfigSync& msg, const PayloadType& payload);

  /**
   * @brief A timer event callback function which checks if local component info entries have been
   * updated and if so, then advertises local components via #advertiseComponent.
   * @param e
   */
  void updateMonitoringTimerCb(const ros::TimerEvent &e);

  /// NodeHandle for the timer
  ros::NodeHandle nh_;

  /// Object that handles component info syncronization.
  temoto_core::trr::ConfigSynchronizer<ComponentSnooper, PayloadType> config_syncer_;

  /// Pointer to a central Component Info Registry object.
  ComponentInfoRegistry* cir_;

  /// Used for managing snooper agents.
  ActionEngine action_engine_;

  /// Base path to components and pipes yaml files
  std::string config_base_path_;

  /**
   * @brief Timer for checking local component info updates (timer event will trigger the #updateMonitoringTimerCb).
   * The local component info objects are asynchronously updated/created by snooper agents and this timer
   * is responsible for checking if the updates are advertised to other instances of temoto.
   */
  ros::Timer update_monitoring_timer_;

};

} // component_manager namespace

#endif
