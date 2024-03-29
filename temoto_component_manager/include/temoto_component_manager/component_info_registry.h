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

#ifndef TEMOTO_COMPONENT_MANAGER__COMPONENT_INFO_REGISTRY_H
#define TEMOTO_COMPONENT_MANAGER__COMPONENT_INFO_REGISTRY_H

#include "temoto_core/common/base_subsystem.h"
#include "temoto_component_manager/component_info.h"
#include "temoto_component_manager/pipe_info.h"
#include "temoto_component_manager/LoadComponent.h"
#include "temoto_component_manager/LoadPipe.h"
#include "temoto_core/common/temoto_id.h"

#include <vector>
#include <mutex>
#include <thread>

namespace temoto_component_manager
{

/**
 * @brief Class that maintains and handles the component info objects
 */
class ComponentInfoRegistry : public temoto_core::BaseSubsystem
{
public:

  struct ComponentInfoPtrs
  {
    std::vector<ComponentInfoPtr>& components;
  };

  ComponentInfoRegistry(temoto_core::BaseSubsystem* b);

  bool findLocalComponents( temoto_component_manager::LoadComponent::Request& req, std::vector<ComponentInfo>& ci_ret ) const;

  bool findLocalComponent( const ComponentInfo& ci, ComponentInfo& ci_ret ) const;

  bool findLocalComponent( const std::string& component_name, ComponentInfo& ci_ret ) const;

  bool findLocalComponent( const ComponentInfo& ci ) const;

  bool findRemoteComponents( temoto_component_manager::LoadComponent::Request& req, std::vector<ComponentInfo>& ci_ret ) const;

  bool findRemoteComponent( const ComponentInfo& ci, ComponentInfo& ci_ret ) const;

  bool findRemoteComponent( const ComponentInfo& ci ) const;

  bool addLocalComponent( const ComponentInfo& ci );

  bool addRemoteComponent( const ComponentInfo& ci );

  bool updateLocalComponent(const ComponentInfo& ci, bool advertised = false);

  bool updateRemoteComponent(const ComponentInfo& ci, bool advertised = false);

  const std::vector<ComponentInfo>& getLocalComponents() const;

  const std::vector<ComponentInfo>& getRemoteComponents() const;

  const std::map<std::string, PipeInfos>& getPipes() const;

  bool findPipes( const LoadPipe::Request& req, PipeInfos& pipes_ret ) const;

  bool addPipe( const PipeInfo& pi);

  bool updatePipe( const PipeInfo& pi );

  /**
   * @brief Adds an external callback function that will be invoked if a component gets added or updated 
   * 
   * @param cir_update_callback 
   */
  void registerUpdateCallback( std::function<void(ComponentInfo)> cir_update_callback);

  /**
   * @brief Calls all registered cir update callbacks
   * 
   * @param ci Copy of the added/updated component 
   * @return true if all callbacks were invoked successfully
   * @return false if a callback failed
   */
  bool callUpdateCallbacks(ComponentInfo ci);

  /**
   * @brief Destroy the Component Info Registry object
   * 
   */
  ~ComponentInfoRegistry();

private:

  /**
   * @brief 
   * 
   * @param l_topics 
   * @param r_topics 
   * @return true if topics are the same
   * @return false if topics are different
   */
  bool compareTopics( const std::vector<temoto_core::StringPair>& l_topics
                    , const std::vector<diagnostic_msgs::KeyValue>& r_topics ) const;

  /**
   * @brief Returns a vector of components that follow the requested criteria
   * 
   * @param req Requested component
   * @param components Vector of known components
   * @param ci_ret Vector of found components
   * @return true Found component(s)
   * @return false Did not find any components
   */
  bool findComponents( temoto_component_manager::LoadComponent::Request& req
                     , const std::vector<ComponentInfo>& components
                     , std::vector<ComponentInfo>& ci_ret ) const;

  /**
   * @brief Returns one component that matches the requested criteria the most
   * 
   * @param ci Requested component
   * @param components Vector of known components
   * @param ci_ret Found component
   * @return true If found a component
   * @return false if component was not found
   */
  bool findComponent( const ComponentInfo& ci
                    , const std::vector<ComponentInfo>& components
                    , ComponentInfo& ci_ret ) const;

  /**
   * @brief 
   * 
   * @param pi 
   * @param pi_ret 
   * @return PipeInfo* 
   */
  PipeInfo* findPipe ( const PipeInfo& pi );

  void updateCallbackCleanupLoop();

  /// Update callback
  std::vector<std::function<void(ComponentInfo)>> cir_update_callbacks_;

  /// Threads where the callbacks are run
  std::vector<std::thread> cir_update_callback_threads_;

  mutable std::recursive_mutex cir_cb_update_mutex_;

  std::thread update_callback_cleanup_thread_;

  bool stop_cleanup_loop_ = false;

  /// List of all locally defined components.
  std::vector<ComponentInfo> local_components_;

  /// List of all components in remote managers.
  std::vector<ComponentInfo> remote_components_;

  /// List of categorized pipes
  std::map<std::string, PipeInfos> categorized_pipes_;

  /// Responsible for grenerating uniquie pipe_info ids
  temoto_core::temoto_id::IDManager pipe_info_id_manager_;

  /// Mutex for protecting component info vectors from data races
  mutable std::recursive_mutex read_write_mutex;

  /// Mutex for protecting pipe info vectors from data races
  mutable std::recursive_mutex read_write_mutex_pipe_;
};

} // component_manager namespace

#endif
