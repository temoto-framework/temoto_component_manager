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

#ifndef TEMOTO_COMPONENT_MANAGER__COMPONENT_INFO_REGISTRY_H
#define TEMOTO_COMPONENT_MANAGER__COMPONENT_INFO_REGISTRY_H

#include "temoto_component_manager/component_info.h"
#include "temoto_component_manager/pipe_info.h"
#include "temoto_component_manager/LoadComponent.h"
#include "temoto_component_manager/LoadPipe.h"
#include "temoto_core/common/temoto_id.h"

#include <mutex>

namespace temoto_component_manager
{

/**
 * @brief Class that maintains and handles the component info objects
 */
class ComponentInfoRegistry
{
public:

  struct ComponentInfoPtrs
  {
    std::vector<ComponentInfoPtr>& components;
  };

  ComponentInfoRegistry();

  bool findLocalComponents( temoto_component_manager::LoadComponent::Request& req, std::vector<ComponentInfo>& si_ret ) const;

  bool findLocalComponent( const ComponentInfo& si, ComponentInfo& si_ret ) const;

  bool findLocalComponent( const ComponentInfo& si ) const;

  bool findRemoteComponents( temoto_component_manager::LoadComponent::Request& req, std::vector<ComponentInfo>& si_ret ) const;

  bool findRemoteComponent( const ComponentInfo& si, ComponentInfo& si_ret ) const;

  bool findRemoteComponent( const ComponentInfo& si ) const;

  bool addLocalComponent( const ComponentInfo& si );

  bool addRemoteComponent( const ComponentInfo& si );

  bool updateLocalComponent(const ComponentInfo& si, bool advertised = false);

  bool updateRemoteComponent(const ComponentInfo& si, bool advertised = false);

  const std::vector<ComponentInfo>& getLocalComponents() const;

  const std::vector<ComponentInfo>& getRemoteComponents() const;

  const std::map<std::string, PipeInfos>& getPipes() const;

  bool findPipes( const LoadPipe::Request& req, PipeInfos& pipes_ret ) const;

  bool addPipe( const PipeInfo& pi);

  bool updatePipe( const PipeInfo& pi );

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
   * @param si_ret Vector of found components
   * @return true Found component(s)
   * @return false Did not find any components
   */
  bool findComponents( temoto_component_manager::LoadComponent::Request& req
                     , const std::vector<ComponentInfo>& components
                     , std::vector<ComponentInfo>& si_ret ) const;

  /**
   * @brief Returns one component that matches the requested criteria the most
   * 
   * @param si Requested component
   * @param components Vector of known components
   * @param si_ret Found component
   * @return true If found a component
   * @return false if component was not found
   */
  bool findComponent( const ComponentInfo& si
                    , const std::vector<ComponentInfo>& components
                    , ComponentInfo& si_ret ) const;

  /**
   * @brief 
   * 
   * @param pi 
   * @param pi_ret 
   * @return PipeInfo* 
   */
  PipeInfo* findPipe ( const PipeInfo& pi );

  /// List of all locally defined components.
  std::vector<ComponentInfo> local_components_;

  /// List of all components in remote managers.
  std::vector<ComponentInfo> remote_components_;

  /// List of categorized pipes
  std::map<std::string, PipeInfos> categorized_pipes_;

  /// Responsible for grenerating uniquie pipe_info ids
  temoto_core::temoto_id::IDManager pipe_info_id_manager_;

  /// Mutex for protecting component info vectors from data races
  mutable std::mutex read_write_mutex;

  /// Mutex for protecting pipe info vectors from data races
  mutable std::mutex read_write_mutex_pipe_;
};

} // component_manager namespace

#endif
