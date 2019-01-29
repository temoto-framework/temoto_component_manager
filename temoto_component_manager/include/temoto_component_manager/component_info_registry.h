#ifndef TEMOTO_COMPONENT_MANAGER__COMPONENT_INFO_REGISTRY_H
#define TEMOTO_COMPONENT_MANAGER__COMPONENT_INFO_REGISTRY_H

#include "temoto_component_manager/component_info.h"
#include "temoto_component_manager/LoadComponent.h"

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

private:

  /**
   * @brief findComponent
   * @param req
   * @param components
   * @return
   */
  bool findComponents( temoto_component_manager::LoadComponent::Request& req
                  , const std::vector<ComponentInfo>& components
                  , std::vector<ComponentInfo>& si_ret ) const;

  bool findComponent( const ComponentInfo& si
                 , const std::vector<ComponentInfo>& components
                 , ComponentInfo& si_ret ) const;

  /// List of all locally defined components.
  std::vector<ComponentInfo> local_components_;

  /// List of all components in remote managers.
  std::vector<ComponentInfo> remote_components_;

  /// Mutex for protecting component info vectors from data races
  mutable std::mutex read_write_mutex;
};

} // component_manager namespace

#endif
