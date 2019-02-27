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

  // TODO: Unused service, should be removed
  bool listComponentsCb(temoto_component_manager::ListComponents::Request& req, temoto_component_manager::ListComponents::Response& res);

  /**
   * @brief Callback to a service that executes/runs a requested device
   * and sends back the topic that the device is publishing to
   * @param req
   * @param res
   */
  void loadComponentCb(temoto_component_manager::LoadComponent::Request& req, temoto_component_manager::LoadComponent::Response& res);

  /**
   * @brief Called when a component is unloaded.
   * @param req
   * @param res
   */
  void unloadComponentCb(temoto_component_manager::LoadComponent::Request& req, temoto_component_manager::LoadComponent::Response& res);

  /**
   * @brief Called when component status update information is received.
   * @param srv
   */
  void statusCb(temoto_core::ResourceStatus& srv);

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

  /**
   * @brief Checks if given component is already in use
   * 
   * @param ci_to_check 
   * @return temoto_core::temoto_id::ID 
   */
  temoto_core::temoto_id::ID checkIfInUse( const std::vector<ComponentInfo>& cis_to_check) const;

  /// Pointer to a central Component Info Registry object.
  ComponentInfoRegistry* cir_;

  /// Resource Management object which handles resource requests and status info propagation.
  temoto_core::rmp::ResourceManager<ComponentManagerServers> resource_manager_;

  /// List of allocated components.
  std::map<temoto_core::temoto_id::ID, ComponentInfo> allocated_components_;
  std::map<temoto_core::temoto_id::ID, temoto_er_manager::LoadExtResource> allocated_ext_resources_;

}; // ComponentManagerServers

} // component_manager namespace

#endif
