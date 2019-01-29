#include "temoto_core/common/base_subsystem.h"
#include "temoto_component_manager/component_manager_services.h"
#include "temoto_component_manager/component_manager_servers.h"
#include "temoto_component_manager/component_info_registry.h"
#include "temoto_component_manager/component_snooper.h"

#include <signal.h>

using namespace temoto_component_manager;
using namespace temoto_core;

/**
 * @brief The Component Manager maintains 3 components of this subsystem
 */
class ComponentManager : public BaseSubsystem
{
public:

  /**
   * @brief Constructor
   */
  ComponentManager()
  : BaseSubsystem("component_manager", error::Subsystem::COMPONENT_MANAGER, __func__)
  , cs_(this, &cir_)
  , cms_(this, &cir_)
  {
    cs_.startSnooping();
    TEMOTO_INFO("Component Manager is good to go.");
  }

  ~ComponentManager()
  {
    // "cout" instead of "TEMOTO_INFO" because otherwise it will print nothing
    std::cout << "Shutting down the Component Manager ..." << std::endl;
  }

private:

  /// Component Info Database
  ComponentInfoRegistry cir_;

  /// Component Manager Servers
  ComponentManagerServers cms_;

  /// Component Snooper
  ComponentSnooper cs_;
};

/*
 * Main
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "component_manager");

  // Create a ComponentManager object
  ComponentManager sm;

  //use single threaded spinner for global callback queue
  ros::spin();

  return 0;
}
