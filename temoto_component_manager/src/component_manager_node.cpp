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
  {}

  bool initialize()
  {
    try
    {
      cs_.startSnooping();
      TEMOTO_INFO("Component Manager is good to go.");
      return true;
    }
    catch(temoto_core::error::ErrorStack e)
    {
      TEMOTO_ERROR_STREAM("Could not start the Component Manager: ");
      std::cout << e << std::endl;
      return false;
    }
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
  ComponentManager cm;
  if (!cm.initialize())
  {
    return 1;
  }

  //use single threaded spinner for global callback queue
  ros::spin();

  return 0;
}
