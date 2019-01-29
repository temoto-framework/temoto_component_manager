#ifndef TEMOTO_COMPONENT_MANAGER__COMPONENT_MANAGER_YAML_H
#define TEMOTO_COMPONENT_MANAGER__COMPONENT_MANAGER_YAML_H

#include <yaml-cpp/yaml.h>
#include "temoto_component_manager/component_info.h"

namespace temoto_component_manager
{
namespace sm_yaml
{
  std::vector<ComponentInfoPtr> parseComponents(const YAML::Node& config);
} // sm_yaml namespace
} // component_manager namespace

#endif