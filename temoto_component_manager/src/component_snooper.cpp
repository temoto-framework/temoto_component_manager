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

#include "temoto_component_manager/component_snooper.h"
#include "temoto_component_manager/component_manager_services.h"

#include "ros/package.h"
#include "yaml-cpp/yaml.h"


namespace temoto_component_manager
{
using namespace temoto_core;

// TODO: the constructor of the action_engine_ can throw in the initializer list
//       and I have no clue what kind of behaviour should be expected - prolly bad

ComponentSnooper::ComponentSnooper( temoto_core::BaseSubsystem*b
, ComponentInfoRegistry* cir)
: temoto_core::BaseSubsystem(*b, __func__)
, config_syncer_(srv_name::MANAGER, srv_name::SYNC_TOPIC, &ComponentSnooper::syncCb, this)
, action_engine_()
, cir_(cir)
{
  // Set up the action engine
  std::string action_uri_file_path = ros::package::getPath(ROS_PACKAGE_NAME) + "/config/action_dst.yaml";

  /*
   * Open the action sources yaml file and get the paths to action libs
   * TODO: check for typos and other existance problems
   */
  YAML::Node config = YAML::LoadFile(action_uri_file_path);
  TEMOTO_INFO_STREAM("Indexing TeMoto actions");
  for (YAML::const_iterator it = config.begin(); it != config.end(); ++it)
  {
    std::string package_name = (*it)["package_name"].as<std::string>();
    std::string relative_path = (*it)["relative_path"].as<std::string>();
    std::string full_path = ros::package::getPath(package_name) + "/" + relative_path;
    action_engine_.addActionsPath(full_path);
  }

  // Start the Action Engine
  action_engine_.start();

  // Component Info update monitoring timer
  update_monitoring_timer_ = nh_.createTimer(ros::Duration(1), &ComponentSnooper::updateMonitoringTimerCb, this);

  // Get remote component_infos
  config_syncer_.requestRemoteConfigs();

  // Advertise local components
  advertiseLocalComponents();
}

void ComponentSnooper::startSnooping()
try
{
  // Invoke the component finder action
  {
    UmrfNode find_components_umrf;
    find_components_umrf.setName("TaFindComponentPackages");
    find_components_umrf.setSuffix(0);
    find_components_umrf.setEffect("synchronous");

    ActionParameters ap;
    ap.setParameter("catkin_ws_path", "string", boost::any_cast<std::string>(ros::package::getPath(ROS_PACKAGE_NAME) + "/../../.."));
    ap.setParameter("cir", "cir_pointer", boost::any_cast<ComponentInfoRegistry*>(cir_));

    find_components_umrf.setInputParameters(ap);
    UmrfGraph ug("component_snooper_graph_1", std::vector<UmrfNode>{find_components_umrf});
    action_engine_.executeUmrfGraph(ug, true);
  }

  // Invoke the pipe finding action
  {
    UmrfNode find_pipes_umrf;
    find_pipes_umrf.setName("TaFindComponentPipes");
    find_pipes_umrf.setSuffix(0);
    find_pipes_umrf.setEffect("synchronous");

    ActionParameters ap;
    ap.setParameter("catkin_ws_path", "string", boost::any_cast<std::string>(ros::package::getPath(ROS_PACKAGE_NAME) + "/../../.."));
    ap.setParameter("cir", "cir_pointer", boost::any_cast<ComponentInfoRegistry*>(cir_));

    find_pipes_umrf.setInputParameters(ap);
    UmrfGraph ug("component_snooper_graph_2", std::vector<UmrfNode>{find_pipes_umrf});
    action_engine_.executeUmrfGraph(ug, true);
  }
}
catch(const std::exception& e)
{
  throw CREATE_ERROR(temoto_core::error::Code::ACTION_UNKNOWN, e.what());
}
catch(...)
{
  throw CREATE_ERROR(temoto_core::error::Code::ACTION_UNKNOWN, "Unhandled exception");
}

void ComponentSnooper::advertiseComponent(ComponentInfo& si) const
{
  //TEMOTO_DEBUG("------ Advertising Component \n %s", component_ptr->toString().c_str());
  YAML::Node config;
  config["Components"].push_back(si);
  PayloadType payload;
  payload.data = Dump(config);
  config_syncer_.advertise(payload);
}

void ComponentSnooper::advertiseLocalComponents() const
{
  // publish all local components
  YAML::Node config;
  for(const auto& s : cir_->getLocalComponents())
  {
    config["Components"].push_back(s);
  }

  // send to other managers if there is anything to send
  if(config.size())
  {
    PayloadType payload;
    payload.data = Dump(config);
    config_syncer_.advertise(payload);
  }
}

std::vector<ComponentInfoPtr> ComponentSnooper::parseComponents(const YAML::Node& config)
{
  std::vector<ComponentInfoPtr> components;

  if (!config.IsMap())
  {
    TEMOTO_WARN("Unable to parse 'Components' key from config.");
    return components;
  }

  YAML::Node components_node = config["Components"];
  if (!components_node.IsSequence())
  {
    TEMOTO_WARN("The given config does not contain sequence of components.");
    return components;
  }

  TEMOTO_DEBUG("Parsing %lu components.", components_node.size());

  // go over each component node in the sequence
  for (YAML::const_iterator node_it = components_node.begin(); node_it != components_node.end(); ++node_it)
  {
    if (!node_it->IsMap())
    {
      TEMOTO_WARN("Unable to parse the component. Parameters in YAML have to be specified in "
                   "key-value pairs.");
      continue;
    }

    try
    {
      ComponentInfo component = node_it->as<ComponentInfo>();
      if (std::count_if(components.begin(), components.end(),
                        [&](const ComponentInfoPtr& s) { return *s == component; }) == 0)
      {
        // OK, this is unique pointer, add it to the components vector.
        components.emplace_back(std::make_shared<ComponentInfo>(component));
        //TEMOTO_DEBUG_STREAM("####### PARSED COMPONENT: #######\n" << components.back()->toString());
      }
      else
      {
        TEMOTO_WARN("Ignoring duplicate of component '%s'.", component.getName().c_str());
      }
    }
    catch (YAML::TypedBadConversion<ComponentInfo> e)
    {
      TEMOTO_WARN("Failed to parse ComponentInfo from config.");
      continue;
    }
  }
  return components;
}

void ComponentSnooper::syncCb(const temoto_core::ConfigSync& msg, const PayloadType& payload)
{

  if (msg.action == trr::sync_action::REQUEST_CONFIG)
  {
    std::cout << "Received a request to advertise local components" << std::endl;
    advertiseLocalComponents();
    return;
  }

  if (msg.action == trr::sync_action::ADVERTISE_CONFIG)
  {
    std::cout << "Received a request to add or update remote components" << std::endl;

    // Convert the config string to YAML tree and parse
    YAML::Node config = YAML::Load(payload.data);
    std::vector<ComponentInfoPtr> components = parseComponents(config);

    // TODO: Hold remote stuff in a map or something keyed by namespace
    // TODO: Temoto namespace can (doesn't have to) be contained in config
    for (auto& s : components)
    {
      s->setTemotoNamespace(msg.temoto_namespace);
      TEMOTO_WARN("---------REMOTE COMPONENT: \n %s", s->toString().c_str());
    }

    for (auto& component : components)
    {
      // Check if component has to be added or updated
      if (cir_->updateRemoteComponent(*component))
      {
        TEMOTO_DEBUG("Updating remote component '%s' at '%s'.", component->getName().c_str(),
                     component->getTemotoNamespace().c_str());
      }
      else
      {
        cir_->addRemoteComponent(*component);
      }
    }
  }
}

void ComponentSnooper::updateMonitoringTimerCb(const ros::TimerEvent& e)
{
  (void)e; // Suppress "unused variable" compiler warnings

  // Iterate through local components and check if their reliability has been updated
  for (const auto component : cir_->getLocalComponents())
  {
    if (!component.getAdvertised())
    {
      ComponentInfo si = component;
      cir_->updateLocalComponent(si, true);
      advertiseComponent(si);
    }
  }
}

ComponentSnooper::~ComponentSnooper()
{
  TEMOTO_INFO("in the destructor of Component Snooper");
}

} // component_manager namespace
