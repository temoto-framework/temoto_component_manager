#include "temoto_component_manager/component_snooper.h"
#include "temoto_component_manager/component_manager_services.h"
#include "temoto_component_manager/component_manager_yaml.h"

#include "ros/package.h"
#include <yaml-cpp/yaml.h>


namespace temoto_component_manager
{
using namespace temoto_core;

// TODO: the constructor of the action_engine_ can throw in the initializer list
//       and I have no clue what kind of behaviour should be expected - prolly bad

ComponentSnooper::ComponentSnooper( temoto_core::BaseSubsystem*b
                            , ComponentInfoRegistry* cir)
: temoto_core::BaseSubsystem(*b, __func__)
, config_syncer_(srv_name::MANAGER, srv_name::SYNC_TOPIC, &ComponentSnooper::syncCb, this)
, action_engine_(this, false, ros::package::getPath(ROS_PACKAGE_NAME) + "/config/action_dst.yaml")
, cir_(cir)
{
  // Component Info update monitoring timer
  update_monitoring_timer_ = nh_.createTimer(ros::Duration(1), &ComponentSnooper::updateMonitoringTimerCb, this);

  // Get remote component_infos
  config_syncer_.requestRemoteConfigs();
}

void ComponentSnooper::startSnooping()
{
  /*
   * Action related stuff up ahead: A semantic frame is manually created. Based on that SF
   * a SF tree is created, given that an action implementation, that corresponds to the
   * manually created SF, exists. The specific tracker task is started and it continues
   * running in the background until its ordered to be stopped.
   */

  // Invoke the component finder action
  {
    std::string action = "find";
    temoto_nlp::Subjects subjects;

    // Subject that will contain the name of the tracked object.
    // Necessary when the tracker has to be stopped
    temoto_nlp::Subject sub_0("what", "component packages");

    // Topic from where the raw AR tag tracker data comes from
    std::string catkin_ws = ros::package::getPath(ROS_PACKAGE_NAME) + "/../../..";
    sub_0.addData("string", catkin_ws);

    // Pass a pointer of CIR to the action. This will be used for updating CIR.
    sub_0.addData("pointer", boost::any_cast<ComponentInfoRegistry*>(cir_));

    subjects.push_back(sub_0);

    // Create a SF
    std::vector<temoto_nlp::TaskDescriptor> task_descriptors;
    task_descriptors.emplace_back(action, subjects);
    task_descriptors[0].setActionStemmed(action);

    // Create a sematic frame tree
    temoto_nlp::TaskTree sft = temoto_nlp::SFTBuilder::build(task_descriptors);

    // Execute the SFT
    action_engine_.executeSFTThreaded(std::move(sft));
  }

  // Invoke the pipe finding action
  {
    std::string action = "find";
    temoto_nlp::Subjects subjects;

    // Subject that will contain the name of the tracked object.
    // Necessary when the tracker has to be stopped
    temoto_nlp::Subject sub_0("what", "pipes");

    // Topic from where the raw AR tag tracker data comes from
    std::string catkin_ws = ros::package::getPath(ROS_PACKAGE_NAME) + "/../../..";
    sub_0.addData("string", catkin_ws);

    // Pass a pointer of CIR to the action. This will be used for updating CIR.
    sub_0.addData("pointer", boost::any_cast<ComponentInfoRegistry*>(cir_));

    subjects.push_back(sub_0);

    // Create a SF
    std::vector<temoto_nlp::TaskDescriptor> task_descriptors;
    task_descriptors.emplace_back(action, subjects);
    task_descriptors[0].setActionStemmed(action);

    // Create a sematic frame tree
    temoto_nlp::TaskTree sft = temoto_nlp::SFTBuilder::build(task_descriptors);

    // Execute the SFT
    action_engine_.executeSFTThreaded(std::move(sft));
  }
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

  if (msg.action == rmp::sync_action::REQUEST_CONFIG)
  {
    std::cout << "Received a request to advertise local components" << std::endl;
    advertiseLocalComponents();
    return;
  }

  if (msg.action == rmp::sync_action::ADVERTISE_CONFIG)
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
    }

    //for (auto& s : remote_components_)
    //{
    //  TEMOTO_DEBUG("---------REMOTE COMPONENT: \n %s", s->toString().c_str());
    //}

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
