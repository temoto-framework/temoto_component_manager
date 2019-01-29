#include "temoto_component_manager/component_info_registry.h"
#include <algorithm>

namespace temoto_component_manager
{
using namespace temoto_core;

ComponentInfoRegistry::ComponentInfoRegistry(){}

bool ComponentInfoRegistry::addLocalComponent(const ComponentInfo& si)
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  // Check if there is no such component
  ComponentInfo si_ret;
  if (!findComponent(si, local_components_, si_ret))
  {
    local_components_.push_back(si);
    return true;
  }

  // Return false if such component already exists
  return false;
}

bool ComponentInfoRegistry::addRemoteComponent(const ComponentInfo &si)
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  // Check if there is no such component
  ComponentInfo si_ret;
  if (!findComponent(si, remote_components_, si_ret))
  {
    remote_components_.push_back(si);
    return true;
  }

  // Return false if such component already exists
  return false;
}

bool ComponentInfoRegistry::updateLocalComponent(const ComponentInfo &si, bool advertised)
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  const auto it = std::find_if( local_components_.begin()
                              , local_components_.end()
                              , [&](const ComponentInfo& ls)
                              {
                                return ls == si;
                              });

  // Update the local component if its found
  if (it != local_components_.end())
  {
    *it = si;
    it->setAdvertised( advertised );
    return true;
  }

  // Return false if no such component was found
  return false;
}

bool ComponentInfoRegistry::updateRemoteComponent(const ComponentInfo &si, bool advertised)
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  const auto it = std::find_if( remote_components_.begin()
                              , remote_components_.end()
                              , [&](const ComponentInfo& rs)
                              {
                                return rs == si;
                              });

  // Update the local component if its found
  if (it != remote_components_.end())
  {
    *it = si;
    it->setAdvertised( advertised );
    return true;
  }

  // Return false if no such component was found
  return false;
}

bool ComponentInfoRegistry::findLocalComponents( LoadComponent::Request& req
                                         , std::vector<ComponentInfo>& si_ret ) const
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  return findComponents(req, local_components_, si_ret);
}

bool ComponentInfoRegistry::findLocalComponent( const ComponentInfo &si, ComponentInfo& si_ret ) const
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  return findComponent(si, local_components_, si_ret);
}

bool ComponentInfoRegistry::findLocalComponent( const ComponentInfo &si ) const
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  ComponentInfo si_ret;
  return findComponent(si, local_components_, si_ret);
}

bool ComponentInfoRegistry::findRemoteComponents( LoadComponent::Request& req
                                          , std::vector<ComponentInfo>& si_ret ) const
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  return findComponents(req, remote_components_, si_ret);
}

bool ComponentInfoRegistry::findRemoteComponent( const ComponentInfo &si, ComponentInfo& si_ret ) const
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  return findComponent(si, remote_components_, si_ret);
}

bool ComponentInfoRegistry::findRemoteComponent( const ComponentInfo &si ) const
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  ComponentInfo si_ret;
  return findComponent(si, remote_components_, si_ret);
}

bool ComponentInfoRegistry::findComponents( LoadComponent::Request& req
                                    , const std::vector<ComponentInfo>& components
                                    , std::vector<ComponentInfo>& si_ret ) const
{
  // Local list of devices that follow the requirements
  std::vector<ComponentInfo> candidates;

  // Find the devices that follow the "type" criteria
  auto it = std::copy_if(components.begin()
                       , components.end()
                       , std::back_inserter(candidates)
                       , [&](const ComponentInfo& s)
                         {
                           return s.getType() == req.component_type;
                         });

  // The requested type of component is not available
  if (candidates.empty())
  {
    return false;
  }

  // If package_name is specified, remove all non-matching candidates
  auto it_end = candidates.end();
  if (req.package_name != "")
  {
    it_end = std::remove_if(candidates.begin(), candidates.end(),
                            [&](ComponentInfo s)
                            {
                              return s.getPackageName() != req.package_name;
                            });
  }

  // If executable is specified, remove all non-matching candidates
  if (req.executable != "")
  {
    it_end = std::remove_if(candidates.begin(), it_end,
                            [&](ComponentInfo s)
                            {
                              return s.getExecutable() != req.executable;
                            });
  }

  // If output topics are specified ...
  if (!req.output_topics.empty())
  {
    it_end = std::remove_if(candidates.begin(), it_end,
                            [&](ComponentInfo s)
                            {
                              if (s.getOutputTopics().size() < req.output_topics.size())
                                return true;

                              // Make a copy of the input topics
                              std::vector<StringPair> output_topics_copy = s.getOutputTopics();

                              // Start looking for the requested topic types
                              for (auto& topic : req.output_topics)
                              {
                                bool found = false;
                                for (auto it=output_topics_copy.begin(); it != output_topics_copy.end(); it++)
                                {
                                  // If the topic was found then remove it from the copy list
                                  if (topic.key == it->first)
                                  {
                                    found = true;
                                    output_topics_copy.erase(it);
                                    break;
                                  }
                                }

                                // If this topic type was not found then return with false
                                if (!found)
                                {
                                  return true;
                                }
                              }

                              return false;
                            });
  }

  // Sort remaining candidates based on their reliability.
  std::sort( candidates.begin()
           , it_end
           , [](ComponentInfo& s1, ComponentInfo& s2)
             {
               return s1.getReliability() > s2.getReliability();
             });

  if (candidates.begin() == it_end)
  {
    // Component with the requested criteria was not found.
    return false;
  }

  // Return the first component of the requested type.
  si_ret = candidates;
  return true;
}

bool ComponentInfoRegistry::findComponent( const ComponentInfo &si
                                   , const std::vector<ComponentInfo>& components
                                   , ComponentInfo& si_ret ) const
{

  const auto it = std::find_if( components.begin()
                              , components.end()
                              , [&](const ComponentInfo& rs)
                              {
                                return rs == si;
                              });


  if (it == components.end())
  {
    return false;
  }
  else
  {
    si_ret = *it;
    return true;
  }
}

const std::vector<ComponentInfo>& ComponentInfoRegistry::getLocalComponents() const
{
  return local_components_;
}

const std::vector<ComponentInfo>& ComponentInfoRegistry::getRemoteComponents() const
{
  return remote_components_;
}

} // component_manager namespace
