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

#include "temoto_component_manager/component_info_registry.h"
#include <algorithm>

namespace temoto_component_manager
{
using namespace temoto_core;

ComponentInfoRegistry::ComponentInfoRegistry(temoto_core::BaseSubsystem* b)
: temoto_core::BaseSubsystem(*b, __func__)
{
  // Start the component update cleanup loop thread
  update_callback_cleanup_thread_ = std::thread(&ComponentInfoRegistry::updateCallbackCleanupLoop, this);
}

void ComponentInfoRegistry::registerUpdateCallback( std::function<void(ComponentInfo)> cir_update_callback)
{
  // TODO: Check if the callback is unique
  cir_update_callbacks_.push_back(cir_update_callback);
}

bool ComponentInfoRegistry::callUpdateCallbacks(ComponentInfo ci)
{
  bool all_cbs_invoked_successfully = true;
  for (const auto& cir_update_callback : cir_update_callbacks_)
  {
    if(!cir_update_callback)
    {
      all_cbs_invoked_successfully = false;
      continue;
    }
    try
    {
      std::lock_guard<std::recursive_mutex> guard(cir_cb_update_mutex_);
      TEMOTO_DEBUG_STREAM("Invoking an update callback in a separate thread ...");
      cir_update_callback_threads_.emplace_back(cir_update_callback, ci);
    }
    catch(const std::exception& e)
    {
      all_cbs_invoked_successfully = false;
      continue;
    }
    catch(...)
    {
      all_cbs_invoked_successfully = false;
      continue;
    }
  }
  return all_cbs_invoked_successfully;
}

void ComponentInfoRegistry::updateCallbackCleanupLoop()
{
  while(true)
  {
    // Sleep for 1 second
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::lock_guard<std::recursive_mutex> guard(cir_cb_update_mutex_);
    if (stop_cleanup_loop_ /*&& cir_update_callback_threads_.empty()*/)
    {
      break;
    }

    for (auto it=cir_update_callback_threads_.begin();
         it!=cir_update_callback_threads_.end();
         /* empty */)
    {
      if (it->joinable())
      {
        TEMOTO_DEBUG_STREAM("An update callback has finished, joining the thread.");
        it->join();
        cir_update_callback_threads_.erase(it);
      }
      else
      {
        it++;
      }
    }
  }
}

bool ComponentInfoRegistry::addLocalComponent(const ComponentInfo& ci)
{
  // Lock the mutex
  std::lock_guard<std::recursive_mutex> guard(read_write_mutex);

  // Check if there is no such component
  ComponentInfo ci_ret;
  if (!findComponent(ci, local_components_, ci_ret))
  {
    local_components_.push_back(ci);

    // Trigger the cir update callback
    callUpdateCallbacks(ci);

    return true;
  }

  // Return false if such component already exists
  return false;
}

bool ComponentInfoRegistry::addRemoteComponent(const ComponentInfo &ci)
{
  // Lock the mutex
  std::lock_guard<std::recursive_mutex> guard(read_write_mutex);

  // Check if there is no such component
  ComponentInfo ci_ret;
  if (!findComponent(ci, remote_components_, ci_ret))
  {
    remote_components_.push_back(ci);
    return true;
  }

  // Return false if such component already exists
  return false;
}

bool ComponentInfoRegistry::updateLocalComponent(const ComponentInfo &ci, bool advertised)
{
  // Lock the mutex
  std::lock_guard<std::recursive_mutex> guard(read_write_mutex);

  const auto it = std::find_if( local_components_.begin()
                              , local_components_.end()
                              , [&](const ComponentInfo& ls)
                              {
                                return ls == ci;
                              });

  // Update the local component if its found
  if (it != local_components_.end())
  {
    *it = ci;
    it->setAdvertised( advertised );
    return true;
  }

  // Return false if no such component was found
  return false;
}

bool ComponentInfoRegistry::updateRemoteComponent(const ComponentInfo &ci, bool advertised)
{
  // Lock the mutex
  std::lock_guard<std::recursive_mutex> guard(read_write_mutex);

  const auto it = std::find_if( remote_components_.begin()
                              , remote_components_.end()
                              , [&](const ComponentInfo& rs)
                              {
                                return rs == ci;
                              });

  // Update the local component if its found
  if (it != remote_components_.end())
  {
    *it = ci;
    it->setAdvertised( advertised );
    return true;
  }

  // Return false if no such component was found
  return false;
}

bool ComponentInfoRegistry::findLocalComponents( LoadComponent::Request& req
                                         , std::vector<ComponentInfo>& ci_ret ) const
{
  // Lock the mutex
  std::lock_guard<std::recursive_mutex> guard(read_write_mutex);

  return findComponents(req, local_components_, ci_ret);
}

bool ComponentInfoRegistry::findLocalComponent( const ComponentInfo &ci, ComponentInfo& ci_ret ) const
{
  // Lock the mutex
  std::lock_guard<std::recursive_mutex> guard(read_write_mutex);

  return findComponent(ci, local_components_, ci_ret);
}

bool ComponentInfoRegistry::findLocalComponent( const std::string& component_name, ComponentInfo& ci_ret ) const
{
  std::lock_guard<std::recursive_mutex> guard(read_write_mutex);
  auto component_it = std::find_if(local_components_.begin()
  , local_components_.end()
  , [&component_name](const ComponentInfo& s)
    {
      return s.getName() == component_name;
    });

  if (component_it != local_components_.end())
  {
    ci_ret = *component_it;
    return true;
  }
  else
  {
    return false;
  }
}

bool ComponentInfoRegistry::findLocalComponent( const ComponentInfo &ci ) const
{
  // Lock the mutex
  std::lock_guard<std::recursive_mutex> guard(read_write_mutex);

  ComponentInfo ci_ret;
  return findComponent(ci, local_components_, ci_ret);
}

bool ComponentInfoRegistry::findRemoteComponents( LoadComponent::Request& req
                                          , std::vector<ComponentInfo>& ci_ret ) const
{
  // Lock the mutex
  std::lock_guard<std::recursive_mutex> guard(read_write_mutex);

  return findComponents(req, remote_components_, ci_ret);
}

bool ComponentInfoRegistry::findRemoteComponent( const ComponentInfo &ci, ComponentInfo& ci_ret ) const
{
  // Lock the mutex
  std::lock_guard<std::recursive_mutex> guard(read_write_mutex);

  return findComponent(ci, remote_components_, ci_ret);
}

bool ComponentInfoRegistry::findRemoteComponent( const ComponentInfo &ci ) const
{
  // Lock the mutex
  std::lock_guard<std::recursive_mutex> guard(read_write_mutex);

  ComponentInfo ci_ret;
  return findComponent(ci, remote_components_, ci_ret);
}

bool ComponentInfoRegistry::compareTopics( const std::vector<temoto_core::StringPair>& l_topics
                                         , const std::vector<diagnostic_msgs::KeyValue>& r_topics) const
{
  if (l_topics.size() < r_topics.size())
  { 
    return true;
  }

  // Make a copy of the input topics
  std::vector<StringPair> l_topics_copy = l_topics;

  // Start looking for the requested topic types
  for (const auto& r_topic : r_topics)
  {
    bool found = false;
    for (auto it=l_topics_copy.begin(); it != l_topics_copy.end(); it++)
    {
      // If the topic was found then remove it from the copy list
      if (r_topic.key == it->first)
      {
        found = true;
        l_topics_copy.erase(it);
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
}

bool ComponentInfoRegistry::findComponents( LoadComponent::Request& req
                                    , const std::vector<ComponentInfo>& components
                                    , std::vector<ComponentInfo>& ci_ret ) const
{
  // Local list of devices that follow the requirements
  std::vector<ComponentInfo> candidates;

  // Check if the name of the component is required
  if (!req.component_name.empty())
  {
    std::copy_if( components.begin()
                , components.end()
                , std::back_inserter(candidates)
                , [&](const ComponentInfo& s)
                  {
                    return s.getName() == req.component_name;
                  });

    if (candidates.empty())
    {
      return false;
    }
  }

  // Find the devices that follow the "type" criteria
  std::copy_if( components.begin()
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

  // If input topics are specified ...
  if (!req.input_topics.empty())
  {
    it_end = std::remove_if(candidates.begin(), it_end,
                            [&](ComponentInfo s)
                            {
                              return compareTopics(s.getInputTopics(), req.input_topics);
                            });
  }

  // If output topics are specified ...
  if (!req.output_topics.empty())
  {
    it_end = std::remove_if(candidates.begin(), it_end,
                            [&](ComponentInfo s)
                            {
                              return compareTopics(s.getOutputTopics(), req.output_topics);
                            });
  }

  // If required parameters are specified ...
  if (!req.required_parameters.empty())
  {
    it_end = std::remove_if(candidates.begin(), it_end,
                            [&](ComponentInfo s)
                            {
                              return compareTopics(s.getRequiredParameters(), req.required_parameters);
                            });
  }

  // Sort remaining candidates based on their reliability.
  std::sort( candidates.begin()
           , it_end
           , [](ComponentInfo& ci1, ComponentInfo& s2)
             {
               return ci1.getReliability() > s2.getReliability();
             });

  if (candidates.begin() == it_end)
  {
    // Component with the requested criteria was not found.
    return false;
  }

  // Return the first component of the requested type.
  ci_ret = candidates;
  return true;
}

bool ComponentInfoRegistry::findComponent( const ComponentInfo &ci
                                   , const std::vector<ComponentInfo>& components
                                   , ComponentInfo& ci_ret ) const
{
  const auto it = std::find_if( components.begin()
                              , components.end()
                              , [&](const ComponentInfo& rs)
                              {
                                return rs == ci;
                              });


  if (it == components.end())
  {
    return false;
  }
  else
  {
    ci_ret = *it;
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

const std::map<std::string, PipeInfos>& ComponentInfoRegistry::getPipes() const
{
  return categorized_pipes_;
}

/*
 * ComponentInfoRegistry::findPipes
 */
bool ComponentInfoRegistry::findPipes( const LoadPipe::Request& req
                                     , PipeInfos& pipes_ret) const
{
  // Lock the mutex
  std::lock_guard<std::recursive_mutex> guard(read_write_mutex_pipe_);

  // Get the tracking methods of the requested category
  const auto& pipes_cat = categorized_pipes_.find(req.pipe_category);

  // Throw an error if the requested pipe category does not exist
  if (pipes_cat == categorized_pipes_.end())
  {
    return false;
  }

  // Get the pipes
  pipes_ret = pipes_cat->second;

  /*
   * Check if a specific pipe (pipe_name is specified) is requested
   */
  if (!req.pipe_name.empty())
  {
    // PipeInfos pipes
    PipeInfos pipes_ret_cpy = pipes_ret;
    bool named_pipe_found = false;
    for (const auto& pipe_ret : pipes_ret_cpy)
    {
      if(pipe_ret.getName() == req.pipe_name)
      {
        pipes_ret.clear();
        pipes_ret.push_back(pipe_ret);
        named_pipe_found = true;
        break;
      }
    }

    if (!named_pipe_found)
    {
      return false;
    }
  }

  /*
   * TODO: Check the segment specifiers
   */
  // if (!req.pipe_segment_specifiers.empty())
  // {
  //   // Loop over pipes
  //   for (auto pipe_it = pipes_ret.begin(); pipe_it != pipes_ret.end(); /* empty */)
  //   {
  //     // Remove the pipe if its last filter does not contain the required topic type
  //     if ( false)
  //     {
  //       pipes_ret.erase(pipe_it);
  //     }
  //     else
  //     {
  //       pipe_it++;
  //     }
  //   }
  // }

  /*
   * Check if there are any required types for the output topics of the pipe
   */ 
  if (!req.output_topics.empty())
  {
    // Loop over pipes
    for (auto pipe_it = pipes_ret.begin(); pipe_it != pipes_ret.end(); /* empty */)
    {
      bool pipe_suitable = true;

      // Create a copy of the required topics
      std::vector<diagnostic_msgs::KeyValue> req_topic_types = req.output_topics;
      std::set<std::string> last_filter_topics = pipe_it->getSegments().back().required_output_topic_types_;

      // The topics that the last filter of the pipe provides
      for (auto& last_filter_topic : last_filter_topics)
      {
        bool topic_found = false;

        // Compare the topic of the last filter with the required topics
        for (auto rtt_it = req_topic_types.begin(); rtt_it != req_topic_types.end(); rtt_it++)
        {
          if (rtt_it->key == last_filter_topic)
          {
            topic_found = true;
            req_topic_types.erase(rtt_it);
            break;
          }
        }

        // If the required topic was not found then break the loop and indicate
        // that the pipe is not suitable
        if (!topic_found)
        {
          pipe_suitable = false;
          break;
        }
      }

      // Remove the pipe if its last filter does not contain the required topic type
      if (!pipe_suitable)
      {
        pipes_ret.erase(pipe_it);
      }
      else
      {
        pipe_it++;
      }
    }

    // If no pipe was suitable, then throw an error
    if (pipes_ret.empty())
    {
      return false;
    }
  }

  /*
   * Sort the pipes with decreasing reliability order
   */
  std::sort( pipes_ret.begin()
           , pipes_ret.end()
           , [](const PipeInfo& lhs, const PipeInfo& rhs)
             {
               return lhs.reliability_.getReliability() >
                      rhs.reliability_.getReliability();
             });

  return true;
}

PipeInfo* ComponentInfoRegistry::findPipe ( const PipeInfo& pi )
{
  // Check if the requested type exists
  const auto pipes_it = categorized_pipes_.find(pi.getType());
  PipeInfo* pi_ret = NULL;

  if (pipes_it == categorized_pipes_.end())
  {
    return pi_ret;
  }
  
  // Check if there is such a pipe as requested
  const auto pipe_it = std::find_if( pipes_it->second.begin()
    , pipes_it->second.end()
    , [&](const PipeInfo& rs)
    {
      return rs == pi;
    });

  if (pipe_it == pipes_it->second.end())
  {
    return pi_ret;
  }
  else
  {
    pi_ret = &(*pipe_it);
    return pi_ret;
  }
}

bool ComponentInfoRegistry::addPipe( const PipeInfo& pi)
{
  // Lock the mutex
  std::lock_guard<std::recursive_mutex> guard(read_write_mutex_pipe_);

  // Check if there is no such pipe
  PipeInfo* pi_ret = findPipe(pi);
  if (pi_ret == NULL)
  {
    // Create an unique identifier for this specific pipe_info instance
    std::string pipe_name = pi.getType() + std::to_string(pipe_info_id_manager_.generateID());
    categorized_pipes_[pi.getType()].emplace_back(pi, pipe_name);
    return true;
  }

  // Return false if such pipe already exists
  return false;
}

/*
 * ComponentInfoRegistry::updatePipe
 */
bool ComponentInfoRegistry::updatePipe( const PipeInfo& pi )
{
  // Lock the mutex
  std::lock_guard<std::recursive_mutex> guard(read_write_mutex_pipe_);

  PipeInfo* pi_ret = findPipe(pi);
  if (pi_ret != NULL)
  {
    *pi_ret = pi;
    return true;
  }

  // Return false if no such pipe was found
  return false;
}

ComponentInfoRegistry::~ComponentInfoRegistry()
{
  stop_cleanup_loop_ = true;
  update_callback_cleanup_thread_.join();
}

} // component_manager namespace
