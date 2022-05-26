
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 *
 *  The basis of this file has been automatically generated
 *  by the TeMoto action package generator. Modify this file
 *  as you wish but please note:
 *
 *    WE HIGHLIY RECOMMEND TO REFER TO THE TeMoto ACTION
 *    IMPLEMENTATION TUTORIAL IF YOU ARE UNFAMILIAR WITH
 *    THE PROCESS OF CREATING CUSTOM TeMoto ACTION PACKAGES
 *    
 *  because there are plenty of components that should not be
 *  modified or which do not make sence at the first glance.
 *
 *  See TeMoto documentation & tutorials at: 
 *    https://temoto-telerobotics.github.io
 *
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

/* REQUIRED BY TEMOTO */
#include <class_loader/class_loader.hpp>
#include "ta_find_component_packages/temoto_action.h"

#include "temoto_component_manager/component_info.h"
#include "temoto_component_manager/component_info_registry.h"

#include <boost/filesystem/operations.hpp>
#include "yaml-cpp/yaml.h"
#include <fstream>
#include <chrono>

/* 
 * ACTION IMPLEMENTATION of TaFindComponentPackages 
 */
class TaFindComponentPackages : public TemotoAction
{
public:

// Constructor. REQUIRED BY TEMOTO
TaFindComponentPackages()
{
  // ---> YOUR CONSTRUCTION ROUTINES HERE <--- //
  TEMOTO_INFO("Constructed");
}

// REQUIRED BY TEMOTO
void executeTemotoAction()
{
  // Input parameters
  std::string catkin_ws_path = GET_PARAMETER("catkin_ws_path", std::string);
  temoto_component_manager::ComponentInfoRegistry* cir = GET_PARAMETER("cir", temoto_component_manager::ComponentInfoRegistry*);

  // Look for new packages every 10 seconds
  while(actionOk())
  {
    TEMOTO_DEBUG_STREAM("Snooping the catkin workspace at: " << catkin_ws_path);

    // Find all component descriptor file paths
    boost::filesystem::path base_path (catkin_ws_path);
    std::vector<std::string> component_desc_file_paths = findComponentDescFiles(base_path);

    // Read in the component descriptors
    std::vector<temoto_component_manager::ComponentInfo> component_infos;
    for (const std::string& desc_file_path : component_desc_file_paths)
    {
      try
      {
        std::vector<temoto_component_manager::ComponentInfo> component_infos_current = getComponentInfo(desc_file_path);
        component_infos.insert( component_infos.end()
                              , component_infos_current.begin()
                              , component_infos_current.end());
      }
      catch (std::exception& e)
      {
        // Rethrow the exception
        TEMOTO_ERROR_STREAM(e.what() << " in " << desc_file_path);
      }
    }

    TEMOTO_DEBUG_STREAM("got " << component_infos.size() << " components");

    for (auto si : component_infos)
    {
      if (cir->addLocalComponent(si))
      {
        TEMOTO_INFO("Added a new component");
      }
      else
      {
        TEMOTO_DEBUG("This component already exists in the CID");
      }
    }
    // Sleep for 10 seconds
    sleepAndCheckOk(10);
  }
}

/**
 * @brief Sleeps for set time while checking if the action should be stopped or not
 * 
 * @param time 
 */
void sleepAndCheckOk(float time)
{
  float timestep = 1;
  float current_duration = 0;
  while(actionOk() && (current_duration < time))
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(int(timestep*1000)));
    current_duration += timestep;
  }
}

/**
 * @brief findComponentDescFiles
 * @param path
 * @return
 */
std::vector<std::string> findComponentDescFiles( const boost::filesystem::path& base_path, int search_depth = 4)
{
  boost::filesystem::directory_iterator end_itr;
  std::vector <std::string> desc_file_paths;

  try
  {
    // Start looking the files in current directory
    for (boost::filesystem::directory_iterator itr( base_path ); itr != end_itr; ++itr)
    {
      // if its a directory and depth limit is not there yet, go incire it
      if (boost::filesystem::is_directory(*itr) &&
          checkIgnoreDirs(itr->path().filename().string()) &&
          (search_depth > 0))
      {
         std::vector <std::string> sub_desc_file_paths = findComponentDescFiles( *itr, search_depth - 1 );

        // Append the subtasks if not empty
        if (!sub_desc_file_paths.empty())
        {
          desc_file_paths.insert( std::end(desc_file_paths)
                                , std::begin(sub_desc_file_paths)
                                , std::end(sub_desc_file_paths));
        }
      }

      // if its a file and matches the desc file name, process the file
      else if ( boost::filesystem::is_regular_file(*itr) &&
                (itr->path().filename() == description_file_) )
      {
        desc_file_paths.push_back(itr->path().string());
      }

      // if (boost::filesystem::is_regular_file(*itr))
      // {
      //   std::cout << "got: " << itr->path().filename() << std::endl;
      // }  
    }
    return desc_file_paths;
  }
  catch (std::exception& e)
  {
    // Rethrow the exception
    throw CREATE_ERROR(temoto_core::error::Code::FIND_TASK_FAIL, e.what());
  }

  catch(...)
  {
    // Rethrow the exception
    throw CREATE_ERROR(temoto_core::error::Code::UNHANDLED_EXCEPTION, "Received an unhandled exception");
  }
}

/**
 * @brief getComponentInfo
 * @param desc_file_path
 * @return
 */
std::vector<temoto_component_manager::ComponentInfo> getComponentInfo(const std::string& desc_file_path) const
{
  std::ifstream in( desc_file_path );
  YAML::Node config = YAML::Load(in);
  std::vector<temoto_component_manager::ComponentInfo> components;

  // Parse any component related information
  if (config["Components"])
  {
    components = parseComponents(config, desc_file_path);
    for (auto& s : components)
    {
      TEMOTO_DEBUG("Got component: '%s'.", s.getName().c_str());
    }
  }
  else
  {
    TEMOTO_WARN("Failed to read '%s'. Verify that the file exists and the sequence of components "
                "is listed under 'Components' node.", desc_file_path.c_str());
  }

  return std::move(components);
}

/**
 * @brief parseComponents
 * @param config
 * @return
 */
std::vector<temoto_component_manager::ComponentInfo> parseComponents(const YAML::Node& config, std::string file_path) const
{
  std::vector<temoto_component_manager::ComponentInfo> components;

  if (!config.IsMap())
  {
    TEMOTO_WARN_STREAM("Unable to parse 'Components' key from config:"  << file_path);
    return components;
  }

  YAML::Node components_node = config["Components"];
  if (!components_node.IsSequence())
  {
    TEMOTO_DEBUG_STREAM("The given config does not contain sequence of components: " << file_path);
    return components;
  }

  TEMOTO_DEBUG("Parsing %lu components.", components_node.size());

  // go over each component node in the sequence
  unsigned int i = 0;
  for (YAML::const_iterator node_it = components_node.begin(); node_it != components_node.end(); ++node_it)
  {
    if (!node_it->IsMap())
    {
      TEMOTO_WARN_STREAM("Unable to parse the component. Parameters in YAML have to be specified in "
                   "key-value pairs: " << file_path);
      continue;
    }

    try
    {
      temoto_component_manager::ComponentInfo component = node_it->as<temoto_component_manager::ComponentInfo>();
      if (std::count_if( components.begin()
                       , components.end()
                       , [&](const temoto_component_manager::ComponentInfo& s)
                         {
                            return s == component;
                         }) == 0)
      {
        // OK, this is unique pointer, add it to the components vector.
        components.emplace_back(component);
        //TEMOTO_DEBUG_STREAM("####### PARSED COMPONENT: #######\n" << components.back()->toString());
      }
      else
      {
        TEMOTO_WARN("Ignoring duplicate of component '%s'.", component.getName().c_str());
      }
      i++;
    }
    catch (YAML::TypedBadConversion<temoto_component_manager::ComponentInfo> e)
    {
      TEMOTO_WARN_STREAM("Failed to parse component nr '" << i << "' from config: " << file_path);
      continue;
    }
  }
  return std::move(components);
}

/**
 * @brief Checks if the given directory should be ignored or not
 * @param dir
 * @return
 */
bool checkIgnoreDirs( std::string dir)
{
  for (const std::string& ignore_dir : ignore_dirs_)
  {
    if (dir == ignore_dir)
    {
      return false;
    }
  }

  return true;
}

/// Directories that can be ignored
std::vector<std::string> ignore_dirs_{ "src"
                                     , "include"
                                     , ".git"
                                     , "launch"
                                     , "build"
                                     , "description"
                                     , "actions"
                                     , "msg"
                                     , "srv"
                                     , "scripts"};

/// Name of the component description file
std::string description_file_= "components.yaml";

// Destructor
~TaFindComponentPackages()
{
  // ---> YOUR CONSTRUCTION ROUTINES HERE <--- //
  TEMOTO_INFO_STREAM("Destructor");
}

}; // TaFindComponentPackages class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaFindComponentPackages, ActionBase);
