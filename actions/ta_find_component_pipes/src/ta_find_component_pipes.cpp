
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
#include "ta_find_component_pipes/temoto_action.h"
#include "ta_find_component_pipes/macros.h"

#include "temoto_component_manager/pipe_info.h"
#include "temoto_component_manager/component_info_registry.h"

#include <boost/filesystem/operations.hpp>
#include "yaml-cpp/yaml.h"
#include <fstream>

/* 
 * ACTION IMPLEMENTATION of TaFindComponentPipes 
 */
class TaFindComponentPipes : public TemotoAction
{
public:

// Constructor. REQUIRED BY TEMOTO
TaFindComponentPipes()
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
    std::vector<std::string> pipe_desc_file_paths = findPipeDescFiles(base_path);

    // Read in the pipe infos
    temoto_component_manager::PipeInfos pipe_infos;
    for (const std::string& desc_file_path : pipe_desc_file_paths)
    {
      try
      {
        temoto_component_manager::PipeInfos pipe_infos_current = getPipeInfos(desc_file_path);
        pipe_infos.insert( pipe_infos.end()
                         , pipe_infos_current.begin()
                         , pipe_infos_current.end());
      }
      catch(...)
      {
        // TODO: implement a proper catch block
      }
    }

    TEMOTO_DEBUG_STREAM("got " << pipe_infos.size() << " pipes");

    for (auto pi : pipe_infos)
    {
      if (cir->addPipe(pi))
      {
        TEMOTO_INFO("Added a new pipe");
      }
      else
      {
        TEMOTO_DEBUG("This pipe already exists in the CID");
      }
    }

    // Sleep for 10 seconds
    sleepAndCheckOk(10);
  }
}

/**
 * @brief findPipeDescFiles
 * @param path
 * @return
 */
std::vector<std::string> findPipeDescFiles( const boost::filesystem::path& base_path
                                               , int search_depth = 3)
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
         std::vector <std::string> sub_desc_file_paths = findPipeDescFiles( *itr, search_depth - 1 );

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
 * @brief Get the Pipe Infos object
 * 
 * @param desc_file_path 
 * @return temoto_component_manager::PipeInfos 
 */
temoto_component_manager::PipeInfos getPipeInfos(const std::string& desc_file_path) const
{
  std::ifstream in( desc_file_path );
  YAML::Node config = YAML::Load(in);
  temoto_component_manager::PipeInfos pipe_infos;

  // Parse any component related information
  pipe_infos = parsePipes(config);
  for (auto& pi : pipe_infos)
  {
    TEMOTO_DEBUG("Got pipe: '%s'.", pi.getType().c_str());
  }

  return std::move(pipe_infos);
}

/**
 * @brief 
 * 
 * @param config 
 * @return temoto_component_manager::PipeInfos 
 */
temoto_component_manager::PipeInfos parsePipes(const YAML::Node& config) const
{
  temoto_component_manager::PipeInfos pipe_infos;

  // Check if it is a map
  if (!config.IsMap())
  {
    // TODO Throw
    std::cout << " throw throw throw \n";
    return pipe_infos;
  }

  // Iterate over different pipe categories (hand pipes, artag pipes, ...)
  for (YAML::const_iterator pipe_type_it = config.begin(); pipe_type_it != config.end(); ++pipe_type_it)
  {
    // Each category must contain a sequence of tracking methods
    if (!pipe_type_it->second.IsSequence())
    {
      // TODO: Throw
      std::cout << " throw TODO throw TODO \n";
      return pipe_infos;
    }

    // Get the category of the pipe
    std::string pipe_category = pipe_type_it->first.as<std::string>();

    // Iterate over different tracking methods within the given category
    for (YAML::const_iterator method_it = pipe_type_it->second.begin();
         method_it != pipe_type_it->second.end();
         ++method_it)
    {
      try
      {
        // Convert the tracking method yaml description into PipeInfo
        temoto_component_manager::PipeInfo pipe_info = method_it->as<temoto_component_manager::PipeInfo>();
        pipe_info.setType(pipe_category);

        // Add the tracking method into the map of locally known pipes
        pipe_infos.push_back(pipe_info);

        // TODO: Print via TEMOTO_DEBUG
        // std::cout << pipe_info.toString() << std::endl;
      }
      catch (YAML::InvalidNode e)
      {
        // print out the error message
        std::cout << "Conversion failed: " << e.what() << std::endl;
      }
    }
  }

  return std::move(pipe_infos);
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
    ros::Duration(timestep).sleep();
    current_duration += timestep;
  }
}

/// Directories that will be ignored
std::vector<std::string> ignore_dirs_{ "src"
                                     , "include"
                                     , ".git"
                                     , "launch"
                                     , "config"
                                     , "build"
                                     , "description"
                                     , "actions"
                                     , "msg"
                                     , "srv"
                                     , "scripts"};

/// Name of the component description file
std::string description_file_= "pipes.yaml";

// Destructor
~TaFindComponentPipes()
{
  // ---> YOUR CONSTRUCTION ROUTINES HERE <--- //
  TEMOTO_PRINT_OF("Destructor", getUmrfPtr()->getName());
}

}; // TaFindComponentPipes class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaFindComponentPipes, ActionBase);
