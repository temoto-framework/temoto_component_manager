
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
 *    https://utnuclearroboticspublic.github.io/temoto2
 *
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

/* REQUIRED BY TEMOTO */
#include "temoto_nlp/base_task/base_task.h"    // The base task
#include <class_loader/class_loader.h>  // Class loader includes

#include "temoto_component_manager/component_info.h"
#include "temoto_component_manager/component_info_registry.h"

#include <boost/filesystem/operations.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>


/* 
 * ACTION IMPLEMENTATION of TaFindComponentPackages 
 */
class TaFindComponentPackages : public temoto_nlp::BaseTask
{
public:

/* REQUIRED BY TEMOTO */
TaFindComponentPackages()
{
  // ---> YOUR CONSTRUCTION ROUTINES HERE <--- //
  TEMOTO_INFO("TaFindComponentPackages constructed");
}
    
/* REQUIRED BY TEMOTO */
void startTask(temoto_nlp::TaskInterface task_interface)
{
  input_subjects = task_interface.input_subjects_;
  switch (task_interface.id_)
  {
        
    // Interface 0
    case 0:
      startInterface_0();
      break;

  }
}

/* REQUIRED BY TEMOTO */
std::vector<temoto_nlp::Subject> getSolution()
{
  return output_subjects;
}

~TaFindComponentPackages()
{
  TEMOTO_INFO("TaFindComponentPackages destructed");
}

/********************* END OF REQUIRED PUBLIC INTERFACE *********************/


private:
    
/*
 * Interface 0 body
 */
void startInterface_0()
{
  /* EXTRACTION OF INPUT SUBJECTS */
  temoto_nlp::Subject what_0_in = temoto_nlp::getSubjectByType("what", input_subjects);
  std::string  what_0_word_in = what_0_in.words_[0];
  std::string  what_0_data_0_in = boost::any_cast<std::string>(what_0_in.data_[0].value);
  temoto_component_manager::ComponentInfoRegistry*     what_0_data_1_in = boost::any_cast<temoto_component_manager::ComponentInfoRegistry*>(what_0_in.data_[1].value);

  // Get the catkin workspace src directory path
  std::string catkin_ws_src_dir = what_0_data_0_in;
  temoto_component_manager::ComponentInfoRegistry* cir = what_0_data_1_in;

  // Look for new packages every 10 seconds
  while(stop_task_ == false)
  {
    TEMOTO_DEBUG_STREAM("Snooping the catkin workspace at: " << catkin_ws_src_dir);

    // Find all component descriptor file paths
    boost::filesystem::path base_path (catkin_ws_src_dir);
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
      catch(...)
      {
        // TODO: implement a proper catch block
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
        TEMOTO_DEBUG("This component already exists in the SID");
      }
    }

    // Sleep for 10 seconds
    ros::Duration(10).sleep();
  }
}

/**
 * @brief findComponentDescFiles
 * @param path
 * @return
 */
std::vector<std::string> findComponentDescFiles( const boost::filesystem::path& base_path, int search_depth = 1)
{
  boost::filesystem::directory_iterator end_itr;
  std::vector <std::string> desc_file_paths;

  try
  {
    // Start looking the files incire current directory
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
    components = parseComponents(config);
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
std::vector<temoto_component_manager::ComponentInfo> parseComponents(const YAML::Node& config) const
{
  std::vector<temoto_component_manager::ComponentInfo> components;

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
    }
    catch (YAML::TypedBadConversion<temoto_component_manager::ComponentInfo> e)
    {
      TEMOTO_WARN("Failed to parse temoto_component_manager::ComponentInfo from config.");
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
std::vector<std::string> ignore_dirs_{"src", "launch", "config", "build", "description"};

/// Name of the component description file
std::string description_file_= "component_description.yaml";

}; // TaFindComponentPackages class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaFindComponentPackages, temoto_nlp::BaseTask);
