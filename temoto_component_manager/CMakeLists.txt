cmake_minimum_required(VERSION 2.8.3)
project(temoto_component_manager)
add_compile_options(-std=c++1y -Wno-reorder -Wno-unused-function -Wno-pedantic)

find_package(catkin REQUIRED COMPONENTS
  temoto_core
  temoto_action_engine
  temoto_er_manager
  roscpp
  roslib
  genmsg
  std_msgs
  diagnostic_msgs
  message_generation
)

add_message_files(FILES
  Component.msg
  Pipe.msg
  PipeSegment.msg
  PipeSegmentSpecifier.msg
)

add_service_files(FILES
  ListComponents.srv
  ListPipes.srv
  LoadComponent.srv
  LoadPipe.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  diagnostic_msgs
  temoto_core
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yaml-cpp
  CATKIN_DEPENDS roscpp std_msgs diagnostic_msgs temoto_core temoto_action_engine temoto_er_manager
  DEPENDS 
    TinyXML
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(temoto_component_manager 
  src/component_manager_node.cpp
  src/component_manager_servers.cpp
  src/component_snooper.cpp
  src/component_info_registry.cpp
  src/component_info.cpp
)
add_dependencies(temoto_component_manager
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(temoto_component_manager
  ${catkin_LIBRARIES} 
  yaml-cpp 
)

