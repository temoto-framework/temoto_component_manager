cmake_minimum_required(VERSION 2.8.3)
project(temoto_component_manager)
add_compile_options(-std=c++14 -Wno-reorder -Wno-unused-function -Wno-pedantic -g)

option(TEMOTO_ENABLE_TRACING "Use tracer" OFF)

if(TEMOTO_ENABLE_TRACING)
  add_compile_options(-Denable_tracing)
endif()

find_package(catkin REQUIRED COMPONENTS
  temoto_core
  temoto_resource_registrar
  temoto_action_engine
  temoto_er_manager
  roscpp
  roslib
  genmsg
  std_msgs
  diagnostic_msgs
  message_generation
)

find_package(Boost COMPONENTS program_options REQUIRED)

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
  temoto_resource_registrar
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs diagnostic_msgs temoto_core temoto_action_engine temoto_er_manager temoto_resource_registrar
  DEPENDS 
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
  ${Boost_LIBRARIES}
)

install(TARGETS temoto_component_manager
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

