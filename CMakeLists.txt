cmake_minimum_required(VERSION 3.5)
project(software_training)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(rcutils REQUIRED)
# find_package(rcl REQUIRED)
# find_package(rclcpp_action REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(builtin_interfaces REQUIRED)
# find_package(rosidl_default_generators REQUIRED)


#include the 'include' directory
include_directories(include)

# custom services and messages and actions
# rosidl_generate_interfaces(${PROJECT_NAME}
# 	"msg/Software.msg"
# 	"srv/Software.srv"
# 	"action/Software.action"
# 	DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
# 	)
# ament_export_dependencies(rosidl_default_runtime)

#create resource which references the libraries in the binary bin
set(node_plugins "")

#add plugins as SHARED library 

#add turtle_request_service_node as a plugin
add_library(kill_alive SHARED 
	src/turtle_kill.cpp)
target_compile_definitions(kill_alive
	PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(kill_alive
	"rclcpp"
	"rclcpp_components"
	"turtlesim"
	"std_msgs")
rclcpp_components_register_nodes(kill_alive "kill_all_composition::kill_all_turtle_service_call")
# this way we can execute the component with - ros2 component standalone software_training composition::turtle_service_request_node
set(node_plugins "${node_plugins}kill_all_composition::kill_all_turtle_service_call;$<TARGET_FILE:kill_alive>\n")



#tell where to put binaries
install(TARGETS
	kill_alive
	
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib 
	RUNTIME DESTINATION bin)

#install launch file
install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}
	)

#install configuration file (yaml files) for param server
# install(DIRECTORY
# 	config
# 	DESTINATION share/${PROJECT_NAME}
# 	)

ament_package()