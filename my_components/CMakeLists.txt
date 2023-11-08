cmake_minimum_required(VERSION 3.8)
project(my_components)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(composition REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

# For adding Preapproach component 
add_library(PreApproach SHARED src/pre_approach.cpp)
target_compile_definitions(PreApproach PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(PreApproach
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
  )
rclcpp_components_register_nodes(PreApproach "my_components::PreApproach")
set(node_plugins "${node_plugins}my_components::PreApproach;$<TARGET_FILE:PreApproach>\n")
#end

# For adding server component 
add_library(server_component SHARED src/approach_service_server.cpp)
target_compile_definitions(server_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(server_component
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "tf2_ros"
  "custom_interfaces"
  "std_msgs"
  "sensor_msgs"
  )
rclcpp_components_register_nodes(server_component "my_components::AttachServer")
set(node_plugins "${node_plugins}my_components::AttachServer;$<TARGET_FILE:server_component>\n")
#end

# For adding client component 
add_library(client_component SHARED src/attach_client.cpp)
target_compile_definitions(client_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(client_component
  "rclcpp"
  "rclcpp_components"
  "custom_interfaces"
  )
rclcpp_components_register_nodes(client_component "my_components::AttachClient")
set(node_plugins "${node_plugins}my_components::AttachClient;$<TARGET_FILE:client_component>\n")
# end


# For adding component node to executable
add_executable(attach_server
  src/manual_composition.cpp)

target_link_libraries(attach_server
	server_component
)
ament_target_dependencies(attach_server
  "rclcpp")
# end


install(TARGETS
  attach_server

  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  PreApproach
  server_component
  client_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
