cmake_minimum_required(VERSION 3.8)
project(spawn_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SpawnBox.srv"
  "srv/SpawnBeer.srv"
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(spawn_box_node src/box_spawner.cpp)
add_executable(spawn_beer_node src/beer_spawner.cpp)

rosidl_target_interfaces(spawn_box_node
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

rosidl_target_interfaces(spawn_beer_node
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_include_directories(spawn_box_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(spawn_beer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(spawn_box_node
  rclcpp
  gazebo_msgs
  tf2_ros
  geometry_msgs
  ament_index_cpp
  rosidl_default_runtime
)

ament_target_dependencies(spawn_beer_node
  rclcpp
  gazebo_msgs
  tf2_ros
  geometry_msgs
  ament_index_cpp
  rosidl_default_runtime
)

install(TARGETS
  spawn_box_node
  spawn_beer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY srv
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY models
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include/)

ament_package()
