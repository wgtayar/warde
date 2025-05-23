cmake_minimum_required(VERSION 3.8)
project(warde_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_index_cpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_srvs REQUIRED)
# find_package(manipulation_msgs REQUIRED)
find_package(robot_nav REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)


include_directories(include
  ${BEHAVIORTREE_CPP_V3_INCLUDE_DIRS}
)

# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# CREATE AN EXECUTABLE FOR THE WARDE TREE AND ITS CORRESPONDING AMENT TARGET DEPENDENCIES AND INSTALL TARGETS
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

add_executable(warde_bt_main
  src/warde_bt_main.cpp
  src/condition_box_present.cpp
  src/condition_beer_present.cpp
  src/action_get_closest_beer.cpp
  src/action_navigate.cpp
  src/action_manipulate.cpp
)

add_executable(test_conditions
  src/test_condition.cpp
  src/condition_box_present.cpp
  src/condition_beer_present.cpp
  src/action_get_closest_beer.cpp
  src/action_navigate.cpp
  src/action_manipulate.cpp
)
ament_target_dependencies(test_conditions
  rclcpp
  behaviortree_cpp_v3
  tf2_ros
  ament_index_cpp
  robot_nav
  rclcpp_action
  geometry_msgs
)

ament_target_dependencies(warde_bt_main
  rclcpp
  behaviortree_cpp_v3
  tf2_ros
  ament_index_cpp
  robot_nav
  rclcpp_action
  geometry_msgs
)

# !!!!!!!!!!!!!!!!!!!!!!! also add manipulation_msgs later !!!!!!!!!!!!!!!!!!!!!!!!!

install(TARGETS test_conditions
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS warde_bt_main
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

add_library(bt_get_closest_beer SHARED
  src/action_get_closest_beer.cpp
)

add_library(bt_navigate SHARED
  src/action_navigate.cpp
)

ament_target_dependencies(bt_get_closest_beer
  rclcpp tf2_ros behaviortree_cpp_v3
)

ament_target_dependencies(bt_navigate
  rclcpp
  std_srvs
  behaviortree_cpp_v3
  robot_nav
  rclcpp_action
  geometry_msgs
  tf2_ros
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS bt_get_closest_beer
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(TARGETS bt_navigate
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
