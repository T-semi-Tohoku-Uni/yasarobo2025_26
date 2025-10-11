cmake_minimum_required(VERSION 3.8)
project(yasarobo2025_26)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if("$ENV{WITH_SIM}" STREQUAL "1")
  set(BUILD_WITH_SIM ON)
else()
  set(BUILD_WITH_SIM OFF)
endif()
message(STATUS "BUILD_WITH_SIM = ${BUILD_WITH_SIM}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(inrof2025_ros_type REQUIRED)

if (BUILD_WITH_SIM)
  find_package(gazebo_ros REQUIRED)
  find_package(gazebo_msgs REQUIRED)
endif()

# library link utility
function(dependencies target)
  ament_target_dependencies(${target} 
    "rclcpp"
    "geometry_msgs"
    "std_msgs"
    "tf2_ros"
    "sensor_msgs"
    "nav_msgs"
    "visualization_msgs"
    "OpenCV"
    "tf2_geometry_msgs"
    "laser_geometry"
    "tf2_sensor_msgs"
    "behaviortree_cpp"
    "inrof2025_ros_type"
  )
  if (BUILD_WITH_SIM)
    ament_target_dependencies(${target} 
      "gazebo_msgs"
    )
  endif()
  target_link_libraries(${target} ${YAML_CPP_LIBRARIES}) 
endfunction()

function(ros_node_executable target)
  add_executable(${target} src/${target}.cpp)
  dependencies(${target})
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()


# shared library of behaivor tree
add_library(bt_lib SHARED
  bt/bt_vacume_on.cpp
  bt/bt_ball_detect.cpp
  # add bt/bt_..._.cpp
)
target_include_directories(bt_lib
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(bt_lib
  "rclcpp"
  "rclcpp_action"
  "behaviortree_cpp"
  "inrof2025_ros_type"
)
install(TARGETS bt_lib DESTINATION lib/${PROJECT_NAME})

# bt_node
add_executable(bt_node src/bt_node.cpp)
dependencies(bt_node)
set_target_properties(bt_node PROPERTIES INSTALL_RPATH "$ORIGIN")
target_link_libraries(bt_node bt_lib)
install(TARGETS bt_node DESTINATION lib/${PROJECT_NAME})

# ros2 node executable
ros_node_executable(joy2vel)
ros_node_executable(vel_feedback_uart)
ros_node_executable(vacume_uart)
ros_node_executable(gen_path)
ros_node_executable(follow_node)
ros_node_executable(rotate_node)
ros_node_executable(dummy_vacume_uart)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch worlds models urdf config map meshes
  DESTINATION share/${PROJECT_NAME}
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

  # Integration tests
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()
  add_ros_isolated_launch_test(test/test_integration.py)
endif()

ament_package()
