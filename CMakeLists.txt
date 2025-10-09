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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


function(custom_examples target)
  add_executable(${target} examples/${target}.cpp)
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
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()


function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target} 
    "rclcpp"
    "geometry_msgs"
    "std_msgs"
    "tf2_ros"
    "sensor_msgs"
    "nav_msgs"
    "visualization_msgs"
    "yaml-cpp"
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
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()

function(custom_component target class_name)
  rclcpp_components_register_node(${target}
    PLUGIN ${class_name}
    EXECUTABLE ${target}
  )
  ament_target_dependencies(${target}
    "rclcpp"
    "geometry_msgs"
    "std_msgs"
    "tf2_ros"
    "sensor_msgs"
    "nav_msgs"
    "rclcpp_components"
    "visualization_msgs"
    "yaml-cpp"
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
endfunction()

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

install(
  DIRECTORY launch worlds models urdf config map meshes
  DESTINATION share/${PROJECT_NAME}
)

if (BUILD_WITH_SIM)
  custom_executable(omuni_drive_node)
  custom_executable(omuni_odometry)
  custom_executable(example_omuni_drive_node)
  custom_executable(diff_drive)
  custom_executable(vel_feedback_node)
  custom_executable(dummy_vacume_uart)
else()
  custom_executable(vel_feedback_uart)
  custom_executable(vacume_uart)
endif()

custom_executable(mcl_node)
custom_executable(joy2vel)
custom_executable(uart_sample)
custom_executable(gen_path)
custom_executable(follow_node)
custom_executable(bt_node)
# target_link_libraries(bt_node "${cpp_typesupport_target}") 
custom_executable(rotate_node)

ament_package()
