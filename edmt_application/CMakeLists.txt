cmake_minimum_required(VERSION 3.8)
project(edmt_application)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies_pkgs
  rclcpp
  std_msgs
  sensor_msgs
  tf2_eigen
  moveit_visual_tools
  moveit_ros_planning_interface
  tl_expected
  controller_manager
  behaviortree_ros2
)

# find dependencies
find_package(ament_cmake REQUIRED)

foreach(pkg IN LISTS dependencies_pkgs)
  find_package(${pkg} REQUIRED)
endforeach()

set(behavior_cpp_files
  src/behaviors/create_collision_object.cpp
  src/behaviors/plan_joint_states.cpp
  src/behaviors/plan_named_state.cpp
  src/behaviors/plan_relative_move.cpp
  src/behaviors/prompt_and_execute.cpp
  src/behaviors/publish_instruction_text.cpp
  src/behaviors/tf_lookup.cpp
  src/behaviors/update_collision_matrix.cpp
)

add_library(edmt_application 
  src/edmt_application.cpp 
  src/edmt_application_btcpp_executor.cpp 
  src/edmt_application_btcpp_logger.cpp 
  ${behavior_cpp_files})
target_compile_features(edmt_application PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(edmt_application PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
ament_target_dependencies(edmt_application PUBLIC ${dependencies_pkgs})

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS edmt_application
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

ament_export_include_directories(
  include
)
ament_export_libraries(
  edmt_application
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(${dependencies_pkgs})

ament_package()
