cmake_minimum_required(VERSION 3.16)
project(moveit_behavior)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(dependencies_pkgs
    ament_cmake
    rclcpp
    moveit_core
    moveit_msgs
    behaviortree_cpp
    behaviortree_ros2
    btcpp_ros2_interfaces
    drt_behavior
    dex_ivr_helpers
)

foreach(pkg IN LISTS dependencies_pkgs)
    find_package(${pkg} REQUIRED)
endforeach()

set(behavior_cpp_files
    src/execute_trajectory.cpp
    src/get_planning_scene.cpp
    src/modify_collisions.cpp
    src/plan_to_joint_state.cpp
    src/plan_to_pose.cpp
    src/publish_trajectory.cpp
    src/plugins.cpp
    src/twist_about_frame.cpp
)

add_library(${PROJECT_NAME} SHARED ${behavior_cpp_files})
target_compile_definitions(${PROJECT_NAME} PRIVATE BT_PLUGIN_EXPORT)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${dependencies_pkgs})

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)
install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS
  ${PROJECT_NAME}
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
)

install(DIRECTORY behavior_trees_xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(${dependencies_pkgs})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(gtest_plugin test/gtest_plugin.cpp)
  target_include_directories(gtest_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )
  ament_target_dependencies(gtest_plugin ${dependencies_pkgs} ament_cmake_gtest ament_cmake_gmock)
  target_link_libraries(gtest_plugin ${PROJECT_NAME})
endif()


ament_package()
