cmake_minimum_required(VERSION 3.16)
project(geometry_behavior)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(dependencies_pkgs
    ament_cmake
    rclcpp
    behaviortree_cpp
    behaviortree_ros2
    btcpp_ros2_interfaces
    geometry_msgs
    tf2_eigen
    drt_behavior
)

foreach(pkg IN LISTS dependencies_pkgs)
    find_package(${pkg} REQUIRED)
endforeach()

set(behavior_cpp_files
    src/align_transform.cpp
    src/apply_transform.cpp
    src/async_tf_lookup.cpp
    src/transform_to_pose.cpp
    src/pose_to_transform.cpp
    src/create_random_transform.cpp
    src/publish_transform.cpp
    src/wrench_check.cpp
    src/plugins.cpp
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
