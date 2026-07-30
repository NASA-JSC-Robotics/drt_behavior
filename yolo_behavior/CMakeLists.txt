cmake_minimum_required(VERSION 3.16)
project(yolo_behavior)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ENABLE_GPU "Enable CUDA GPU support" ON)


set(dependencies_pkgs
    ament_cmake
    rclcpp
    # Behavior Tree packages
    behaviortree_cpp
    behaviortree_ros2
    btcpp_ros2_interfaces
    drt_behavior
    # deps that should be available regardless
    geometry_msgs
    # framework related deps
    color_tools_msgs
    message_filters
    image_transport
    #
    color_blob_centroid
    ros2_yolos_cpp
    vision_msgs
)

foreach(pkg IN LISTS dependencies_pkgs)
    find_package(${pkg} REQUIRED)
endforeach()

find_package(ros2_yolos_cpp REQUIRED)
set(behavior_cpp_files
    src/detect_pose.cpp
    src/pixels_to_depth.cpp
    src/segmentor.cpp
    src/cvmat_to_depth.cpp
    src/plugins.cpp
)

add_library(${PROJECT_NAME} SHARED ${behavior_cpp_files})
target_compile_definitions(${PROJECT_NAME} PRIVATE BT_PLUGIN_EXPORT)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${dependencies_pkgs})
# target_link_libraries(${PROJECT_NAMES} ${ros2_yolos_cpp_LIBRARIES})
# get_target_property(LINKED_LIBS ${PROJECT_NAME} LINK_LIBRARIES)
# message(WARNING "Libraries linked to my_executable: ${LINKED_LIBS}")

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
