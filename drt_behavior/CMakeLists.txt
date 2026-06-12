cmake_minimum_required(VERSION 3.16)
project(drt_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies_pkgs
    rclcpp
    tf2_ros
    std_srvs
    behaviortree_ros2
)

# find dependencies
find_package(ament_cmake REQUIRED)

foreach(pkg IN LISTS dependencies_pkgs)
    find_package(${pkg} REQUIRED)
endforeach()

set(behavior_cpp_files
    src/behaviors/check_param.cpp
    src/behaviors/call_trigger_service.cpp
)

add_library(drt_behavior
    src/drt_behavior_btcpp_executor.cpp
    src/drt_behavior_btcpp_logger.cpp
    src/drt_tree_context.cpp
    ${behavior_cpp_files})
target_compile_features(drt_behavior PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
target_include_directories(drt_behavior PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(drt_behavior PUBLIC ${dependencies_pkgs})

add_executable(behavior_executor src/behavior_executor.cpp)
target_include_directories(behavior_executor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(behavior_executor ${dependencies_pkgs})
target_link_libraries(behavior_executor ${PROJECT_NAME})

install(TARGETS behavior_executor
    DESTINATION lib/${PROJECT_NAME})

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

# if(BUILD_TESTING)
#     find_package(ament_lint_auto REQUIRED)
#     # the following line skips the linter which checks for copyrights
#     # comment the line when a copyright and license is added to all source files
#     set(ament_cmake_copyright_FOUND TRUE)
#     # the following line skips cpplint (only works in a git repo)
#     # comment the line when this package is in a git repo and when
#     # a copyright and license is added to all source files
#     set(ament_cmake_cpplint_FOUND TRUE)
#     ament_lint_auto_find_test_dependencies()
# endif()

install(DIRECTORY config launch behavior_trees_xml rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
    include
)
ament_export_libraries(
    ${PROJECT_NAME}
)
ament_export_targets(
    export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(${dependencies_pkgs})

ament_package()
