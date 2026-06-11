cmake_minimum_required(VERSION 3.16)
project(drt_behavior_visualization)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# Find dependencies
set(dependencies_pkgs
    ament_cmake
    pluginlib
    rclcpp
    rviz_common
    # BT
    btcpp_ros2_interfaces
    # QT
    Qt5Core
    Qt5Gui
    Qt5Widgets
)

foreach(pkg IN LISTS dependencies_pkgs)
    find_package(${pkg} REQUIRED)
endforeach()


#####################################
############### Build ###############
#####################################

include_directories(include)

set(bt_panel behavior_tree_panel)

# Define ui file
set(${bt_panel}_uis
    resource/rviz2_panel.ui
    resource/behavior_tree_widget.ui
    resource/approval_widget.ui
)

# Generate header for ui
qt5_wrap_ui(${bt_panel}_uis_h ${${bt_panel}_uis})


set(${PROJECT_NAME}_headers_to_moc
    include/${PROJECT_NAME}/ui/behavior_tree_panel.hpp
    include/${PROJECT_NAME}/ui/behavior_tree_widget.hpp
    include/${PROJECT_NAME}/ui/behavior_approval_widget.hpp
)

foreach(header "${${PROJECT_NAME}_headers_to_moc}")
    qt5_wrap_cpp(${PROJECT_NAME}_moc_files "${header}")
endforeach()

set(${PROJECT_NAME}_source_files
    src/ui/behavior_tree_panel.cpp
    src/ui/behavior_tree_widget.cpp
    src/ui/behavior_approval_widget.cpp
)

add_library(${bt_panel} SHARED
    ${${PROJECT_NAME}_moc_files}
    ${${PROJECT_NAME}_source_files}
    ${${bt_panel}_uis_h}
)
target_include_directories(${bt_panel}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
set_target_properties(${bt_panel} PROPERTIES LINKER_LANGUAGE CXX)
ament_target_dependencies(${bt_panel} PUBLIC ${dependencies_pkgs})
target_link_libraries(${bt_panel}
    PUBLIC
    rviz_common::rviz_common
    "${cpp_typesupport_target}"
)

target_compile_definitions(${bt_panel} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(${bt_panel} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

#####################################
############## Install ##############
#####################################

pluginlib_export_plugin_description_file(rviz_common rviz2_plugin.xml)

install(
    TARGETS ${bt_panel}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Install headers
install(
    DIRECTORY include/
    DESTINATION include/
)

# Install resources
install(DIRECTORY
    resource
    DESTINATION share/${PROJECT_NAME}/
)

install(FILES rviz2_plugin.xml
    DESTINATION share
)

#####################################
############## Exports ##############
#####################################

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies_pkgs})
ament_export_libraries(${bt_panel})
ament_package()
