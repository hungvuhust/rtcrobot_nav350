cmake_minimum_required(VERSION 3.8)
project(rtcrobot_nav350)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic )
endif()

set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -fPIC" )
set( CMAKE_C_FLAGS  "${CMAKE_C_FLAGS}  -fPIC" )


# find ROS2 dependencies packages
# ==== Find dependencies ====
find_package(ament_cmake REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  angles 
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  geometry_msgs
  nav_msgs
  rtcrobot_interfaces
)

foreach(pkg ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${pkg} REQUIRED)
endforeach()
# ============================

find_package(glog REQUIRED)
find_package(spdlog REQUIRED)


# ==== Include directories ====
include_directories(
  include
  ${glog_INCLUDE_DIRS}
)
# ============================

set(NAV350_COMPONENTS
  nav350_components
)

# ==== Library ==== (Ros2 component)
add_library(${NAV350_COMPONENTS} SHARED
  src/sick_nav350.cpp
  src/nav350_node.cpp
)
target_include_directories(${NAV350_COMPONENTS}
  PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${NAV350_COMPONENTS}
  glog::glog
  spdlog::spdlog
)
ament_target_dependencies(${NAV350_COMPONENTS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
# Export component for ros2
rclcpp_components_register_nodes(${NAV350_COMPONENTS}
  PLUGIN "rtcrobot_nav350::Nav350Node"
  EXECUTABLE nav350_node
)
ament_export_libraries(export_${NAV350_COMPONENTS})
# ============================

# Execuable
add_executable(nav350_node
  src/nav350_node.cpp
)
target_link_libraries(nav350_node
  ${NAV350_COMPONENTS}
)
ament_target_dependencies(nav350_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# ==== Install ====
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${NAV350_COMPONENTS}
  EXPORT export_${NAV350_COMPONENTS}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install (TARGETS nav350_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
