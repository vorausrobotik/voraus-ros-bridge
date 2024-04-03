cmake_minimum_required(VERSION 3.20)
project(voraus_ros_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(open62541pp CONFIG REQUIRED)

add_executable(voraus_ros_bridge
  src/voraus_robot_control_interface.cpp
  src/voraus_ros_bridge_node.cpp
  src/voraus_ros_bridge.cpp
)

ament_target_dependencies(voraus_ros_bridge PUBLIC rclcpp sensor_msgs)

target_include_directories(voraus_ros_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(voraus_ros_bridge PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(voraus_ros_bridge PUBLIC open62541pp::open62541pp)

install(TARGETS voraus_ros_bridge
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  # find_package(ament_cmake_clang_tidy REQUIRED)

  ament_lint_cmake()
  ament_xmllint()
  ament_clang_format(CONFIG_FILE ../.clang-format)
  # ament_clang_tidy(CONFIG_FILE ../.clang-tidy ${CMAKE_BINARY_DIR})
endif()

ament_package()
