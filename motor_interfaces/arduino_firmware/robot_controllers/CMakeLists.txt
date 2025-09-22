cmake_minimum_required(VERSION 3.8)
project(robot_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(controller_interface REQUIRED)

include_directories(include)

# Library
add_library(${PROJECT_NAME} SHARED
  src/robot_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC include)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  controller_interface
  pluginlib
  example_interfaces
)

# Export plugin description (ensure file exists in resource/)
pluginlib_export_plugin_description_file(controller_interface resource/controller_plugin.xml)

install(FILES resource/controller_plugin.xml
  DESTINATION share/${PROJECT_NAME})


# Install targets and headers
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  example_interfaces
  
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
