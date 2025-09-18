# cmake_minimum_required(VERSION 3.8)
# project(robot_hardware_interface)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_lifecycle REQUIRED)
# find_package(hardware_interface REQUIRED)
# find_package(pluginlib REQUIRED)
# find_package(dynamixel_sdk REQUIRED)

# include_directories(include)

# add_library(
#   ${PROJECT_NAME}
#   SHARED  
#   src/mobile_base_hardware_interface.cpp
# )


# add_library(
#   ${PROJECT_NAME}
#   SHARED  
#   src/.
#   src/arm_hardware_interface.cpp
# )


# ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_lifecycle hardware_interface pluginlib dynamixel_sdk)

# pluginlib_export_plugin_description_file(hardware_interface robot_hardware_interface.xml)
# install(FILES robot_hardware_interface.xml DESTINATION share/${PROJECT_NAME})


# install(TARGETS ${PROJECT_NAME}
# DESTINATION lib
# )

# install(
#   DIRECTORY include
#   DESTINATION include
# )

# ament_export_libraries(${PROJECT_NAME})

# ament_export_dependencies(
#   hardware_interface
#   pluginlib
#   rclcpp 
#   rclcpp_lifecycle
# )


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_copyright_FOUND TRUE)
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()



cmake_minimum_required(VERSION 3.8)
project(robot_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(dynamixel_sdk REQUIRED)

# Library
add_library(${PROJECT_NAME} SHARED
  src/mobile_base_hardware_interface.cpp
  src/arm_hardware_interface.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC include)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  dynamixel_sdk
)

# Export plugin description (ensure file exists in resource/)
pluginlib_export_plugin_description_file(hardware_interface resource/robot_hardware_interface.xml)
install(FILES resource/robot_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

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
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  dynamixel_sdk
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
