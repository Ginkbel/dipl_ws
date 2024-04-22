cmake_minimum_required(VERSION 3.5)
project(nmpc_astro)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(casadi REQUIRED)

set(dependencies 
  rclcpp 
  geometry_msgs 
  sensor_msgs 
  nav_msgs
  tf2 
  tf2_ros 
  casadi)

include_directories(include)

add_executable(${PROJECT_NAME}_node src/main.cpp src/nmpc_ros.cpp src/nmcp.cpp)

add_library(${PROJECT_NAME}_core SHARED 
  src/nmcp.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  ${dependencies}
)

ament_target_dependencies(${PROJECT_NAME}_core
  ${dependencies}
)

target_link_libraries(${PROJECT_NAME}_node casadi)
target_link_libraries(${PROJECT_NAME}_core casadi)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${nonlinear_programs_library}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(${dependencies})
ament_package()