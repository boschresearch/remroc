cmake_minimum_required(VERSION 3.8)
project(remroc_mapf_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(remroc_interfaces REQUIRED)
find_package(remroc_cbs_library REQUIRED)
# find_package(cmath REQUIRED)

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

add_executable(mapf_solver_server src/mapf_solver_server.cpp)
ament_target_dependencies(
  mapf_solver_server 
  rclcpp 
  example_interfaces 
  std_msgs 
  geometry_msgs
  remroc_interfaces 
  remroc_cbs_library
  # cmath
  )

install(TARGETS 
  mapf_solver_server
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
