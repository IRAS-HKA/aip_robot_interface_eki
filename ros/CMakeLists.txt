cmake_minimum_required(VERSION 3.5)
project(robot_interface_eki)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#IMPORTANT
# add_subdirectory (../lib robot_interface_eki_lib)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(cpp_core REQUIRED)
find_package(robot_interface_eki_lib REQUIRED)

include_directories(
  include
  ${cpp_core_INCLUDE_DIRS}
  ${robot_interface_eki_lib_INCLUDE_DIRS}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Move.srv"
  "srv/Grip.srv"
 )


add_executable(Server src/Server.cpp)
ament_target_dependencies(Server
  rclcpp std_srvs cpp_core robot_interface_eki_lib)

target_link_libraries(Server
  cpp_core::cpp_core_lib
  robot_interface_eki_lib::robot_interface_eki_library
)

rosidl_target_interfaces(Server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install Python modules
ament_python_install_package(scripts)

# Install launch files.
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/)

install(TARGETS
  Server
  DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/client.py
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
