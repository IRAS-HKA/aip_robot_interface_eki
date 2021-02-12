cmake_minimum_required (VERSION 3.1 FATAL_ERROR)
project(robot_interface_eki_lib)

# message("${CMAKE_INSTALL_PREFIX}")

find_package(ament_cmake REQUIRED)
find_package(ros_core REQUIRED) 

file (GLOB SOURCES src/*.cpp)
file (GLOB SOURCES_CORE src/core/*.cpp)
file (GLOB SOURCES_XML src/xml/*.cpp)
file (GLOB SOURCES_TINYXML2 third_party/tinyxml2/*.cpp)


add_library(robot_interface_eki_lib SHARED
  ${SOURCES}
  ${SOURCES_CORE}
  ${SOURCES_XML}
  ${SOURCES_TINYXML2}
)

target_include_directories(robot_interface_eki_lib PUBLIC include
  third_party
  )

target_link_libraries(robot_interface_eki_lib PUBLIC
   ros_core::cpp_core_lib
  )


# Install library
install(DIRECTORY include/
    DESTINATION include)

install(TARGETS cpp_core_lib
    EXPORT export_cpp_core_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

ament_export_include_directories(include)
ament_export_interfaces(export_cpp_core_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(cpp_core_lib)
ament_package()
