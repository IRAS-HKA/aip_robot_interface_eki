cmake_minimum_required (VERSION 3.5)
project(robot_interface_eki_lib)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(cpp_core REQUIRED) 

file (GLOB SOURCES src/*.cpp)
file (GLOB SOURCES_CORE src/core/*.cpp)
file (GLOB SOURCES_XML src/xml/*.cpp)
file (GLOB SOURCES_TINYXML2 third_party/tinyxml2/*.cpp)

include_directories(
  include
  third_party
  ${cpp_core_INCLUDE_DIRS}
)

add_library(robot_interface_eki_library SHARED
  ${SOURCES}
  ${SOURCES_CORE}
  ${SOURCES_XML}
  ${SOURCES_TINYXML2}
)

# target_include_directories(robot_interface_eki_library PUBLIC include
#   third_party
#   )

target_link_libraries(robot_interface_eki_library PUBLIC
   cpp_core::cpp_core_lib
  )


# Install library
install(DIRECTORY include/
    DESTINATION include)

install(TARGETS robot_interface_eki_library
    EXPORT export_robot_interface_eki_library
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

ament_export_include_directories(include)
ament_export_interfaces(export_robot_interface_eki_library HAS_LIBRARY_TARGET)
ament_export_dependencies(cpp_core)
ament_package()
