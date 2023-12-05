cmake_minimum_required(VERSION 3.5)
project(cpp_core)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

file (GLOB SOURCES src/*.cpp)
file (GLOB SOURCES_CORE src/core/*.cpp)
file (GLOB SOURCES_EVENTS src/events/*.cpp)
file (GLOB SOURCES_TYPES src/types/*.cpp)
file (GLOB SOURCES_TOOLS src/tools/*.cpp)

add_library(cpp_core_lib SHARED
  ${SOURCES}
  ${SOURCES_CORE}
  ${SOURCES_EVENTS}
  ${SOURCES_TYPES}
  ${SOURCES_TOOLS}
)

include_directories(
  include
)

find_package(Threads)
find_package(ament_cmake REQUIRED)

target_link_libraries(cpp_core_lib PUBLIC
  ${CMAKE_THREAD_LIBS_INIT}
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
ament_export_dependencies(Threads)

ament_package()