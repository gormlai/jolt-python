#cmake_minimum_required (VERSION 3.6.0)
cmake_minimum_required (VERSION 3.5.1)


set(CMAKE_BUILD_PARALLEL_LEVEL 8)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

add_subdirectory(src/lib)
add_subdirectory(src/cpp_test)
