#cmake_minimum_required (VERSION 3.6.0)
cmake_minimum_required (VERSION 3.5.1)

set(JPH_SHARED_LIBRARY 1)
include (Jolt.cmake)

set(CMAKE_BUILD_PARALLEL_LEVEL 8)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

set(SRC_DIR "src")
set(INCLUDE_DIR "include")
set(TEINCLUDE_DIR "${INCLUDE_DIR}/TinyEngine")

file(GLOB PRIVATE_SOURCES
  ${SRC_DIR}/*.cpp
  ${INCLUDE_DIR}
  ${TEINCLUDE_DIR}/TinyEngine.cpp
  ${TEINCLUDE_DIR}/imgui/*.cpp
  )

set(SOURCES ${PRIVATE_SOURCES})


set(PRIVATE_INCLUDE_DIRS
  ${SRC_DIR}
  ${INCLUDE_DIR}
)


if(MSVC)
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}  -DGUID_WINDOWS /MP /EHsc /FI ${SHARED_SRC_DIR}/Common.h")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /FI ${SHARED_SRC_DIR}/Common.h")
set(CMAKE_C_FLAGS_DEBUG          "-g -D_DEBUG -D_NO_DEBUG_HEAP=1 /Od /GS /RTC1 /Qpar /fp:fast /fp:except- /Zc:wchar_t- /WX-")
set(CMAKE_C_FLAGS_RELEASE        "-DNDEBUG /O2 /Ob2 /Oi /Qt /GT /GL /Gy /arch:SSE2 /fp:fast /fp:except- /Zc:wchar_t- ")
else()
set(CMAKE_C_FLAGS                "-Wall -std=c11 -ffast-math -Wl,-rpath,./")
set(CMAKE_C_FLAGS_DEBUG          "-g -D_DEBUG")
set(CMAKE_C_FLAGS_RELEASE        "-O3 -DNDEBUG -s")
set(CMAKE_CXX_FLAGS              "-Wall -Wl,-rpath,./")
set(CMAKE_CXX_FLAGS_DEBUG          "-g -D_DEBUG")
set(CMAKE_CXX_FLAGS_RELEASE        "-O3 -DNDEBUG -s")

endif()
set(SHARED_CXX_FLAGS " -DIMGUI_DEFINE_MATH_OPERATORS -DGLM_ENABLE_EXPERIMENTAL -fpermissive -DHAVE_STDIO_H")

if(MSVC)
set(SHARED_CXX_FLAGS "${SHARED_CXX_FLAGS} -D_WIN64 -D_UNICODE -DUNICODE -D_SILENCE_ALL_CXX17_DEPRECATION_WARNINGS")
elseif(APPLE)
elseif(ANDROID)
else()
  set(SHARED_CXX_FLAGS "${SHARED_CXX_FLAGS}")
endif()

set(SHARED_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SHARED_CXX_FLAGS}")
message(STATUS "SHARED_CXX_FLAGS")
message(STATUS "${SHARED_CXX_FLAGS}")
message(STATUS "CMAKE_CXX_FLAGS")
message(STATUS "${CMAKE_CXX_FLAGS}")


if(MSVC)
#    add_definitions(/FI "${SHARED_SRC_DIR}/Common.h")
else()
#    add_definitions(-include Common.h)
endif()

#linker paths
if(MSVC)
set(PRIVATE_LIB_DIRS
  ${SHARED_DIR}/Lib/x64/
  ${SHARED_DIR}/Lib/x64/${CMAKE_BUILD_TYPE}
)
else()
set(PRIVATE_LIB_DIRS
  ${SHARED_DIR}/Lib/linux
  ${SHARED_DIR}/Lib/linux/${CMAKE_BUILD_TYPE}
)
endif()

# private linker libs
if(MSVC)
set(PRIVATE_LIBS
)
else()
set(PRIVATE_LIBS
  SDL2
  SDL2_mixer
  GLEW
  boost_system
  boost_filesystem
  SDL2_ttf
  SDL2_image
  pthread
)

endif()


# public linker libs
if(MSVC)
else()
set(PUBLIC_LIBS
  GL
  pthread
  dl
)
endif()


set(LIBS ${PLATFORM_LIBS} ${PUBLIC_LIBS} ${PRIVATE_LIBS})

set(PROJECT_NAME "jolt-python")
project(${PROJECT_NAME})

message(STATUS "${PROJECT_NAME}")
message(STATUS "${PRIVATE_INCLUDE_DIRS}")



link_directories(${PRIVATE_LIB_DIRS})
add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE ${PRIVATE_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${LIBS} Jolt)

set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)
