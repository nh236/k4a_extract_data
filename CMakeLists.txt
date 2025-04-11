cmake_minimum_required(VERSION 3.9.0)

cmake_policy(SET CMP0048 NEW)


add_definitions(-std=c++11)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

project(k4a_extract_data LANGUAGES C CXX
    VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)
set(CMAKE_BUILD_TYPE RELEASE)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

#find_package(GFlags)
#find_package(Glog)
FIND_PACKAGE(k4a REQUIRED)
FIND_PACKAGE(k4arecord REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS}})

# These specific settings tell the loader to search the directory of the
# executable for shared objects. This is done on Linux to emulate the default
# behavior of the Windows loader, which searches for DLLs in the path of the
# executable.
if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Linux")
    set(CMAKE_BUILD_RPATH "\$ORIGIN")
endif()


add_executable(k4a_extract_data src/main.cpp include/utils.h src/utils.cpp src/process_images.cpp include/process_images.h)



target_include_directories(k4a_extract_data PRIVATE include)
# Dependencies of this library
target_link_libraries(k4a_extract_data PRIVATE 
    k4a
    k4arecord
    ${OpenCV_LIBS}
#    ${GFLAGS_LIBRARY}
#    ${GLOG_LIBRARY}
    stdc++fs
    )
