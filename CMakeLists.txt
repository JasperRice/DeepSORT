cmake_minimum_required(VERSION 3.14)

# Project
set(DEBUG Debug)
project(${DEBUG} VERSION 1.0)

# Standard
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 11)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin/debug)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin/release)
set(CURRENT ${CMAKE_CURRENT_SOURCE_DIR})

# Debug
include(CMakePrintHelpers)

# Subdirectories
add_subdirectory(${CURRENT}/deepsort)
add_subdirectory(${CURRENT}/filter)
