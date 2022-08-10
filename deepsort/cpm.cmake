include(${CURRENT}/cmake/CPM.cmake)

# openjpeg-2.5, which is required by opencv
include_directories($/usr/include/openjpeg-2.5/)

# opencv-4.6
CPMAddPackage(
  NAME opencv
  GITHUB_REPOSITORY opencv/opencv
  GIT_TAG 4.6.0
  VERSION 4.6.0
  OPTIONS
    "CMAKE_BUILD_TYPE Release"
    "OPENCV_GENERATE_PKGCONFIG YES"
)
