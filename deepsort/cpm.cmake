include(${CURRENT}/cmake/CPM.cmake)

CPMAddPackage(
  NAME OpenJPEG
  GITHUB_REPOSITORY uclouvain/openjpeg
  GIT_TAG v2.5.0
  VERSION v2.5.0
  OPTIONS
  "BUILD_JAVA OFF"
  "BUILD_PKGCONFIG_FILES ON"
  "CMAKE_BUILD_TYPE Release"
)
include_directories(${OPENJPEG_INCLUDE_DIRS})

CPMAddPackage(
  NAME opencv
  GITHUB_REPOSITORY opencv/opencv
  GIT_TAG 4.6.0
  VERSION 4.6.0
  OPTIONS
    "CMAKE_BUILD_TYPE Release"
    "OPENCV_GENERATE_PKGCONFIG YES"
)
