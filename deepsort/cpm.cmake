include(${CURRENT}/cmake/CPM.cmake)

# openjpeg-2.5, which is required by opencv
CPMAddPackage(
  NAME openjpeg
  GITHUB_REPOSITORY uclouvain/openjpeg
  GIT_TAG v2.5.0
  VERSION v2.5.0
  OPTIONS
  "BUILD_JAVA OFF"
  "BUILD_PKGCONFIG_FILES ON"
  "CMAKE_BUILD_TYPE Release"
  "CMAKE_INSTALL_PREFIX /usr"
)

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
