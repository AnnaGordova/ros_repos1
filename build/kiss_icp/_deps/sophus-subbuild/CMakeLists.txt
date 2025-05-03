# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.22.1)

# We name the project and the target for the ExternalProject_Add() call
# to something that will highlight to the user what we are working on if
# something goes wrong and an error message is produced.

project(sophus-populate NONE)



include(ExternalProject)
ExternalProject_Add(sophus-populate
                     "UPDATE_DISCONNECTED" "False" "SYSTEM" "URL" "https://github.com/strasdat/Sophus/archive/refs/tags/1.22.10.tar.gz" "PATCH_COMMAND" "patch" "-p1" "<" "/home/anna/ros2_ws1/src/kiss-icp/cpp/kiss_icp/3rdparty/sophus/sophus.patch" "UPDATE_DISCONNECTED" "1"
                    SOURCE_DIR          "/home/anna/ros2_ws1/build/kiss_icp/_deps/sophus-src"
                    BINARY_DIR          "/home/anna/ros2_ws1/build/kiss_icp/_deps/sophus-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
                    USES_TERMINAL_DOWNLOAD  YES
                    USES_TERMINAL_UPDATE    YES
)


