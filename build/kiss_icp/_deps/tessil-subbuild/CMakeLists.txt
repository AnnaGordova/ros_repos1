# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.22.1)

# We name the project and the target for the ExternalProject_Add() call
# to something that will highlight to the user what we are working on if
# something goes wrong and an error message is produced.

project(tessil-populate NONE)



include(ExternalProject)
ExternalProject_Add(tessil-populate
                     "UPDATE_DISCONNECTED" "False" "SYSTEM" "URL" "https://github.com/Tessil/robin-map/archive/refs/tags/v1.2.1.tar.gz"
                    SOURCE_DIR          "/home/anna/ros2_ws1/build/kiss_icp/_deps/tessil-src"
                    BINARY_DIR          "/home/anna/ros2_ws1/build/kiss_icp/_deps/tessil-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
                    USES_TERMINAL_DOWNLOAD  YES
                    USES_TERMINAL_UPDATE    YES
)


