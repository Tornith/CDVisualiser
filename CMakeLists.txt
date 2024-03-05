################################################################################
#
# Common Framework for Computer Graphics Courses at FI MUNI.
#
# Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
# All rights reserved.
#
# This is the main CMake file that defines project-wise settings.
#
################################################################################

# Sets the minimum required version of cmake.
cmake_minimum_required(VERSION 3.14)

# Sets the most important *_DIR variables.
set(VISITLAB_PROJECT_DIR ${CMAKE_CURRENT_LIST_DIR})
set(VISITLAB_MODULE_DIR "${CMAKE_CURRENT_LIST_DIR}/framework")
set(VISITLAB_COURSE_DIR "${CMAKE_CURRENT_LIST_DIR}/courses")

# Sets the name of the project, and stores it in the variable PROJECT_NAME.
# CXX specifies which programming language (i.e., C++) will be used.
project(ComputerGraphicsFramework CXX)

# Includes global utility macros and functions.
include(cmake/utils.cmake)

# Includes support for compiling courses.
include(cmake/courses.cmake)

# Includes support for compiling framework modules.
include(cmake/modules.cmake)

# Enable testing.
enable_testing()

# Initialize the CMake build process.
visitlab_prepare_build()

# Sets a list of courses build by this CMake.
visitlab_register_course(PB009)
visitlab_register_course(PV112)
visitlab_register_course(PA199)
visitlab_register_course(PA010)
visitlab_register_course(PA213)
visitlab_register_course(PV227)
visitlab_register_course(SBAPR)
visitlab_register_course(OTHER)

# Adds modules required by the courses above.
visitlab_add_modules(${visitlab_required_modules})

# Below is some code that can be used later if you need your own libraries outside vcpgk.
# include(FetchContent)
# FetchContent_Declare(range-v3
#   GIT_REPOSITORY https://github.com/ericniebler/range-v3.git
#   GIT_TAG 4989f3e9ff2efee1852942bb9328ef121369ba02
# )
# FetchContent_MakeAvailable(Range-v3)

set(CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/dist" CACHE STRING "Install path" FORCE)
set(CMAKE_INSTALL_RPATH "./")
