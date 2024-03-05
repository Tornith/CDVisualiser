################################################################################
# Common Framework for Computer Graphics Courses at FI MUNI.
#
# Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
# All rights reserved.
#
# Module: CORE
################################################################################

# Creates the module.
visitlab_create_module(module_name)

if(MSVC)
    # Forces MSVC to define correct values for __cplusplus macro.
    target_compile_options(${module_name} PUBLIC "/Zc:__cplusplus")
endif()

# Finds the external libraries and load their settings.
find_package(glfw3 CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(toml11 CONFIG REQUIRED)
find_package(GTest CONFIG REQUIRED)

# Specifies external libraries to link with the module.
target_link_libraries(${module_name} PUBLIC glad::glad glfw toml11::toml11 GTest::gtest)

# Specifies the include directories to use when compiling the module.
target_include_directories(${module_name}
    PUBLIC include include/opengl include/scene include/geometry geometries
)

# Collects the source files and specified them as target sources.
target_sources(${module_name} PRIVATE 
    "include/code_utils.h"    
    "include/iapplication.h"
    "include/manager.h"
    "include/configuration.h"
    "src/iapplication.cpp"
    "src/manager.cpp"
    "src/configuration.cpp"
)