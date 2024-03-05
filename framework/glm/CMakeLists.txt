################################################################################
# Common Framework for Computer Graphics Courses at FI MUNI.
#
# Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
# All rights reserved.
#
# Module: GLM
################################################################################

# Creates the module.
visitlab_create_module(module_name)

# Finds the external libraries and load their settings.
find_package(glm CONFIG REQUIRED)

# Specifies external libraries to link with the module.
if (TARGET glm)
    target_link_libraries(${module_name} PUBLIC glm)
endif()
if (TARGET glm::glm)
    target_link_libraries(${module_name} PUBLIC glm::glm)
endif()

# Specifies the include directories to use when compiling the module.
target_include_directories(${module_name} PUBLIC include)

# A hack to build the module witout any actual *.cpp file.
target_sources(${module_name} PRIVATE "include/glm_headers.hpp" "src/empty.cpp")