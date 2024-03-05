################################################################################
# Common Framework for Computer Graphics Courses at FI MUNI.
#
# Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
# All rights reserved.
#
# Module: GEOMETRY_4_5
################################################################################

# Creates the module.
visitlab_create_module(module_name)

# Collects the source files and specified them as target sources.
target_sources(
    ${module_name} 
    PUBLIC
    src/cube.cpp
    src/geometry.cpp
)