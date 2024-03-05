################################################################################
# Common Framework for Computer Graphics Courses at FI MUNI.
#
# Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
# All rights reserved.
#
# Module: OPENGL_4_5
################################################################################

# Creates the module.
visitlab_create_module(module_name)

# Specifies the include directories to use when compiling the module.
target_include_directories(${module_name} PUBLIC include include/scene)

# Collects the source files and specified them as target sources.
target_sources(
    ${module_name} 
    PUBLIC 
    "include/opengl_4_5_application.hpp"
    "include/scene_object_deprecated.hpp"
    "include/ubo_impl.hpp"
    "src/ubo/light_ubo.cpp"
    "src/ubo/ubo.cpp"
    "src/fbo_utils.cpp"
    "src/opengl_4_5_application.cpp"
    "src/program.cpp"
    "src/texture.cpp"
    "src/texture_utils.cpp"
 )