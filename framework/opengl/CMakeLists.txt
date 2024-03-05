################################################################################
# Common Framework for Computer Graphics Courses at FI MUNI.
#
# Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
# All rights reserved.
#
# Module: OPENGL
################################################################################

# Creates the module.
visitlab_create_module(module_name)

# Specifies the include directories to use when compiling the module.
target_include_directories(${module_name} PUBLIC include include/ubo)

# Collects the source files and specified them as target sources.
target_sources(
    ${module_name} 
    PRIVATE 
        "include/color.hpp"
        "include/fbo_utils.hpp"
        "include/opengl_application.hpp"
        "include/opengl_object.hpp"
        "include/opengl_utils.hpp"
        "include/program.hpp"
        "include/program_map.hpp"
        "include/scene_object.hpp"
        "include/shader.hpp"
        "include/shader_utils.hpp"
        "include/texture.hpp"
        "include/texture_utils.hpp"

        "include/ubo/camera_ubo.hpp"
        "include/ubo/light_ubo.hpp"
        "include/ubo/material_ubo.hpp"
        "include/ubo/model_ubo.hpp" 
        "include/ubo/pbr_material_ubo.hpp"
        "include/ubo/phong_material_ubo.hpp"
        "include/ubo/ubo.hpp"

        "src/color.cpp"
        "src/fbo_utils.cpp"
        "src/opengl_application.cpp"
        "src/program.cpp"
        "src/program_map.cpp"
        "src/shader.cpp"
        "src/texture.cpp"
        "src/texture_utils.cpp"
)