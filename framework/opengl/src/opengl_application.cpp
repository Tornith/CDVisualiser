// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
//
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#include "opengl_application.hpp"

// ----------------------------------------------------------------------------
// Constructors
// ----------------------------------------------------------------------------
OpenGLApplication::OpenGLApplication(int initial_width, int initial_height, std::vector<std::string> arguments)
    : GUIApplication(initial_width, initial_height, arguments) {
    framework_shaders_path = framework_folder_path / "opengl" / "shaders";
    framework_textures_path = framework_folder_path / "opengl" / "textures";
    lecture_shaders_path = lecture_folder_path / "shaders";
    lecture_textures_path = lecture_folder_path / "textures";

    // Initializes the materials.
    red_material_ubo.set_material(PhongMaterialData(glm::vec3(1.0f, 0.0f, 0.0f), true, 200.0f, 1.0f));
    green_material_ubo.set_material(PhongMaterialData(glm::vec3(0.0f, 1.0f, 0.0f), true, 200.0f, 1.0f));
    blue_material_ubo.set_material(PhongMaterialData(glm::vec3(0.0f, 0.0f, 1.0f), true, 200.0f, 1.0f));
    cyan_material_ubo.set_material(PhongMaterialData(glm::vec3(0.0f, 1.0f, 1.0f), true, 200.0f, 1.0f));
    magenta_material_ubo.set_material(PhongMaterialData(glm::vec3(1.0f, 0.0f, 1.0f), true, 200.0f, 1.0f));
    yellow_material_ubo.set_material(PhongMaterialData(glm::vec3(1.0f, 1.0f, 0.0f), true, 200.0f, 1.0f));
    white_material_ubo.set_material(PhongMaterialData(glm::vec3(1.0f), true, 200.0f, 1.0f));
    black_material_ubo.set_material(PhongMaterialData(glm::vec3(0.0f), 1.0f, 0.0f, false));
    gray_material_ubo.set_material(PhongMaterialData(glm::vec3(0.7f), 1.0f, 200.0f, 1.0f));

    // Uploads the OpenGL materials.
    red_material_ubo.update_opengl_data();
    green_material_ubo.update_opengl_data();
    blue_material_ubo.update_opengl_data();
    cyan_material_ubo.update_opengl_data();
    magenta_material_ubo.update_opengl_data();
    yellow_material_ubo.update_opengl_data();
    white_material_ubo.update_opengl_data();
    black_material_ubo.update_opengl_data();
    gray_material_ubo.update_opengl_data();

    OpenGLApplication::compile_shaders();
}

OpenGLApplication::~OpenGLApplication() = default;

// ----------------------------------------------------------------------------
// Methods
// ----------------------------------------------------------------------------

void OpenGLApplication::compile_shaders() {
    // The default shader paths assume the standard file structure.
    // TODO: replace the original shaders in the framework with the new versions from PV227.
    default_unlit_program = ShaderProgram(framework_shaders_path / "object.vert", framework_shaders_path / "unlit.frag");
    default_lit_program = ShaderProgram(framework_shaders_path / "object.vert", framework_shaders_path / "lit.frag");
}

void OpenGLApplication::activate_render_buffer(GLint buffer, bool clear, bool depth_test) {
    // Binds the frame buffer.
    glBindFramebuffer(GL_FRAMEBUFFER, buffer);

    // Clears the color and depth attachments (this is also fine if there is no color attachment).
    if (clear) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }
    // Activates (or deactivates) the depth test.
    if (depth_test) {
        glEnable(GL_DEPTH_TEST);
    } else {
        glDisable(GL_DEPTH_TEST);
    }
}