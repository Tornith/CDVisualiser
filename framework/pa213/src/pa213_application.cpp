// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#include "pa213_application.h"

// ----------------------------------------------------------------------------
// Constructors & Destructors
// ----------------------------------------------------------------------------

PA213Application::PA213Application(int initial_width, int initial_height, std::vector<std::string> arguments)
    : OpenGLApplication(initial_width, initial_height, arguments) {
    // The default shader paths assume the standard file structure.
    display_texture_program = ShaderProgram(framework_shaders_path / "full_screen_quad.vert", framework_shaders_path / "display_texture.frag");
    glCreateVertexArrays(1, &base_vao);
}

PA213Application::~PA213Application() = default;

// ----------------------------------------------------------------------------
// Methods
// ----------------------------------------------------------------------------
void PA213Application::activate_render_buffer(GLint buffer, bool clear, bool depth_test) const {
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

void PA213Application::activate_viewport(GLint x, GLint y, GLsizei width, GLsizei height) const {
    glViewport(x, y, width > 0 ? width : this->width, height > 0 ? height : this->height);
}

void PA213Application::activate_viewport_grid(int x, int y, int grid_size_x, int grid_size_y) const {
    const int viewport_width = static_cast<int>(width) / grid_size_x;
    const int viewport_height = static_cast<int>(height) / grid_size_y;
    const int viewport_x = x * viewport_width;
    const int viewport_y = y * viewport_height;
    activate_viewport(viewport_x, viewport_y, viewport_width, viewport_height);
}

void PA213Application::prepare_rendering(int x, int y, int width, int height, GLint buffer, bool clear, bool depth_test) const {
    activate_render_buffer(buffer, clear, depth_test);
    activate_viewport(x, y, width, height);
}

void PA213Application::prepare_rendering_grid(int x, int y, int grid_size_x, int grid_size_y, GLint buffer, bool clear, bool depth_test) const {
    activate_render_buffer(buffer, clear, depth_test);
    activate_viewport_grid(x, y, grid_size_x, grid_size_y);
}

void PA213Application::upload_projection_view(const ShaderProgram& program, glm::mat4 projection, glm::mat4 view) const {
    if (program.is_valid()) {
        program.use();
        program.uniform_matrix("projection", {projection});
        program.uniform_matrix("view", {view});
    }
}

void PA213Application::display_texture(const Texture& texture, bool depth_texture) {
    if (display_texture_program.is_valid()) {
        display_texture_program.use();
        display_texture_program.uniform("depth_texture", depth_texture);

        // Binds the shadow texture and sets its parameters
        texture.bind(0);

        // Binds an empty basic VAO as we do not need any state
        glBindVertexArray(base_vao);

        // Calls a draw command with 3 vertices that are generated in vertex shader
        glDrawArrays(GL_TRIANGLES, 0, 3);
    }
}
