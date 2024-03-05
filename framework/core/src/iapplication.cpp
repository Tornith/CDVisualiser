// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#include "iapplication.h"
#include "configuration.h"

// ----------------------------------------------------------------------------
// Constructors
// ----------------------------------------------------------------------------
IApplication::IApplication(int initial_width, int initial_height, std::vector<std::string> arguments)
    : width(initial_width), height(initial_height), arguments(arguments), configuration(Configuration(arguments[0])) {

    framework_folder_path = configuration.get_path("framework_dir", "/framework");
    lecture_folder_path = configuration.get_path("lecture_dir", "/");
}

IApplication::~IApplication() = default;

// ----------------------------------------------------------------------------
// Input Events
// ----------------------------------------------------------------------------
void IApplication::on_resize(int width, int height) {
    this->width = std::max(width, 1);
    this->height = std::max(height, 1);
}

// ----------------------------------------------------------------------------
// Methods
// ----------------------------------------------------------------------------
//
//void IApplication::activate_viewport(GLint x, GLint y, GLsizei width, GLsizei height) const {
//    glViewport(x, y, width > 0 ? width : this->width, height > 0 ? height : this->height);
//}
//
//void IApplication::activate_viewport_grid(int x, int y, int grid_size_x, int grid_size_y) const {
//    const int viewport_width = static_cast<int>(width) / grid_size_x;
//    const int viewport_height = static_cast<int>(height) / grid_size_y;
//    const int viewport_x = x * viewport_width;
//    const int viewport_y = y * viewport_height;
//    activate_viewport(viewport_x, viewport_y, viewport_width, viewport_height);
//}
//
//void IApplication::prepare_rendering(int x, int y, int width, int height, GLint buffer, bool clear, bool depth_test) const {
//    activate_render_buffer(buffer, clear, depth_test);
//    activate_viewport(x, y, width, height);
//}
//
//void IApplication::prepare_rendering_grid(int x, int y, int grid_size_x, int grid_size_y, GLint buffer, bool clear, bool depth_test) const {
//    activate_render_buffer(buffer, clear, depth_test);
//    activate_viewport_grid(x, y, grid_size_x, grid_size_y);
//}
//
//void IApplication::upload_projection_view(const ShaderProgram& program, glm::mat4 projection, glm::mat4 view) const {
//    if (program.is_valid()) {
//        program.use();
//        program.uniform_matrix("projection", {projection});
//        program.uniform_matrix("view", {view});
//    }
//}

// ----------------------------------------------------------------------------
// Getters & Setters
// ----------------------------------------------------------------------------
size_t IApplication::get_width() const { return this->width; }

size_t IApplication::get_height() const { return this->height; }

std::filesystem::path IApplication::get_framework_folder_path() const { return this->framework_folder_path; }

void IApplication::set_window(GLFWwindow* window) { this->window = window; }
