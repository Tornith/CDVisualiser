// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
//
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#include "gui_application.h"

// ----------------------------------------------------------------------------
// Constructors & Destructors
// ----------------------------------------------------------------------------

GUIApplication::GUIApplication(int initial_width, int initial_height, std::vector<std::string> arguments)
    : IApplication(initial_width, initial_height, arguments) {}

GUIApplication::~GUIApplication() = default;

// ----------------------------------------------------------------------------
// Input Events
// ----------------------------------------------------------------------------

void GUIApplication::on_mouse_move(double x, double y) {
    // Consumes the event when interacting with GUI.
    if (!ImGui::GetIO().WantCaptureMouse) {
        camera.on_mouse_move(x, y);
    }
}

void GUIApplication::on_mouse_button(int button, int action, int mods) {
    if (!ImGui::GetIO().WantCaptureMouse) {
        camera.on_mouse_button(button, action, mods);
    }
}

void GUIApplication::on_key_pressed(int key, int scancode, int action, int mods) {
    if (!ImGui::GetIO().WantCaptureMouse && action == GLFW_PRESS) {
        switch (key) {
        case GLFW_KEY_R:
            compile_shaders();
            break;
        }
    }
}
