// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#include "camera.h"
#include "glm/ext/matrix_transform.inl"
#include <GLFW/glfw3.h>
#include <math.h>

// ----------------------------------------------------------------------------
// Variables
// ----------------------------------------------------------------------------
const float Camera::min_elevation = -1.5f;
const float Camera::max_elevation = 1.5f;
const float Camera::min_distance = 1.0f;
const float Camera::angle_sensitivity = 0.008f;
const float Camera::zoom_sensitivity = 0.003f;

// ----------------------------------------------------------------------------
// Constructors
// ----------------------------------------------------------------------------
Camera::Camera(bool switch_axes)
    : angle_direction(0.0f), angle_elevation(0.0f), distance(10.0f), last_x(0), last_y(0), is_rotating(false), is_zooming(false),
      switch_axes(switch_axes) {
    update_eye_pos();
}

// ----------------------------------------------------------------------------
// Methods
// ----------------------------------------------------------------------------
void Camera::update_eye_pos() {
    if (switch_axes) {
        eye_position.x = distance * cosf(angle_elevation) * sinf(angle_direction);
        eye_position.z = distance * sinf(angle_elevation);
        eye_position.y = distance * cosf(angle_elevation) * cosf(angle_direction);
    } else {
        eye_position.x = distance * cosf(angle_elevation) * -sinf(angle_direction);
        eye_position.y = distance * sinf(angle_elevation);
        eye_position.z = distance * cosf(angle_elevation) * cosf(angle_direction);
    }
}

// ----------------------------------------------------------------------------
// Input Events
// ----------------------------------------------------------------------------
void Camera::on_mouse_move(double x, double y) {
    float dx = static_cast<float>(x - last_x);
    float dy = static_cast<float>(y - last_y);
    last_x = static_cast<int>(x);
    last_y = static_cast<int>(y);

    if (is_rotating) {
        angle_direction += dx * angle_sensitivity;
        angle_elevation += dy * angle_sensitivity;

        // Clamps the results.
        angle_elevation = glm::clamp(angle_elevation, min_elevation, max_elevation);
    }
    if (is_zooming) {
        distance *= (1.0f + dy * zoom_sensitivity);

        // Clamps the results.
        if (distance < min_distance)
            distance = min_distance;
    }
    update_eye_pos();
}

void Camera::on_mouse_button(int button, int action, int mods) {
    // Left mouse button affects the angles.
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            is_rotating = true;
        } else {
            is_rotating = false;
        }
    }
    // Right mouse button affects the zoom.
    if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        if (action == GLFW_PRESS) {
            is_zooming = true;
        } else {
            is_zooming = false;
        }
    }
}

// ----------------------------------------------------------------------------
// Getters & Setters
// ----------------------------------------------------------------------------
void Camera::set_eye_position(float angle_direction, float angle_elevation, float distance) {
    this->angle_direction = angle_direction;
    this->angle_elevation = angle_elevation;
    this->distance = distance;
    update_eye_pos();
}

glm::vec3 Camera::get_eye_position() const { return eye_position; }

glm::mat4x4 Camera::get_view_matrix() const {
    return lookAt(eye_position, glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
}
