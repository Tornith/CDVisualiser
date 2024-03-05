// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#include "pv227_application.hpp"

// ----------------------------------------------------------------------------
// Constructors & Destructors
// ----------------------------------------------------------------------------

PV227Application::PV227Application(int initial_width, int initial_height, std::vector<std::string> arguments)
    : OpenGL45Application(initial_width, initial_height, arguments) {
    PV227Application::compile_shaders();
}

PV227Application::~PV227Application() = default;