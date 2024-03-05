// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#include "opengl_4_5_application.hpp"

// ----------------------------------------------------------------------------
// Constructors
// ----------------------------------------------------------------------------
OpenGL45Application::OpenGL45Application(int initial_width, int initial_height, std::vector<std::string> arguments)
    : OpenGLApplication(initial_width, initial_height, arguments) {

    // Creates the empty VAO.
    glCreateVertexArrays(1, &empty_vao);

    // Creates the query object.
    glCreateQueries(GL_TIME_ELAPSED, 1, &render_time_query);
}