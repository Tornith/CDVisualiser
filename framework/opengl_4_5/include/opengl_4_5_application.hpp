// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
//
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#pragma once
#include "opengl_application.hpp"
#include "capsule.hpp"
#include "cube.hpp"
#include "cylinder.hpp"
#include "phong_material_ubo.hpp"
#include "sphere.hpp"
#include "teapot.hpp"
#include "torus.hpp"

class OpenGL45Application : public OpenGLApplication {

        // ----------------------------------------------------------------------------
    // Variables
    // ----------------------------------------------------------------------------
  protected:
    /** The OpenGL query object that we use to get render time of one frame. */
    GLuint render_time_query;

    /** The current FPS on GPU. */
    float fps_gpu;

    // ----------------------------------------------------------------------------
    // Constructors & Destructors
    // ----------------------------------------------------------------------------
  public:
    /**
     * Constructs a new @link OpenGL45Application with a custom width and height.
     *
     * @param 	initial_width 	The initial width of the window.
     * @param 	initial_height	The initial height of the window.
     * @param 	arguments	  	The command line arguments used to obtain the application directory.
     */
    OpenGL45Application(int initial_width, int initial_height, std::vector<std::string> arguments);

    /** The destructor that releases all resources. */
    virtual ~OpenGL45Application() = default;
};
