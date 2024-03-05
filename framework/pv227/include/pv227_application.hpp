// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
//
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#pragma once
#include "capsule.hpp"
#include "cube.hpp"
#include "cylinder.hpp"
#include "phong_material_ubo.hpp"
#include "sphere.hpp"
#include "teapot.hpp"
#include "torus.hpp"
#include "opengl_4_5_application.hpp"

class PV227Application : public OpenGL45Application {
    // ----------------------------------------------------------------------------
    // Constructors & Destructors
    // ----------------------------------------------------------------------------
public:
    /**
     * Constructs a new @link PV227Application with a custom width and height.
     *
     * @param 	initial_width 	The initial width of the window.
     * @param 	initial_height	The initial height of the window.
     * @param 	arguments	  	The command line arguments used to obtain the application directory.
     */
    PV227Application(int initial_width, int initial_height, std::vector<std::string> arguments = {});

    /** The destructor that releases all resources. */
    ~PV227Application() override;
};
