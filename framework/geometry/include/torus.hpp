// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#pragma once

#include "geometry.hpp"
#include "torus.inl"

/** This class represents a geometry for a default torus model. */
class Torus : public Geometry {
    // ----------------------------------------------------------------------------
    // Constructors
    // ----------------------------------------------------------------------------
public:
    /**
     * Creates a @link Torus whose center is in (0,0,0), its height is 1 (y coordinates are from -0.5 to 0.5), and its width
     * is 3 (x/z coordinates are from -1.5 to 1.5).
     */
    Torus()
        : Geometry(GL_TRIANGLE_STRIP, 14, torus_vertices_count, torus_vertices, torus_indices_count, torus_indices) { }
};
