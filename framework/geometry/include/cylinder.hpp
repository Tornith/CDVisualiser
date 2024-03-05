// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#pragma once

#include "cylinder.inl"
#include "geometry.hpp"

/** This class represents a geometry for a default cylinder model. */
class Cylinder : public Geometry {
    // ----------------------------------------------------------------------------
    // Constructors
    // ----------------------------------------------------------------------------
public:
    /**
     * Creates a @link Cylinder whose center is in (0,0,0), its height is 2 (y coordinates are from -1 to 1), and its radius
     * is 0.5 (x/z coordinates are from -0.5 to 0.5)
     */
    Cylinder()
        : Geometry(GL_TRIANGLE_STRIP, 14, cylinder_vertices_count, cylinder_vertices, cylinder_indices_count, cylinder_indices) { }
};
