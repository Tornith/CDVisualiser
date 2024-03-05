// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#pragma once

#include "geometry.hpp"
#include "sphere.inl"

/** This class represents a geometry for a default sphere model. */
class Sphere : public Geometry {
    // ----------------------------------------------------------------------------
    // Constructors
    // ----------------------------------------------------------------------------
public:
    /**
     * Creates a @link Sphere whose center is in (0,0,0) and its radius is 1 (positions of its vertices are from -1 to 1).
     */
    Sphere()
        : Geometry(GL_TRIANGLE_STRIP, 14, sphere_vertices_count, sphere_vertices, sphere_indices_count, sphere_indices) { }
};
