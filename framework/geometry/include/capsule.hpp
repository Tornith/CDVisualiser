// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#pragma once

#include "geometry.hpp"
#include "capsule.inl"

/** This class represents a geometry for a default capsule model. */
class Capsule : public Geometry {
    // ----------------------------------------------------------------------------
    // Constructors
    // ----------------------------------------------------------------------------
public:
    /**
     * Creates a @link Capsule whose center is in (0,0,0), its height is 3 (y coordinates are from -1.5 to 1.5), and its
     * radius is 0.5 (x/z coordinates are from -0.5 to 0.5).
     */
    Capsule() : Geometry(GL_TRIANGLE_STRIP, 14, capsule_vertices_count, capsule_vertices, capsule_indices_count, capsule_indices) {}
};
