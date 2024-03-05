// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#pragma once

#include "geometry.hpp"
#include "teapot.inl"
#include "teapot_patch.inl"

/** This class represents a geometry for a default teapot model. */
class Teapot : public Geometry {
    // ----------------------------------------------------------------------------
    // Constructors
    // ----------------------------------------------------------------------------
public:
    /**
     * Creates a new @link Teapot such that the center of the bottom of its body is roughly in (0,0,0) and the radius of
     * the body is roughly 1. Its handle is in -X direction, its spout is in +X direction, and its lid is in +Y
     * direction.
     */
    Teapot()
        : Geometry(GL_TRIANGLE_STRIP, 14, teapot_vertices_count, teapot_vertices, teapot_indices_count, teapot_indices) { }
};

/** This class represents a geometry for a default teapot model using patches. */
class TeapotPatches : public Geometry {
    // ----------------------------------------------------------------------------
    // Constructors
    // ----------------------------------------------------------------------------
public:

    /**
     * Creates a new @link TeapotPatches such that the center of the bottom of its body is roughly in (0,0,0) and the
     * radius of the body is roughly 1. Its handle is in -X direction, its spout is in +X direction, and its lid is in
     * +Y direction. 
     * <p>
     * The teapot is defined using patches consisting of 16 vertices each.
     */
    TeapotPatches()
        : Geometry(
            GL_PATCHES,
            std::vector<float>(std::begin(teapot_patch_positions), std::end(teapot_patch_positions)),
            {/*indices*/},
            {/*normals*/},
            {/*colors*/},
            {/*texture cords*/},
            std::vector<float>(std::begin(teapot_patch_tangents), std::end(teapot_patch_tangents)),
            std::vector<float>(std::begin(teapot_patch_bitangents), std::end(teapot_patch_bitangents))
            ) { }
};
