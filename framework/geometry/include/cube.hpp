// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
//
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#pragma once

#include "cube.inl"
#include "geometry.hpp"
#include "glm/mat4x4.hpp"

/** This class represents a geometry for a default cube model. */
class Cube : public Geometry {
    // ----------------------------------------------------------------------------
    // Constructors
    // ----------------------------------------------------------------------------
  public:
    /**
     * Creates a @link Cube whose center is in (0,0,0) and the length of its side is 2 units (positions of its vertices are
     * from -1 to 1).
     */
    Cube() : Geometry(GL_TRIANGLES, 14, cube_vertices_count, cube_vertices, cube_indices_count, cube_indices) {}

    /**
     * Updates the position of vertices of this cube.
     *
     * @param 	left_top_front	  	The left top front point.
     * @param 	right_top_front   	The right top front point.
     * @param 	left_bottom_front 	The left bottom front point.
     * @param 	right_bottom_front	The right bottom front point.
     * @param 	left_top_back	  	The left top back point.
     * @param 	right_top_back	  	The right top back point.
     * @param 	left_bottom_back  	The left bottom back point.
     * @param 	right_bottom_back 	The right bottom back point.
     */
    void update(const glm::vec3& left_top_front, const glm::vec3& right_top_front, const glm::vec3& left_bottom_front,
                const glm::vec3& right_bottom_front, const glm::vec3& left_top_back, const glm::vec3& right_top_back,
                const glm::vec3& left_bottom_back, const glm::vec3& right_bottom_back);

    /**
     * Updates the position of the cube using two corner points.
     *
     * @param 	min	The left bottom front point.
     * @param 	max	The right top back point.
     */
    void update(const glm::vec3& min, const glm::vec3& max) {
        glm::vec3 left_top_front = glm::vec3(min.x, max.y, max.z);
        glm::vec3 right_top_front = glm::vec3(max.x, max.y, max.z);
        glm::vec3 left_bottom_front = glm::vec3(min.x, min.y, max.z);
        glm::vec3 right_bottom_front = glm::vec3(max.x, min.y, max.z);
        glm::vec3 left_top_back = glm::vec3(min.x, max.y, min.z);
        glm::vec3 right_top_back = glm::vec3(max.x, max.y, min.z);
        glm::vec3 left_bottom_back = glm::vec3(min.x, min.y, min.z);
        glm::vec3 right_bottom_back = glm::vec3(max.x, min.y, min.z);

        // Updates the geometry.
        update(left_top_front, right_top_front, left_bottom_front, right_bottom_front, left_top_back, right_top_back, left_bottom_back,
               right_bottom_back);
    }

    /**
     * Updates the position of the cube such that it encloses all 8 specified vertices (i.e., like a bounding box).
     *
     * @param 	points	The points to enclose.
     */
    void update(const glm::vec3 points[8]) {
        // Computes the correct ordering of the points
        glm::vec3 min = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
        glm::vec3 max = {std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()};
        for (int i = 0; i < 8; i++) {
            min = glm::min(min, points[i]);
            max = glm::max(max, points[i]);
        }

        // Updates the geometry.
        update(min, max);
    }

    /**
     * This method uses a specified projection matrix to set the model of this cube to enclose the view frustum.
     *
     * @param 	matrix	The projection matrix defining the view frustum.
     */
    void update(const glm::mat4& matrix) {
        glm::mat4 inverse = glm::inverse(matrix);
        glm::vec4 p1 = inverse * glm::vec4(-1, -1, -1, 1);
        glm::vec4 p2 = inverse * glm::vec4(-1, -1, 1, 1);
        glm::vec4 p3 = inverse * glm::vec4(-1, 1, -1, 1);
        glm::vec4 p4 = inverse * glm::vec4(-1, 1, 1, 1);
        glm::vec4 p5 = inverse * glm::vec4(1, -1, -1, 1);
        glm::vec4 p6 = inverse * glm::vec4(1, -1, 1, 1);
        glm::vec4 p7 = inverse * glm::vec4(1, 1, -1, 1);
        glm::vec4 p8 = inverse * glm::vec4(1, 1, 1, 1);
        update(p1, p2, p3, p4, p5, p6, p7, p8);
    }
};
