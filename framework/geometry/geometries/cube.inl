// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
//
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#pragma once

#include "glm/vec3.hpp"
#include <array>

const int cube_vertices_count = 24;
const float cube_vertices[cube_vertices_count * 14] =
{
//       Position                 Normal             Tex Coord          Tangent                BiTangent
// Front face
	-1.0f, 1.0f, 1.0f,		 0.0f, 0.0f, 1.0f,		0.0f, 1.0f,		1.0f, 0.0f, 0.0f,		0.0f, 1.0f, 0.0f,
	-1.0f,-1.0f, 1.0f,		 0.0f, 0.0f, 1.0f,		0.0f, 0.0f,		1.0f, 0.0f, 0.0f,		0.0f, 1.0f, 0.0f,
	 1.0f,-1.0f, 1.0f,		 0.0f, 0.0f, 1.0f,		1.0f, 0.0f,		1.0f, 0.0f, 0.0f,		0.0f, 1.0f, 0.0f,
	 1.0f, 1.0f, 1.0f,		 0.0f, 0.0f, 1.0f,		1.0f, 1.0f,		1.0f, 0.0f, 0.0f,		0.0f, 1.0f, 0.0f,
// Right face
	 1.0f, 1.0f, 1.0f,		 1.0f, 0.0f, 0.0f,		0.0f, 1.0f,		0.0f, 0.0f, -1.0f,		0.0f, 1.0f, 0.0f,
	 1.0f,-1.0f, 1.0f,		 1.0f, 0.0f, 0.0f,		0.0f, 0.0f,		0.0f, 0.0f, -1.0f,		0.0f, 1.0f, 0.0f,
	 1.0f,-1.0f,-1.0f,		 1.0f, 0.0f, 0.0f,		1.0f, 0.0f,		0.0f, 0.0f, -1.0f,		0.0f, 1.0f, 0.0f,
	 1.0f, 1.0f,-1.0f,		 1.0f, 0.0f, 0.0f,		1.0f, 1.0f,		0.0f, 0.0f, -1.0f,		0.0f, 1.0f, 0.0f,
// Back face
	 1.0f, 1.0f,-1.0f,		 0.0f, 0.0f,-1.0f,		0.0f, 1.0f,		-1.0f, 0.0f, 0.0f,		0.0f, 1.0f, 0.0f,
	 1.0f,-1.0f,-1.0f,		 0.0f, 0.0f,-1.0f,		0.0f, 0.0f,		-1.0f, 0.0f, 0.0f,		0.0f, 1.0f, 0.0f,
	-1.0f,-1.0f,-1.0f,		 0.0f, 0.0f,-1.0f,		1.0f, 0.0f,		-1.0f, 0.0f, 0.0f,		0.0f, 1.0f, 0.0f,
	-1.0f, 1.0f,-1.0f,		 0.0f, 0.0f,-1.0f,		1.0f, 1.0f,		-1.0f, 0.0f, 0.0f,		0.0f, 1.0f, 0.0f,
// Left face
	-1.0f, 1.0f,-1.0f,		-1.0f, 0.0f, 0.0f,		0.0f, 1.0f,		0.0f, 0.0f, 1.0f,		0.0f, 1.0f, 0.0f,
	-1.0f,-1.0f,-1.0f,		-1.0f, 0.0f, 0.0f,		0.0f, 0.0f,		0.0f, 0.0f, 1.0f,		0.0f, 1.0f, 0.0f,
	-1.0f,-1.0f, 1.0f,		-1.0f, 0.0f, 0.0f,		1.0f, 0.0f,		0.0f, 0.0f, 1.0f,		0.0f, 1.0f, 0.0f,
	-1.0f, 1.0f, 1.0f,		-1.0f, 0.0f, 0.0f,		1.0f, 1.0f,		0.0f, 0.0f, 1.0f,		0.0f, 1.0f, 0.0f,
// Top face
	-1.0f, 1.0f,-1.0f,		 0.0f, 1.0f, 0.0f,		0.0f, 1.0f,		1.0f, 0.0f, 0.0f,		0.0f, 0.0f, -1.0f,
	-1.0f, 1.0f, 1.0f,		 0.0f, 1.0f, 0.0f,		0.0f, 0.0f,		1.0f, 0.0f, 0.0f,		0.0f, 0.0f, -1.0f,
	 1.0f, 1.0f, 1.0f,		 0.0f, 1.0f, 0.0f,		1.0f, 0.0f,		1.0f, 0.0f, 0.0f,		0.0f, 0.0f, -1.0f,
	 1.0f, 1.0f,-1.0f,		 0.0f, 1.0f, 0.0f,		1.0f, 1.0f,		1.0f, 0.0f, 0.0f,		0.0f, 0.0f, -1.0f,
// Bottom face
	-1.0f,-1.0f, 1.0f,		 0.0f,-1.0f, 0.0f,		0.0f, 1.0f,		1.0f, 0.0f, 0.0f,		0.0f, 0.0f, 1.0f,
	-1.0f,-1.0f,-1.0f,		 0.0f,-1.0f, 0.0f,		0.0f, 0.0f,		1.0f, 0.0f, 0.0f,		0.0f, 0.0f, 1.0f,
	 1.0f,-1.0f,-1.0f,		 0.0f,-1.0f, 0.0f,		1.0f, 0.0f,		1.0f, 0.0f, 0.0f,		0.0f, 0.0f, 1.0f,
	 1.0f,-1.0f, 1.0f,		 0.0f,-1.0f, 0.0f,		1.0f, 1.0f,		1.0f, 0.0f, 0.0f,		0.0f, 0.0f, 1.0f,
};

const int cube_indices_count = 36;
const unsigned int cube_indices[cube_indices_count] =
{
	0, 1, 2,		0, 2, 3,		// Front face
	4, 5, 6,		4, 6, 7,		// Right face
	8, 9, 10,		8, 10, 11,		// Back face
	12, 13, 14,		12, 14, 15,		// Left face
	16, 17, 18,		16, 18, 19,		// Top face
	20, 21, 22,		20, 22, 23,		// Bottom face
};

inline std::array<float,cube_vertices_count * 14> generate_custom_cube_vertices(
	const glm::vec3& left_top_front, 
    const glm::vec3& right_top_front,
    const glm::vec3& left_bottom_front, 
    const glm::vec3& right_bottom_front,
    const glm::vec3& left_top_back, 
    const glm::vec3& right_top_back,
    const glm::vec3& left_bottom_back, 
    const glm::vec3& right_bottom_back) {

	return {
//       Position															  Normal            Tex Coord          Tangent                     Bi
// Front face
    left_top_front.x,     left_top_front.y,     left_top_front.z,        0.0f, 0.0f, 1.0f,		0.0f, 1.0f,		1.0f, 0.0f, 0.0f,		0.0f, 1.0f, 0.0f,
    left_bottom_front.x,  left_bottom_front.y,  left_bottom_front.z,     0.0f, 0.0f, 1.0f,		0.0f, 0.0f,		1.0f, 0.0f, 0.0f,		0.0f, 1.0f, 0.0f,
    right_bottom_front.x, right_bottom_front.y, right_bottom_front.z,    0.0f, 0.0f, 1.0f,		1.0f, 0.0f,		1.0f, 0.0f, 0.0f,		0.0f, 1.0f, 0.0f,
    right_top_front.x,    right_top_front.y,    right_top_front.z,       0.0f, 0.0f, 1.0f,		1.0f, 1.0f,		1.0f, 0.0f, 0.0f,		0.0f, 1.0f, 0.0f,
// Right face
    right_top_front.x,    right_top_front.y,    right_top_front.z,       1.0f, 0.0f, 0.0f,		0.0f, 1.0f,		0.0f, 0.0f, -1.0f,		0.0f, 1.0f, 0.0f,
    right_bottom_front.x, right_bottom_front.y, right_bottom_front.z,    1.0f, 0.0f, 0.0f,		0.0f, 0.0f,		0.0f, 0.0f, -1.0f,		0.0f, 1.0f, 0.0f,
    right_bottom_back.x,  right_bottom_back.y,  right_bottom_back.z,     1.0f, 0.0f, 0.0f,		1.0f, 0.0f,		0.0f, 0.0f, -1.0f,		0.0f, 1.0f, 0.0f,
    right_top_back.x,     right_top_back.y,     right_top_back.z,        1.0f, 0.0f, 0.0f,		1.0f, 1.0f,		0.0f, 0.0f, -1.0f,		0.0f, 1.0f, 0.0f,
// Back face
	right_top_back.x,     right_top_back.y,     right_top_back.z,		 0.0f, 0.0f,-1.0f,		0.0f, 1.0f,		-1.0f, 0.0f, 0.0f,		0.0f, 1.0f, 0.0f,
	right_bottom_back.x,  right_bottom_back.y,  right_bottom_back.z,	 0.0f, 0.0f,-1.0f,		0.0f, 0.0f,		-1.0f, 0.0f, 0.0f,		0.0f, 1.0f, 0.0f,
	left_bottom_back.x,   left_bottom_back.y,   left_bottom_back.z,		 0.0f, 0.0f,-1.0f,		1.0f, 0.0f,		-1.0f, 0.0f, 0.0f,		0.0f, 1.0f, 0.0f,
	left_top_back.x,      left_top_back.y,      left_top_back.z,		 0.0f, 0.0f,-1.0f,		1.0f, 1.0f,		-1.0f, 0.0f, 0.0f,		0.0f, 1.0f, 0.0f,

	left_top_back.x,      left_top_back.y,      left_top_back.z,		-1.0f, 0.0f, 0.0f,		0.0f, 1.0f,		0.0f, 0.0f, 1.0f,		0.0f, 1.0f, 0.0f,
	left_bottom_back.x,   left_bottom_back.y,   left_bottom_back.z,		-1.0f, 0.0f, 0.0f,		0.0f, 0.0f,		0.0f, 0.0f, 1.0f,		0.0f, 1.0f, 0.0f,
	left_bottom_front.x,  left_bottom_front.y,  left_bottom_front.z,	-1.0f, 0.0f, 0.0f,		1.0f, 0.0f,		0.0f, 0.0f, 1.0f,		0.0f, 1.0f, 0.0f,
	left_top_front.x,     left_top_front.y,     left_top_front.z,		-1.0f, 0.0f, 0.0f,		1.0f, 1.0f,		0.0f, 0.0f, 1.0f,		0.0f, 1.0f, 0.0f,
// Top face
	left_top_back.x,      left_top_back.y,      left_top_back.z,		 0.0f, 1.0f, 0.0f,		0.0f, 1.0f,		1.0f, 0.0f, 0.0f,		0.0f, 0.0f, -1.0f,
	left_top_front.x,     left_top_front.y,     left_top_front.z,		 0.0f, 1.0f, 0.0f,		0.0f, 0.0f,		1.0f, 0.0f, 0.0f,		0.0f, 0.0f, -1.0f,
	right_top_front.x,    right_top_front.y,    right_top_front.z,		 0.0f, 1.0f, 0.0f,		1.0f, 0.0f,		1.0f, 0.0f, 0.0f,		0.0f, 0.0f, -1.0f,
	right_top_back.x,     right_top_back.y,     right_top_back.z,		 0.0f, 1.0f, 0.0f,		1.0f, 1.0f,		1.0f, 0.0f, 0.0f,		0.0f, 0.0f, -1.0f,
// Bottom face
	left_bottom_front.x,  left_bottom_front.y,  left_bottom_front.z,	 0.0f,-1.0f, 0.0f,		0.0f, 1.0f,		1.0f, 0.0f, 0.0f,		0.0f, 0.0f, 1.0f,
	left_bottom_back.x,   left_bottom_back.y,   left_bottom_back.z,		 0.0f,-1.0f, 0.0f,		0.0f, 0.0f,		1.0f, 0.0f, 0.0f,		0.0f, 0.0f, 1.0f,
	right_bottom_back.x,  right_bottom_back.y,  right_bottom_back.z,	 0.0f,-1.0f, 0.0f,		1.0f, 0.0f,		1.0f, 0.0f, 0.0f,		0.0f, 0.0f, 1.0f,
	right_bottom_front.x, right_bottom_front.y, right_bottom_front.z,	 0.0f,-1.0f, 0.0f,		1.0f, 1.0f,		1.0f, 0.0f, 0.0f,		0.0f, 0.0f, 1.0f,
};
}
