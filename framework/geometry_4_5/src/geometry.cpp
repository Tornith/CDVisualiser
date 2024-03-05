// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#include "geometry.hpp"
#include "glm/vec3.hpp"
#include <iostream>
#include <limits>
#include <tiny_obj_loader.h>
#include <glm/gtx/component_wise.hpp>

// ----------------------------------------------------------------------------
// Constructors & Destructors
// ----------------------------------------------------------------------------
Geometry::Geometry(GLenum mode, int elements_per_vertex, int vertices_count, const float* vertices, int indices_count,
                   const uint32_t* indices, GLint position_loc, GLint normal_loc, GLint tex_coord_loc, GLint tangent_loc,
                   GLint bitangent_loc)
    : Geometry_Base(mode, elements_per_vertex, vertices_count, indices_count, position_loc, normal_loc, tex_coord_loc,
                    tangent_loc, bitangent_loc) {

    // Creates a single buffer for vertex data.
    glCreateBuffers(1, &vertex_buffer);
    glNamedBufferStorage(vertex_buffer, vertex_buffer_size, vertices, GL_DYNAMIC_STORAGE_BIT);

    init_vao();

    if (indices && indices_count > 0) {
        // Creates a buffer for indices.
        glCreateBuffers(1, &index_buffer);
        glNamedBufferStorage(index_buffer, indices_count * sizeof(uint32_t), indices, GL_DYNAMIC_STORAGE_BIT);
        glVertexArrayElementBuffer(vao, index_buffer);
    }
}

Geometry::Geometry(GLenum mode, int elements_per_vertex, std::vector<float> interleaved_vertices, std::vector<uint32_t> indices,
                   GLint position_loc, GLint normal_loc, GLint tex_coord_loc, GLint tangent_loc, GLint bitangent_loc)
    : Geometry(mode, elements_per_vertex, static_cast<int>(interleaved_vertices.size()), interleaved_vertices.data(),
               static_cast<int>(indices.size()), indices.data(), position_loc, normal_loc, tex_coord_loc, tangent_loc,
               bitangent_loc) {}

Geometry::Geometry(
    GLenum mode,
    std::vector<float> positions,
    std::vector<uint32_t> indices,
    std::vector<float> normals,
    std::vector<float> colors,
    std::vector<float> tex_coords,
    std::vector<float> tangents,
    std::vector<float> bitangents,
    GLint position_loc,
    GLint normal_loc,
    GLint tex_coord_loc,
    GLint tangent_loc,
    GLint bitangent_loc)
    : Geometry_Base(mode, positions, indices, normals, colors, tex_coords, tangents, bitangents, position_loc, normal_loc,
                    tex_coord_loc, tangent_loc, bitangent_loc) {

    // Creates a single buffer for vertex data.
    glCreateBuffers(1, &vertex_buffer);
    glNamedBufferStorage(vertex_buffer, vertex_buffer_size, interleaved_vertices.data(), GL_DYNAMIC_STORAGE_BIT);

    init_vao();

    if (!indices.empty()) {
        // Creates a buffer for indices.
        glCreateBuffers(1, &index_buffer);
        glNamedBufferStorage(index_buffer, indices.size() * sizeof(uint32_t), indices.data(), GL_DYNAMIC_STORAGE_BIT);
        glVertexArrayElementBuffer(vao, index_buffer);
    }
}

Geometry::Geometry(const Geometry& other)
    : Geometry_Base(other) {
    // Creates a single buffer for vertex data.
    glCreateBuffers(1, &vertex_buffer);
    glNamedBufferStorage(vertex_buffer, vertex_buffer_size, nullptr, GL_DYNAMIC_STORAGE_BIT);
    glCopyNamedBufferSubData(other.vertex_buffer, vertex_buffer, 0, 0, vertex_buffer_size);

    // Creates a buffer for indices.
    if (draw_elements_count > 0) {
        glCreateBuffers(1, &index_buffer);
        glNamedBufferStorage(index_buffer, draw_elements_count * sizeof(unsigned int), nullptr, GL_DYNAMIC_STORAGE_BIT);
        glCopyNamedBufferSubData(other.index_buffer, index_buffer, 0, 0, draw_elements_count * sizeof(unsigned int));
    }

    init_vao();

    // Sets the mode and the number of vertices to draw.
    if (mode != GL_PATCHES) {
        glVertexArrayElementBuffer(vao, index_buffer);
    }
}

// ----------------------------------------------------------------------------
// Methods
// ----------------------------------------------------------------------------
void Geometry::init_vao() {
    // Creates a new VAO.
    glCreateVertexArrays(1, &vao);

    // Binds the interleaved buffer to the VAO.
    glVertexArrayVertexBuffer(vao, 0, vertex_buffer, 0, vertex_buffer_stride);

    // Sets the vertex attributes and their parameters.
    int offset = 0;
    if (position_loc >= 0) {
        glEnableVertexArrayAttrib(vao, position_loc);
        glVertexArrayAttribFormat(vao, position_loc, 3, GL_FLOAT, GL_FALSE, 0);
        glVertexArrayAttribBinding(vao, position_loc, 0);
        offset += 3;
    }
    if (normal_loc >= 0) {
        glEnableVertexArrayAttrib(vao, normal_loc);
        glVertexArrayAttribFormat(vao, normal_loc, 3, GL_FLOAT, GL_FALSE, sizeof(float) * offset);
        glVertexArrayAttribBinding(vao, normal_loc, 0);
        normal_offset = offset;
        offset += 3;
    }
    if (color_loc >= 0) {
        glEnableVertexArrayAttrib(vao, color_loc);
        glVertexArrayAttribFormat(vao, color_loc, 3, GL_FLOAT, GL_FALSE, sizeof(float) * offset);
        glVertexArrayAttribBinding(vao, color_loc, 0);
        // todo solve offset for color
    }
    if (tex_coord_loc >= 0) {
        glEnableVertexArrayAttrib(vao, tex_coord_loc);
        glVertexArrayAttribFormat(vao, tex_coord_loc, 2, GL_FLOAT, GL_FALSE, sizeof(float) * offset);
        glVertexArrayAttribBinding(vao, tex_coord_loc, 0);
        tex_coord_offset = offset;
        offset += 2;
    }
    if (tangent_loc >= 0) {
        glEnableVertexArrayAttrib(vao, tangent_loc);
        glVertexArrayAttribFormat(vao, tangent_loc, 3, GL_FLOAT, GL_FALSE, sizeof(float) * offset);
        glVertexArrayAttribBinding(vao, tangent_loc, 0);
        offset += 3;
    }
    if (bitangent_loc >= 0) {
        glEnableVertexArrayAttrib(vao, bitangent_loc);
        glVertexArrayAttribFormat(vao, bitangent_loc, 3, GL_FLOAT, GL_FALSE, sizeof(float) * offset);
        glVertexArrayAttribBinding(vao, bitangent_loc, 0);
        offset += 3;
    }
}