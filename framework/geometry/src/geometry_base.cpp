// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#include "geometry_base.hpp"
#include "geometry.hpp"
#include "tiny_obj_loader.h"
#include "glm/vec3.hpp"
#include <iostream>

// ----------------------------------------------------------------------------
// Constructors & Destructors
// ----------------------------------------------------------------------------

Geometry_Base::Geometry_Base(GLenum mode, int elements_per_vertex, int vertices_count, int indices_count, GLint position_loc, GLint normal_loc, GLint tex_coord_loc, GLint tangent_loc,
                             GLint bitangent_loc, GLint color_loc)
    : mode(mode), elements_per_vertex(elements_per_vertex), position_loc(position_loc), normal_loc(normal_loc),
      tex_coord_loc(tex_coord_loc), tangent_loc(tangent_loc), bitangent_loc(bitangent_loc), color_loc(color_loc) {

    vertex_buffer_stride = elements_per_vertex * sizeof(float);
    vertex_buffer_size = vertices_count * vertex_buffer_stride;

    init_patches_count();
    draw_elements_count = indices_count;
    draw_arrays_count = vertices_count;
}

Geometry_Base::Geometry_Base(GLenum mode, std::vector<float> positions, std::vector<uint32_t> indices, std::vector<float> normals, std::vector<float> colors, std::vector<float> tex_coords,
                             std::vector<float> tangents, std::vector<float> bitangents, GLint position_loc, GLint normal_loc, GLint tex_coord_loc, GLint tangent_loc, GLint bitangent_loc,
                             GLint color_loc)
    : mode(mode), position_loc(!positions.empty() ? position_loc : -1), normal_loc(!normals.empty() ? normal_loc : -1),
      tex_coord_loc(!tex_coords.empty() ? tex_coord_loc : -1), tangent_loc(!tangents.empty() ? tangent_loc : -1),
      bitangent_loc(!bitangents.empty() ? bitangent_loc : -1), color_loc(!colors.empty() ? color_loc : -1) {

    // Computes the number of elements per vertex.
    elements_per_vertex = 3 + (!normals.empty() ? 3 : 0) + (!colors.empty() ? 3 : 0) + (!tex_coords.empty() ? 2 : 0) +
                          (!tangents.empty() ? 3 : 0) + (!bitangents.empty() ? 3 : 0);

    // Computes the number of vertices.
    const int vertices_count = static_cast<int>(positions.size() / 3);

    // Builds the interleaved buffer from the input data.
    for (size_t i = 0; i < positions.size() / 3; i += 1) {
        interleaved_vertices.push_back(positions[i * 3 + 0]);
        interleaved_vertices.push_back(positions[i * 3 + 1]);
        interleaved_vertices.push_back(positions[i * 3 + 2]);

        if (!normals.empty()) {
            interleaved_vertices.push_back(normals[i * 3 + 0]);
            interleaved_vertices.push_back(normals[i * 3 + 1]);
            interleaved_vertices.push_back(normals[i * 3 + 2]);
        }

        if (!colors.empty()) {
            interleaved_vertices.push_back(colors[i * 3 + 0]);
            interleaved_vertices.push_back(colors[i * 3 + 1]);
            interleaved_vertices.push_back(colors[i * 3 + 2]);
        }

        if (!tex_coords.empty()) {
            interleaved_vertices.push_back(tex_coords[i * 2 + 0]);
            interleaved_vertices.push_back(tex_coords[i * 2 + 1]);
        }

        if (!tangents.empty()) {
            interleaved_vertices.push_back(tangents[i * 3 + 0]);
            interleaved_vertices.push_back(tangents[i * 3 + 1]);
            interleaved_vertices.push_back(tangents[i * 3 + 2]);
        }

        if (!bitangents.empty()) {
            interleaved_vertices.push_back(bitangents[i * 3 + 0]);
            interleaved_vertices.push_back(bitangents[i * 3 + 1]);
            interleaved_vertices.push_back(bitangents[i * 3 + 2]);
        }
    }

    vertex_buffer_stride = elements_per_vertex * sizeof(float);
    vertex_buffer_size = vertices_count * vertex_buffer_stride;

    init_patches_count();

    // Sets the number of elements according to the
    if (!indices.empty()) {
        draw_elements_count = static_cast<GLsizei>(indices.size());
    }

    draw_arrays_count = vertices_count;
}

Geometry_Base::Geometry_Base(Geometry_Base&& other) noexcept: Geometry_Base() { swap_fields(*this, other); }

Geometry::~Geometry() {
    glDeleteBuffers(1, &vertex_buffer);
    glDeleteBuffers(1, &index_buffer);
    glDeleteVertexArrays(1, &vao);
}

// ----------------------------------------------------------------------------
// Methods
// ----------------------------------------------------------------------------
void Geometry_Base::swap_fields(Geometry_Base& first, Geometry_Base& second) const noexcept {
    using std::swap;

    swap(first.vertex_buffer, second.vertex_buffer);
    swap(first.vertex_buffer_size, second.vertex_buffer_size);
    swap(first.index_buffer, second.index_buffer);
    swap(first.vao, second.vao);
    swap(first.mode, second.mode);
    swap(first.draw_arrays_count, second.draw_arrays_count);
    swap(first.draw_elements_count, second.draw_elements_count);
    swap(first.patch_vertices, second.patch_vertices);
    swap(first.position_loc, second.position_loc);
    swap(first.normal_loc, second.normal_loc);
    swap(first.tex_coord_loc, second.tex_coord_loc);
    swap(first.tangent_loc, second.tangent_loc);
    swap(first.bitangent_loc, second.bitangent_loc);
    swap(first.color_loc, second.color_loc);
    swap(first.elements_per_vertex, second.elements_per_vertex);
    swap(first.vertex_buffer_stride, second.vertex_buffer_stride);
    swap(first.interleaved_vertices, second.interleaved_vertices);
    swap(first.position_offset, second.position_offset);
    swap(first.color_offset, second.color_offset);
    swap(first.normal_offset, second.normal_offset);
    swap(first.tex_coord_offset, second.tex_coord_offset);
}

void Geometry_Base::init_patches_count() {
    if (mode == GL_PATCHES) {
        patch_vertices = 16;
    }
}

void Geometry_Base::bind_vao() const {
    glBindVertexArray(vao);
}

void Geometry_Base::draw() const {
    bind_vao();

    if (mode == GL_PATCHES) {
        glPatchParameteri(GL_PATCH_VERTICES, patch_vertices);
    }

    if (draw_elements_count > 0) {
        glDrawElements(mode, draw_elements_count, GL_UNSIGNED_INT, nullptr);
    } else {
        glDrawArrays(mode, 0, draw_arrays_count);
    }
}

void Geometry_Base::draw_instanced(int count) const {
    bind_vao();

    if (mode == GL_PATCHES) {
        glPatchParameteri(GL_PATCH_VERTICES, patch_vertices);
    }

    if (draw_elements_count > 0) {
        glDrawElementsInstanced(mode, draw_elements_count, GL_UNSIGNED_INT, nullptr, count);
    } else {
        glDrawArraysInstanced(mode, 0, draw_arrays_count, count);
    }
}

Geometry Geometry::from_file(std::filesystem::path path, bool center_vertices) {
    const std::string extension = path.extension().generic_string();

    if (extension == ".obj") {
        tinyobj::ObjReader reader;

        if (!reader.ParseFromFile(path.generic_string())) {
            if (!reader.Error().empty()) {
                std::cerr << "TinyObjReader: " << reader.Error();
            }
        }

        if (!reader.Warning().empty()) {
            std::cout << "TinyObjReader: " << reader.Warning();
        }

        auto& attrib = reader.GetAttrib();
        auto& shapes = reader.GetShapes();
        auto& materials = reader.GetMaterials();

        // Take only the first shape found
        const tinyobj::shape_t& shape = shapes[0];

        std::vector<float> positions;
        std::vector<float> normals;
        std::vector<float> tex_coords;

        glm::vec3 min{std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
        glm::vec3 max{std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), std::numeric_limits<float>::min()};

        // Loop over faces(polygon)
        size_t index_offset = 0;
        for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); f++) {
            // Loop over vertices in the face.
            for (size_t v = 0; v < 3; v++) {
                // Access to vertex
                tinyobj::index_t idx = shape.mesh.indices[index_offset + v];

                tinyobj::real_t vx = attrib.vertices[3 * idx.vertex_index + 0];
                tinyobj::real_t vy = attrib.vertices[3 * idx.vertex_index + 1];
                tinyobj::real_t vz = attrib.vertices[3 * idx.vertex_index + 2];

                tinyobj::real_t nx;
                tinyobj::real_t ny;
                tinyobj::real_t nz;
                if (!attrib.normals.empty()) {
                    nx = attrib.normals[3 * idx.normal_index + 0];
                    ny = attrib.normals[3 * idx.normal_index + 1];
                    nz = attrib.normals[3 * idx.normal_index + 2];
                } else {
                    nx = 0.0;
                    ny = 0.0;
                    nz = 0.0;
                }

                tinyobj::real_t tx;
                tinyobj::real_t ty;
                if (!attrib.texcoords.empty()) {
                    tx = attrib.texcoords[2 * idx.texcoord_index + 0];
                    ty = attrib.texcoords[2 * idx.texcoord_index + 1];
                } else {
                    tx = 0.0;
                    ty = 0.0;
                }

                if (center_vertices) {
                    min.x = std::min(min.x, vx);
                    min.y = std::min(min.y, vy);
                    min.z = std::min(min.z, vz);
                    max.x = std::max(max.x, vx);
                    max.y = std::max(max.y, vy);
                    max.z = std::max(max.z, vz);
                }

                positions.insert(positions.end(), {vx, vy, vz});
                normals.insert(normals.end(), {nx, ny, nz});
                tex_coords.insert(tex_coords.end(), {tx, ty});
            }
            index_offset += 3;
        }

        if (center_vertices) {
            glm::vec3 diff = max - min;
            glm::vec3 center = min + 0.5f * diff;
            min -= center;
            max -= center;
            for (int i = 0; i < positions.size(); i += 3) {
                positions[i + 0] -= center.x;
                positions[i + 1] -= center.y;
                positions[i + 2] -= center.z;

                positions[i + 0] /= std::max(std::max(diff.x, diff.y), diff.z);
                positions[i + 1] /= std::max(std::max(diff.x, diff.y), diff.z);
                positions[i + 2] /= std::max(std::max(diff.x, diff.y), diff.z);
            }
        }

        const int elements_per_vertex = 3 + (!normals.empty() ? 3 : 0) + (!tex_coords.empty() ? 2 : 0);

        return Geometry{GL_TRIANGLES, positions, {/*indices*/}, normals, {/*colors*/}, tex_coords};
    }
    std::cerr << "Extension " << extension << " not supported" << std::endl;

    return Geometry{};
}
