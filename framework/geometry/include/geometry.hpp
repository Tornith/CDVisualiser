// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#pragma once

#include "geometry_base.hpp"
#include "glad/glad.h"
#include <filesystem>
#include <vector>

/**
 * The implementation of the Geometry_Base class providing the
 *
 * @author	<a href="mailto:jan.byska@gmail.com">Jan Byška</a>
 * @author	<a href="mailto:matus.talcik@gmail.com">Matúš Talčík</a>
 */
class Geometry : public Geometry_Base {

    // ----------------------------------------------------------------------------
    // Constructors
    // ----------------------------------------------------------------------------
public:
    /** Creates a new empty @link Geometry object. */
    Geometry()
        : Geometry_Base(GL_POINTS, 0, 0, 0) { }

    /**
     * Creates a @link Geometry object. TODO add colors
     * @param   mode                The mode that will be used for rendering the geometry.
     * @param   vertex_buffer_size  The number of float values stored in the buffer for each vertex.
     * @param 	vertices_count	    The number of vertices that the created geometry will have.
     * @param 	vertices	    The actual geometry vertices.
     * @param 	indices_count 	    The number of indices wihing the geometry.
     * @param 	indices		    The actual indices.
     * @param 	position_loc  	    The location of position vertex attribute for the VAO (use -1 if not necessary).
     * @param 	normal_loc	    The location of normal vertex attribute for the VAO (use -1 if not necessary).
     * @param 	tex_coord_loc 	    The location of texture coordinates vertex attribute for the VAO (use -1 if not necessary).
     * @param 	tangent_loc   	    The location of tangent vertex attribute for the VAO (use -1 if not necessary).
     * @param 	bitangent_loc 	    The location of bitangent vertex attribute for the VAO (use -1 if not necessary).
     */
    Geometry(GLenum mode, int vertex_buffer_size, int vertices_count, const float* vertices, int indices_count,
             const uint32_t* indices, GLint position_loc = DEFAULT_POSITION_LOC, GLint normal_loc = DEFAULT_NORMAL_LOC,
             GLint tex_coord_loc = DEFAULT_TEX_COORD_LOC, GLint tangent_loc = DEFAULT_TANGENT_LOC,
             GLint bitangent_loc = DEFAULT_BITANGENT_LOC);

    /**
     * Creates a @link Geometry object. TODO add colors
     *
     * @param 	mode					The mode that will be used for rendering the geometry.
     * @param 	elements_per_vertex 	The elements per vertex.
     * @param 	interleaved_vertices	The interleaved buffer with vertex data.
     * @param 	indices					The actual indices.
     * @param 	position_loc			The location of position vertex attribute for the VAO (use -1 if not necessary).
     * @param 	normal_loc				The location of normal vertex attribute for the VAO (use -1 if not necessary).
     * @param 	tex_coord_loc			The location of texture coordinates vertex attribute for the VAO (use -1 if not	necessary).
     * @param 	tangent_loc				The location of tangent vertex attribute for the VAO (use -1 if not necessary).
     * @param 	bitangent_loc			The location of bitangent vertex attribute for the VAO (use -1 if not necessary).
     */
    Geometry(GLenum mode, int elements_per_vertex, std::vector<float> interleaved_vertices, std::vector<uint32_t> indices = {},
             GLint position_loc = DEFAULT_POSITION_LOC, GLint normal_loc = DEFAULT_NORMAL_LOC,
             GLint tex_coord_loc = DEFAULT_TEX_COORD_LOC, GLint tangent_loc = DEFAULT_TANGENT_LOC,
             GLint bitangent_loc = DEFAULT_BITANGENT_LOC);

    /**
     * Creates a @link Geometry object. TODO add colors
     *
     * @param 	mode		 	The mode that will be used for rendering the geometry.
     * @param 	positions	 	The the list vertex positions.
     * @param 	normals		 	The list of normals for each vertex.
     * @param 	colors		 	The list of colors for each vertex.
     * @param 	tex_coords   	The list of texture coordinates for each vertex.
     * @param 	tangents	 	The list of tangents for each vertex.
     * @param 	bitangents   	The list of bitangents for each vertex.
     * @param 	indices		 	The list of indices.
     * @param 	position_loc 	The location of position vertex attribute for the VAO (use -1 if not necessary).
     * @param 	normal_loc   	The location of normal vertex attribute for the VAO (use -1 if not necessary).
     * @param 	tex_coord_loc	The location of texture coordinates vertex attribute for the VAO (use -1 if not
     * 							necessary).
     * @param 	tangent_loc  	The location of tangent vertex attribute for the VAO (use -1 if not necessary).
     * @param 	bitangent_loc	The location of bitangent vertex attribute for the VAO (use -1 if not necessary).
     */
    Geometry(
        GLenum mode,
        std::vector<float> positions,
        std::vector<uint32_t> indices,
        std::vector<float> normals = {},
        std::vector<float> colors = {},
        std::vector<float> tex_coords = {},
        std::vector<float> tangents = {},
        std::vector<float> bitangents = {},
        GLint position_loc = DEFAULT_POSITION_LOC,
        GLint normal_loc = DEFAULT_NORMAL_LOC,
        GLint tex_coord_loc = DEFAULT_TEX_COORD_LOC,
        GLint tangent_loc = DEFAULT_TANGENT_LOC,
        GLint bitangent_loc = DEFAULT_BITANGENT_LOC);

    /**
     * Creates a new @link Geometry object from another geometry performing a deep copy.
     *
     * @param 	other	The other geometry.
     */
    Geometry(const Geometry& other);

    /**
     * Move constructor swapping the two geometries.
     *
     * @param 	other	The other geometry.
     */
    Geometry(Geometry&& other)
        : Geometry() { swap_fields(*this, other); };

    /**
     * Destroys this @link Geometry.
     */
    virtual ~Geometry();

    /**
     * Loads a geometry from a file.
     *
     * @param 	file_path	   	The file name.
     * @param 	center_vertices	A flag determining if the loaded object should be centered.
     *
     * @return	A loaded geometry.
     */
    static Geometry from_file(std::filesystem::path file_path, bool center_vertices = true);

    // ----------------------------------------------------------------------------
    // Operators
    // ----------------------------------------------------------------------------
public:
    /**
     * The copy assignment using copy-and-swap idiom.
     *
     * @param other The other geometry that will be copied (i.e., swapped into this).
     * @return A shallow copy of the other geometry that was moved into this object.
     */
    Geometry& operator=(Geometry other) {
        swap_fields(*this, other);
        return *this;
    };

    // ----------------------------------------------------------------------------
    // Methods
    // ----------------------------------------------------------------------------
private:
    /** Initialize Vertex Array Object for the geometry. */
    void init_vao();
};
