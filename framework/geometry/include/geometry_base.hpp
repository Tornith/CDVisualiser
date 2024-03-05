// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#pragma once

#include "glad/glad.h"
#include <vector>

/**
 * This is a base class for all geometry classes that wraps buffers and vertex array objects for geometries.
 * <p>
 * The main reason for separating the functionality withing this class is the ability to easily reuse it for both OpenGL 3.3 and
 * OpenGL 4.5+ implementations of the geometries for different courses.
 * <p>
 * The creation of the OpenGL objects is left to the
 *
 * To draw the geometry, bind this geometry's vao (using glBindVertexArray({@link Geometry::vao}) or
 * {@link Geometry::bind_vao), and call drawing commands using @link Geometry_Base::draw or @link Geometry_Base::draw_instanced.
 *
 * Example:
 * <code>
 *  Cube my_cube(...);
 *  ...
 *  my_cube.bind_vao();
 *  my_cube.draw();
 * </code>
 *
 * @author	<a href="mailto:cejka.honza@gmail.com ">Jan Čejka</a>
 * @author	<a href="mailto:jan.byska@gmail.com">Jan Byška</a>
 * @author	<a href="mailto:matus.talcik@gmail.com">Matúš Talčík</a>
 */
class Geometry_Base {

    // ----------------------------------------------------------------------------
    // Static Variables
    // ----------------------------------------------------------------------------
public:
    // The following constants define the default indices of input vertex shader variables.
    // Make sure these numbers corresponds to layout(binding=N) in shaders (or use glBindAttribLocation).
    /** The default location of position vertex attribute. */
    static const int DEFAULT_POSITION_LOC = 0;
    /** The default location of normal vertex attribute. */
    static const int DEFAULT_NORMAL_LOC = 1;
    /** The default location of texture coordinate vertex attribute. */
    static const int DEFAULT_TEX_COORD_LOC = 2;
    /** The default location of tangent vertex attribute. */
    static const int DEFAULT_TANGENT_LOC = 3;
    /** The default location of bitangent vertex attribute. */
    static const int DEFAULT_BITANGENT_LOC = 4;
    /** The default location of color vertex attribute. */
    static const int DEFAULT_COLOR_LOC = 5; // TODO make number 2

    // ----------------------------------------------------------------------------
    // Variables
    // ----------------------------------------------------------------------------
public:
    /** Type of the primitives to be drawn using this geometry, e.g. GL_TRIANGLES. */
    GLenum mode = GL_POINTS;

    /** The sizes of the buffer storing the geometry (positions, normals, texture coordinates, etc.). */
    GLsizei vertex_buffer_size = 0;

    /** The vertex buffer stride, i.e., the spacing of the elements in the array.*/
    GLsizei vertex_buffer_stride = 0;

    /** The buffer storing the interleaved data for the vertices. */
    std::vector<float> interleaved_vertices{};

    /** The offset for positions in the interleaved buffer. */
    int position_offset = 0;

    /** The offset for colors in the interleaved buffer. */
    int color_offset = -1;

    /** The offset for normals in the interleaved buffer. */
    int normal_offset = -1;

    /** The offset for texture coordinates in the interleaved buffer. */
    int tex_coord_offset = -1;

    /** The number of elements (floats) per vertex. */
    int elements_per_vertex = 0;

    /** The number of vertices to be drawn using glDrawArrays. */
    GLsizei draw_arrays_count = 0;

    /** The number of vertices to be drawn using glDrawElements. */
    GLsizei draw_elements_count = 0;

    /** The number of patch vertices. This variable is used only when mode is set to GL_PATCHES, otherwise it is ignored. */
    GLsizei patch_vertices = 0;

    /** The Vertex Array Object that describes how the vertex attributes are stored. */
    GLuint vao = 0;

    /** The OpenGL buffer with the the geometry data (positions, normals, texture coordinates, etc.). */
    GLuint vertex_buffer = 0;

    /** The OpenGL buffer with the indices describing the geometry. */
    GLuint index_buffer = 0;

    /** The location of position vertex attribute. */
    GLint position_loc = DEFAULT_POSITION_LOC;

    /** The location of normal vertex attribute. */
    GLint normal_loc = DEFAULT_NORMAL_LOC;

    /** The location of texture coordinate vertex attribute. */
    GLint tex_coord_loc = DEFAULT_TEX_COORD_LOC;

    /** The location of tangent vertex attribute. */
    GLint tangent_loc = DEFAULT_TANGENT_LOC;

    /** The location of bitangent vertex attribute. */
    GLint bitangent_loc = DEFAULT_BITANGENT_LOC;

    /** The location of bitangent vertex attribute. */
    GLint color_loc = DEFAULT_COLOR_LOC;

    // ----------------------------------------------------------------------------
    // Constructors
    // ----------------------------------------------------------------------------
public:
    Geometry_Base() {}

    /**
     * Creates a new @link Geometry_Base object. Note that this constructor does not initialize the OpenGL objects, this
     * is left to the child classes.
     *
     * @param 	mode			   	The mode that will be used for rendering the geometry.
     * @param 	elements_per_vertex	The number of elements (floats) per vertex.
     * @param 	vertices_count	   	The number of vertices in the geometry.
     * @param 	indices_count	   	The number of indices defining the geometry.
     * @param 	position_loc	   	The location of position vertex attribute for the VAO (use -1 if not necessary).
     * @param 	normal_loc		   	The location of normal vertex attribute for the VAO (use -1 if not necessary).
     * @param 	tex_coord_loc	   	The location of texture coordinates vertex attribute for the VAO (use -1 if not	necessary).
     * @param 	tangent_loc		   	The location of tangent vertex attribute for the VAO (use -1 if not necessary).
     * @param 	bitangent_loc	   	The location of bitangent vertex attribute for the VAO (use -1 if not necessary).
     * @param 	color_loc		   	The location of color vertex attribute for the VAO (use -1 if not necessary).
     */
    Geometry_Base(GLenum mode, int elements_per_vertex, int vertices_count, int indices_count, GLint position_loc = DEFAULT_POSITION_LOC, GLint normal_loc = DEFAULT_NORMAL_LOC,
                  GLint tex_coord_loc = DEFAULT_TEX_COORD_LOC, GLint tangent_loc = DEFAULT_TANGENT_LOC, GLint bitangent_loc = DEFAULT_BITANGENT_LOC, GLint color_loc = DEFAULT_COLOR_LOC);
    ;

    /**
     * Creates a new @link Geometry_Base object. Note that this constructor does not initialize the OpenGL objects, this
     * is left to the child classes.
     *
     * @param 	mode		 	The mode that will be used for rendering the geometry.
     * @param 	positions	 	The the list vertex positions (if empty position_loc will be set to -1).
     * @param 	normals		 	The list of normals for each vertex (if empty @link normal_loc will be set to -1).
     * @param 	colors		 	The list of colors for each vertex (if empty @link color_loc will be set to -1).
     * @param 	tex_coords   	The list of texture coordinates for each vertex (if empty @link tex_coord_loc will be set to -1).
     * @param 	tangents	 	The list of tangents for each vertex (if empty @link tangent_loc will be set to -1).
     * @param 	bitangents   	The list of bitangents for each vertex (if empty @link bitangent_loc will be set to -1).
     * @param 	indices		 	The list of indices.
     * @param 	position_loc 	The location of position vertex attribute for the VAO (use -1 if not necessary).
     * @param 	normal_loc   	The location of normal vertex attribute for the VAO (use -1 if not necessary).
     * @param 	tex_coord_loc	The location of texture coordinates vertex attribute for the VAO (use -1 if not necessary).
     * @param 	tangent_loc  	The location of tangent vertex attribute for the VAO (use -1 if not necessary).
     * @param 	bitangent_loc	The location of bitangent vertex attribute for the VAO (use -1 if not necessary).
     * @param 	color_loc	 	The location of color vertex attribute for the VAO (use -1 if not necessary).
     */
    Geometry_Base(GLenum mode, std::vector<float> positions, std::vector<uint32_t> indices, std::vector<float> normals, std::vector<float> colors, std::vector<float> tex_coords,
                  std::vector<float> tangents, std::vector<float> bitangents, GLint position_loc = DEFAULT_POSITION_LOC, GLint normal_loc = DEFAULT_NORMAL_LOC,
                  GLint tex_coord_loc = DEFAULT_TEX_COORD_LOC, GLint tangent_loc = DEFAULT_TANGENT_LOC, GLint bitangent_loc = DEFAULT_BITANGENT_LOC, GLint color_loc = DEFAULT_COLOR_LOC);
    ;

    /**
     * Creates a new @link Geometry_Base object from another geometry performing a deep copy.
     *
     * @param 	other	The other geometry.
     */
    Geometry_Base(const Geometry_Base& other)
        : mode(other.mode), vertex_buffer_size(other.vertex_buffer_size), vertex_buffer_stride(other.vertex_buffer_stride), interleaved_vertices(other.interleaved_vertices),
          position_offset(other.position_offset), color_offset(other.color_offset), normal_offset(other.normal_offset), tex_coord_offset(other.tex_coord_offset),
          elements_per_vertex(other.elements_per_vertex), draw_arrays_count(other.draw_arrays_count),
          draw_elements_count(other.draw_elements_count), // vao(other.vao), vertex_buffer(other.vertex_buffer), index_buffer(other.index_buffer), THESE SHOULD NOT BE COPIED BUT NEEDS TO BE RECREATED
          patch_vertices(other.patch_vertices), position_loc(other.position_loc), normal_loc(other.normal_loc), tex_coord_loc(other.tex_coord_loc), tangent_loc(other.tangent_loc),
          bitangent_loc(other.bitangent_loc), color_loc(other.color_loc) {};

    /**
     * The move constructor swapping the two geometries.
     *
     * @param 	other	The other geometry.
     */
    Geometry_Base(Geometry_Base&& other) noexcept;

    /** The virtual destructor. */
    virtual ~Geometry_Base() {}

    // ----------------------------------------------------------------------------
    // Operators
    // ----------------------------------------------------------------------------
    /**
     * The copy assignment using copy-and-swap idiom.
     *
     * @param other The other geometry program that will be copied (i.e., swapped into this).
     *
     * @return A shallow copy of the other geometry that was moved into this object.
     */
    Geometry_Base& operator=(Geometry_Base other) {
        swap_fields(*this, other);
        return *this;
    };

    // ----------------------------------------------------------------------------
    // Methods
    // ----------------------------------------------------------------------------
public:
    /**
     * The custom swap method that exchanges the values of fields of two geometries.
     *
     * @param 	first 	The first geometry to swap.
     * @param 	second	The second geometry to swap.
     */
    void swap_fields(Geometry_Base& first, Geometry_Base& second) const noexcept;

    /** Sets the default number of patch vertices if the current mode is GL_PATCHES. */
    void init_patches_count();

    /** Binds the VAO corresponding to this geometry. */
    void bind_vao() const;

    /**
     * Draws the geometry using either glDrawArrays or glDrawElements based on the current values of {@link draw_arrays_count} and
     * {@link draw_elements_count}.
     */
    void draw() const;

    /**
     * Draws multiple instances of the geometry using either glDrawArraysInstanced or glDrawElementsInstanced based on
     * the current values of {@link draw_arrays_count} and {@link draw_elements_count}.
     *
     * @param 	count	The number of instances to render.
     */
    void draw_instanced(int count) const;
};
