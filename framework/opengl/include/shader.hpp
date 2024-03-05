// ###############################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#pragma once

#include "glad/glad.h"
#include <filesystem>
#include <string>

/**
 * The basic representation of a shader.
 *
 * @author	<a href="mailto:jan.byska@gmail.com">Jan Byška</a>
 * @author	<a href="mailto:cejka.honza@gmail.com ">Jan Čejka</a>
 */
struct Shader {

    // ----------------------------------------------------------------------------
    // Variables
    // ----------------------------------------------------------------------------
public:
    /** The OpenGL object corresponding to this shader. */
    GLuint shader = 0;

    /** The type of the shader, e.g. GL_VERTEX_SHADER. */
    GLenum shader_type = 0;

    /** The path to the source code file. */
    std::filesystem::path file_path = {};

    // ----------------------------------------------------------------------------
    // Constructors
    // ----------------------------------------------------------------------------
    Shader() { }

    /**
     * Creates a shader of given type, loads and sets its source code, compiles it, and prints errors to
     * stdout if some occur. Note that the @link shader attribute is set to 0 if the compilation fails.
     *
     * @param 	shader_type	Type of the shader.
     * @param 	file_path  	File path to the file.
     */
    Shader(GLenum shader_type, const std::filesystem::path& file_path);

    Shader(const Shader& other);

    // ----------------------------------------------------------------------------
    // Operators
    // ----------------------------------------------------------------------------
    /**
    * The copy assignment using copy-and-swap idiom.
    *
    * @param other The other shader that will be copied (i.e., swapped into this).
    * 			   
    * @return A shallow copy of the other shader that was moved into this object.
    */
    Shader& operator=(Shader other);

    Shader(Shader&& other);

    friend void swap_fields(Shader& first, Shader& second) noexcept {
        using std::swap;

        swap(first.shader, second.shader);
        swap(first.shader_type, second.shader_type);
        swap(first.file_path, second.file_path);
    }

    virtual ~Shader();
};
