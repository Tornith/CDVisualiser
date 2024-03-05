// ###############################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#include "program.hpp"

#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

// ----------------------------------------------------------------------------
// Constructors
// ----------------------------------------------------------------------------
ShaderProgram::ShaderProgram()
    : program(0), valid(false) { program = glCreateProgram(); }

ShaderProgram::ShaderProgram(const std::filesystem::path& vertex_shader, const std::filesystem::path& fragment_shader)
    : ShaderProgram() {
    add_vertex_shader(vertex_shader);
    add_fragment_shader(fragment_shader);

    link();
}

ShaderProgram::~ShaderProgram() { delete_program(); }

// ----------------------------------------------------------------------------
// Methods
// ----------------------------------------------------------------------------
bool ShaderProgram::link() {
    if (!program) {
        std::cerr << "The OpenGL program object is invalid (link)." << std::endl;
        return false;
    }

    // link program
    glLinkProgram(program);

    // link and get errors
    int link_status;
    glGetProgramiv(program, GL_LINK_STATUS, &link_status);
    if (GL_FALSE == link_status) {
        std::cout << "Failed to link program with shaders: ";
        for (Shader& shader : shaders)
            std::cout << shader.file_path << ", ";
        std::cout << std::endl;

        int log_len = 0;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &log_len);
        const std::unique_ptr<char[]> log(new char[log_len]);
        glGetProgramInfoLog(program, log_len, nullptr, log.get());
        std::cout << log.get() << std::endl;

        return false;
    }
    valid = true;
    return true;
}

void ShaderProgram::use() const {
    if (is_valid()) {
        glUseProgram(program);
    } else {
        std::cerr << "The OpenGL program object is invalid (use)." << std::endl;
    }
}

void ShaderProgram::delete_program() {
    if (is_valid()) {
        valid = false;
        glDeleteProgram(program);
    }
}

// ----------------------------------------------------------------------------
// Add Methods
// ----------------------------------------------------------------------------
bool ShaderProgram::add_shader(GLenum shader_type, const std::filesystem::path& file_name) {
    if (!program) {
        std::cerr << "The OpenGL program object is invalid (add_shader)." << std::endl;
        return false;
    }

    // Loads and compiles the shader.
    Shader& shader = shaders.emplace_back(shader_type, file_name);

    if (shader.shader) {
        glAttachShader(program, shader.shader);
        return true;
    }
    return false;
}

bool ShaderProgram::add_vertex_shader(const std::filesystem::path& file_name) { return add_shader(GL_VERTEX_SHADER, file_name); }

bool ShaderProgram::add_fragment_shader(const std::filesystem::path& file_name) { return add_shader(GL_FRAGMENT_SHADER, file_name); }

bool ShaderProgram::add_compute_shader(const std::filesystem::path& file_name) { return add_shader(GL_COMPUTE_SHADER, file_name); }

bool ShaderProgram::add_geometry_shader(const std::filesystem::path& file_name) { return add_shader(GL_GEOMETRY_SHADER, file_name); }

bool ShaderProgram::add_tess_control_shader(const std::filesystem::path& file_name) { return add_shader(GL_TESS_CONTROL_SHADER, file_name); }

bool ShaderProgram::add_tess_evaluation_shader(const std::filesystem::path& file_name) { return add_shader(GL_TESS_EVALUATION_SHADER, file_name); }

// ----------------------------------------------------------------------------
// Binding Methods
// ----------------------------------------------------------------------------
void ShaderProgram::bind_attrib_location(GLint index, std::string_view name) const {
    if (!program) {
        std::cerr << "The OpenGL program object is invalid (bind_attrib_location)." << std::endl;
        return;
    }
    glBindAttribLocation(program, index, name.data());
}

void ShaderProgram::bind_frag_data_location(GLint idx, std::string_view name) const {
    if (!program) {
        std::cerr << "The OpenGL program object is invalid (bind_frag_data_location)." << std::endl;
        return;
    }
    glBindFragDataLocation(program, idx, name.data());
}

bool ShaderProgram::is_valid() const { return valid; }

GLuint ShaderProgram::get_opengl_program() const { return program; }

GLint ShaderProgram::get_attrib_location(std::string_view name) const {
    if (!program) {
        std::cerr << "The OpenGL program object is invalid (get_attrib_location)." << std::endl;
        return -1;
    }

    return glGetAttribLocation(program, name.data());
}

GLint ShaderProgram::get_uniform_location(std::string_view name) const {
    if (!program) {
        std::cerr << "The OpenGL program object is invalid (get_uniform_location)." << std::endl;
        return -1;
    }

    return glGetUniformLocation(program, name.data());
}

GLuint ShaderProgram::get_uniform_block_index(std::string_view name) const {
    if (!program) {
        std::cerr << "The OpenGL program object is invalid (get_uniform_block_index)." << std::endl;
        return GL_INVALID_INDEX;
    }
    return glGetUniformBlockIndex(program, name.data());
}
