// ###############################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
//
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#include "shader.hpp"
#include "shader_utils.hpp"
#include <iostream>
#include <string>

// ----------------------------------------------------------------------------
// Constructors
// ----------------------------------------------------------------------------
Shader::Shader(GLenum shader_type, const std::filesystem::path& file_path) : shader_type(shader_type), file_path(file_path) {
    // Loads the source code file from the disk.
    this->file_path.make_preferred();
    const std::string s_source = ShaderUtils::load_shader(this->file_path.generic_string());
    if (s_source.empty()) {
        std::cout << "File " << file_path << " is empty or failed to load" << std::endl;
        return;
    }

    // Creates a shader object, sets the source and tries to compile it.
    shader = glCreateShader(shader_type);
    const char* source = s_source.c_str();
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    // Checks the compilation status.
    int compile_status;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compile_status);

    // Prints errors and delete the shader if the compilation failed.
    if (GL_FALSE == compile_status) {
        switch (shader_type) {
        case GL_VERTEX_SHADER:
            std::cout << "Failed to compile vertex shader. " << file_path << std::endl;
            break;
        case GL_FRAGMENT_SHADER:
            std::cout << "Failed to compile fragment shader. " << file_path << std::endl;
            break;
        case GL_GEOMETRY_SHADER:
            std::cout << "Failed to compile geometry shader. " << file_path << std::endl;
            break;
        case GL_TESS_CONTROL_SHADER:
            std::cout << "Failed to compile tessellation control shader. " << file_path << std::endl;
            break;
        case GL_TESS_EVALUATION_SHADER:
            std::cout << "Failed to compile tessellation evaluation shader. " << file_path << std::endl;
            break;
        case GL_COMPUTE_SHADER:
            std::cout << "Failed to compile compute shader. " << file_path << std::endl;
            break;
        default:
            std::cout << "Failed to compile shader. " << file_path << std::endl;
            break;
        }

        int log_len = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &log_len);
        const std::unique_ptr<char[]> log(new char[log_len]);
        glGetShaderInfoLog(shader, log_len, nullptr, log.get());
        std::cout << log.get() << std::endl;

        glDeleteShader(shader);
        shader = 0;
    }
}

Shader::Shader(const Shader& other) : Shader(other.shader_type, other.file_path) {}

Shader& Shader::operator=(Shader other) {
    swap_fields(*this, other);
    return *this;
}

Shader::Shader(Shader&& other) : Shader() { swap_fields(*this, other); }

Shader::~Shader() { glDeleteShader(shader); }
