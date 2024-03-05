// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#include "program.hpp"

#define UNIFORM_FUNCTIONS_TYPE_IMPL(T, GL_T)                                                                                                    \
    void ShaderProgram::uniform(uint32_t location, T v0) const { glProgramUniform1##GL_T(program, location, v0); };                             \
    void ShaderProgram::uniform(uint32_t location, T v0, T v1) const { glProgramUniform2##GL_T(program, location, v0, v1); };                   \
    void ShaderProgram::uniform(uint32_t location, T v0, T v1, T v2) const { glProgramUniform3##GL_T(program, location, v0, v1, v2); };         \
    void ShaderProgram::uniform(uint32_t location, T v0, T v1, T v2, T v3) const {                                                              \
        glProgramUniform4##GL_T(program, location, v0, v1, v2, v3);                                                                             \
    };                                                                                                                                          \
                                                                                                                                                \
    void ShaderProgram::uniform(std::string_view name, T v0) const { uniform(get_uniform_location(name), v0); };                                \
    void ShaderProgram::uniform(std::string_view name, T v0, T v1) const { uniform(get_uniform_location(name), v0, v1); };                      \
    void ShaderProgram::uniform(std::string_view name, T v0, T v1, T v2) const { uniform(get_uniform_location(name), v0, v1, v2); };            \
    void ShaderProgram::uniform(std::string_view name, T v0, T v1, T v2, T v3) const { uniform(get_uniform_location(name), v0, v1, v2, v3); };  \
                                                                                                                                                \
    void ShaderProgram::uniform(uint32_t location, glm::tvec2<T> v) const { glProgramUniform2##GL_T(program, location, v.x, v.y); };            \
    void ShaderProgram::uniform(uint32_t location, glm::tvec3<T> v) const { glProgramUniform3##GL_T(program, location, v.x, v.y, v.z); };       \
    void ShaderProgram::uniform(uint32_t location, glm::tvec4<T> v) const {                                                                     \
        glProgramUniform4##GL_T(program, location, v.x, v.y, v.z, v.w);                                                                         \
    };                                                                                                                                          \
                                                                                                                                                \
    void ShaderProgram::uniform(std::string_view name, glm::tvec2<T> v) const { uniform(get_uniform_location(name), v.x, v.y); };               \
    void ShaderProgram::uniform(std::string_view name, glm::tvec3<T> v) const { uniform(get_uniform_location(name), v.x, v.y, v.z); };          \
    void ShaderProgram::uniform(std::string_view name, glm::tvec4<T> v) const { uniform(get_uniform_location(name), v.x, v.y, v.z, v.w); };     \
                                                                                                                                                \
    void ShaderProgram::uniform(uint32_t location, std::array<T, 2> v) const { uniform(location, v[0], v[1]); };                                \
    void ShaderProgram::uniform(uint32_t location, std::array<T, 3> v) const { uniform(location, v[0], v[1], v[2]); }                           \
    void ShaderProgram::uniform(uint32_t location, std::array<T, 4> v) const { uniform(location, v[0], v[1], v[2], v[3]); }                     \
                                                                                                                                                \
    void ShaderProgram::uniform(std::string_view name, std::array<T, 2> v) const { uniform(get_uniform_location(name), v[0], v[1]); };          \
    void ShaderProgram::uniform(std::string_view name, std::array<T, 3> v) const { uniform(get_uniform_location(name), v[0], v[1], v[2]); };    \
    void ShaderProgram::uniform(std::string_view name, std::array<T, 4> v) const {                                                              \
        uniform(get_uniform_location(name), v[0], v[1], v[2], v[3]);                                                                            \
    };                                                                                                                                          \
    void ShaderProgram::uniform(uint32_t location, std::span<T> v) const {                                                                      \
        switch (v.size()) {                                                                                                                     \
        case 2: {                                                                                                                               \
            uniform(location, v[0], v[1]);                                                                                                      \
            break;                                                                                                                              \
        }                                                                                                                                       \
        case 3: {                                                                                                                               \
            uniform(location, v[0], v[1], v[2]);                                                                                                \
            break;                                                                                                                              \
        }                                                                                                                                       \
        case 4: {                                                                                                                               \
            uniform(location, v[0], v[1], v[2], v[3]);                                                                                          \
            break;                                                                                                                              \
        }                                                                                                                                       \
        default: {                                                                                                                              \
        }                                                                                                                                       \
        }                                                                                                                                       \
    };                                                                                                                                          \
    void ShaderProgram::uniform(std::string_view name, std::span<T> v) const { uniform(get_uniform_location(name), v); }

UNIFORM_FUNCTIONS_TYPE_IMPL(float, f);

UNIFORM_FUNCTIONS_TYPE_IMPL(int32_t, i);

UNIFORM_FUNCTIONS_TYPE_IMPL(uint32_t, ui);

#define UNIFORM_ARRAY_FUNCTIONS_TYPE_IMPL(T, GL_T)                                                                                     \
    void ShaderProgram::uniform_array(uint32_t location, std::span<T> values) const {                                                  \
        glProgramUniform1##GL_T##v(program, location, (GLsizei)values.size(), values.data());                                          \
    };                                                                                                                                 \
    void ShaderProgram::uniform_array(uint32_t location, std::span<glm::tvec2<T>> values) const {                                      \
        glProgramUniform2##GL_T##v(program, location, (GLsizei)values.size(), &(values[0][0]));                                        \
    };                                                                                                                                 \
    void ShaderProgram::uniform_array(uint32_t location, std::span<glm::tvec3<T>> values) const {                                      \
        glProgramUniform3##GL_T##v(program, location, (GLsizei)values.size(), &(values[0][0]));                                        \
    };                                                                                                                                 \
    void ShaderProgram::uniform_array(uint32_t location, std::span<glm::tvec4<T>> values) const {                                      \
        glProgramUniform4##GL_T##v(program, location, (GLsizei)values.size(), &(values[0][0]));                                        \
    };                                                                                                                                 \
    void ShaderProgram::uniform_array(std::string_view name, std::span<T> values) const {                                              \
        uniform_array(get_uniform_location(name), values);                                                                             \
    }                                                                                                                                  \
    void ShaderProgram::uniform_array(std::string_view name, std::span<glm::tvec2<T>> values) const {                                  \
        uniform_array(get_uniform_location(name), values);                                                                             \
    }                                                                                                                                  \
    void ShaderProgram::ShaderProgram::uniform_array(std::string_view name, std::span<glm::tvec3<T>> values) const {                   \
        uniform_array(get_uniform_location(name), values);                                                                             \
    }                                                                                                                                  \
    void ShaderProgram::uniform_array(std::string_view name, std::span<glm::tvec4<T>> values) const {                                  \
        uniform_array(get_uniform_location(name), values);                                                                             \
    }

UNIFORM_ARRAY_FUNCTIONS_TYPE_IMPL(float, f);

UNIFORM_ARRAY_FUNCTIONS_TYPE_IMPL(int32_t, i);

UNIFORM_ARRAY_FUNCTIONS_TYPE_IMPL(uint32_t, ui);

#define UNIFORM_MATRIX_TYPE_IMPL(T, GL_T)                                                                                              \
    void ShaderProgram::uniform_matrix(uint32_t location, T matrix, bool transpose) const {                                            \
        glProgramUniformMatrix##GL_T##fv(program, location, 1, transpose, &(matrix[0][0]));                                            \
    }                                                                                                                                  \
    void ShaderProgram::uniform_matrix(uint32_t location, std::span<T> matrices, bool transpose) const {                               \
        glProgramUniformMatrix##GL_T##fv(program, location, static_cast<GLsizei>(matrices.size()), transpose,                          \
                                         &(matrices[0][0][0]));                                                                        \
    }                                                                                                                                  \
    void ShaderProgram::uniform_matrix(std::string_view name, T matrix, bool transpose) const {                                        \
        uniform_matrix(get_uniform_location(name), matrix, transpose);                                                                 \
    }                                                                                                                                  \
    void ShaderProgram::uniform_matrix(std::string_view name, std::span<T> matrices, bool transpose) const {                           \
        uniform_matrix(get_uniform_location(name), matrices, transpose);                                                               \
    }

UNIFORM_MATRIX_TYPE_IMPL(glm::mat2x2, 2);

UNIFORM_MATRIX_TYPE_IMPL(glm::mat3x3, 3);

UNIFORM_MATRIX_TYPE_IMPL(glm::mat4x4, 4);

UNIFORM_MATRIX_TYPE_IMPL(glm::mat2x3, 2x3);

UNIFORM_MATRIX_TYPE_IMPL(glm::mat3x2, 3x2);

UNIFORM_MATRIX_TYPE_IMPL(glm::mat2x4, 2x4);

UNIFORM_MATRIX_TYPE_IMPL(glm::mat4x2, 4x2);

UNIFORM_MATRIX_TYPE_IMPL(glm::mat3x4, 3x4);

UNIFORM_MATRIX_TYPE_IMPL(glm::mat4x3, 4x3);