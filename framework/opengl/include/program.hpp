// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#pragma once

#include "shader.hpp"
#include <array>
#include <filesystem>
#include <span>
#include <string>
#include <vector>
#include <glm/glm.hpp>

/**
 * This is a VERY SIMPLE class that maintains a shader program and its shaders. It is by all means not a perfect,
 * brilliant, smart, or whatever implementation. However, it suffice for most of our needs. There are several
 * methods that makes the work easier, especially:
 *
 * @link ShaderProgram::use,
 * @link ShaderProgram::get_attrib_location,
 * @link ShaderProgram::get_uniform_location,
 * @link ShaderProgram::get_uniform_block_index,
 * @link ShaderProgram::uniform_*,
 *
 * To use it, follow these steps:
 *
 * ShaderProgram my_program;
 * my_program.add_shader(...);
 * my_program.bind_attrib_location(...);
 * my_program.bind_frag_data_location(...);
 * my_program.link();
 * if (my_program.is_valid()) {
 *     my_program.use();
 * }
 * @author	<a href="mailto:jan.byska@gmail.com">Jan Byška</a>
 * @author	<a href="mailto:cejka.honza@gmail.com">Jan Čejka</a>
 * @author	<a href="mailto:matus.talcik@mail.muni.cz">Matúš Talčík</a>
 */
class ShaderProgram {
    // ----------------------------------------------------------------------------
    // Variables
    // ----------------------------------------------------------------------------
protected:
    /** List of all shaders that the program uses. */
    std::vector<Shader> shaders;

    /** The OpenGL object corresponding to this shader program. */
    GLuint program;

    /** The flag that is @p true if this shader is ready to be used. */
    bool valid;

    // ----------------------------------------------------------------------------
    // Constructors
    // ----------------------------------------------------------------------------
public:
    /**
     * Initializes an empty @link ShaderProgram. This constructors does not create the OpenGL object and can thus be used before
     * the OpenGL context is created.
     */
    ShaderProgram();

    /**
     * Initializes a new @link ShaderProgram including the OpenGL object. This constructors enables to set vertex and
     * fragment shaders and attempts to compile and link the program.
     *
     * @param 	vertex_shader  	The vertex shader.
     * @param 	fragment_shader	The fragment shader.
     */
    ShaderProgram(const std::filesystem::path& vertex_shader, const std::filesystem::path& fragment_shader);

    /**
     * Initializes a new @link ShaderProgram including the OpenGL object. 
     * This constructors copies the shaders from the provides @link ShaderProgram and attempts to compile and link the program.
     *
     * @param 	other  	The other shader program.
     */
    ShaderProgram(const ShaderProgram& other)
        : shaders(other.shaders) {
        program = glCreateProgram();

        for (const Shader& shader : other.shaders) {
            add_shader(shader.shader_type, shader.file_path);
        }

        link();
    }

    // ----------------------------------------------------------------------------
    // Operators
    // ----------------------------------------------------------------------------

    /**
    * The copy assignment using copy-and-swap idiom.
    *
    * @param other The other shader program that will be copied (i.e., swapped into this).
    * 			   
    * @return A shallow copy of the other shader program that was moved into this object.
    */
    ShaderProgram& operator=(ShaderProgram other) {
        swap_fields(*this, other);

        return *this;
    }

    ShaderProgram(ShaderProgram&& other) noexcept
        : ShaderProgram() { swap_fields(*this, other); }

    friend void swap_fields(ShaderProgram& first, ShaderProgram& second) noexcept {
        using std::swap;

        swap(first.shaders, second.shaders);
        swap(first.program, second.program);
        swap(first.valid, second.valid);
    }

    /** Destructor that automatically destroys the OpenGL object. */
    ~ShaderProgram();

    // ----------------------------------------------------------------------------
    // Methods
    // ----------------------------------------------------------------------------
public:
    /**
     * Links the program and checks for errors. When this fails, the errors are printed to stdout and the
     * program is destroyed.
     *
     * Returns true if everything is OK, false if something failed.
     *
     * @return	True if it succeeds, false if it fails.
     */
    bool link();

    /** Checks if the program is ready and if yes then it calls glUseProgram with this program as the parameter. */
    void use() const;

    /** Deletes the program. */
    void delete_program();

    // ----------------------------------------------------------------------------
    // Add Methods
    // ----------------------------------------------------------------------------
    /**
     * Adds a specified shader. Note that if the compilation fails, the program is automatically destroyed.
     *
     * Example of use: my_program.add_shader(GL_VERTEX_SHADER, "my_shader.vert");
     *
     * @param 	shader_type	The type of the shader.
     * @param 	file_name  	The name of the file with the source code.
     * @return	{@p true} if everything is OK, {@p false} if something failed.
     */
    bool add_shader(GLenum shader_type, const std::filesystem::path& file_name);

    /**
     * Adds a specified vertex shader. The method internally calls {@link ShaderProgram::add_shader} with the
     * proper shader_type.
     *
     * @param 	file_name	The name of the file with the source code.
     * @return	{@p true} if everything is OK, {@p false} if something failed.
     */
    bool add_vertex_shader(const std::filesystem::path& file_name);

    /**
     * Adds a specified fragment shader. The method internally calls {@link ShaderProgram::add_shader} with the
     * proper shader_type.
     *
     * @param 	file_name	The name of the file with the source code.
     * @return	{@p true} if everything is OK, {@p false} if something failed.
     */
    bool add_fragment_shader(const std::filesystem::path& file_name);

    /**
     * Adds a specified geometry shader. The method internally calls {@link ShaderProgram::add_shader} with the
     * proper shader_type.
     *
     * @param 	file_name	The name of the file with the source code.
     * @return	{@p true} if everything is OK, {@p false} if something failed.
     */
    bool add_geometry_shader(const std::filesystem::path& file_name);

    /**
     * Adds a specified compute shader. The method internally calls {@link ShaderProgram::add_shader} with the
     * proper shader_type.
     *
     * @param 	file_name	The name of the file with the source code.
     * @return	{@p true} if everything is OK, {@p false} if something failed.
     */
    bool add_compute_shader(const std::filesystem::path& file_name);

    /**
     * Adds a specified tessellation control shader. The method internally calls {@link ShaderProgram::add_shader} with
     * the proper shader_type.
     *
     * @param 	file_name	The name of the file with the source code.
     * @return	{@p true} if everything is OK, {@p false} if something failed.
     */
    bool add_tess_control_shader(const std::filesystem::path& file_name);

    /**
     * Adds a specified tessellation evaluation shader. The method internally calls {@link ShaderProgram::add_shader}
     * with the proper shader_type.
     *
     * @param 	file_name	The name of the file with the source code.
     * @return	{@p true} if everything is OK, {@p false} if something failed.
     */
    bool add_tess_evaluation_shader(const std::filesystem::path& file_name);

    // ----------------------------------------------------------------------------
    // Binding Methods
    // ----------------------------------------------------------------------------
    /**
     * Associates a generic vertex attribute index with a named attribute variable.
     * Calls glBindAttribLocation with this program as the first parameter.
     *
     * @param 	index 	Zero-based index of the generic vertex attribute to be bound.
     * @param 	name	The null terminated string containing the name of the vertex shader attribute variable to which
     * index is to be bound.
     */
    void bind_attrib_location(GLint index, std::string_view name) const;

    /**
     * Binds a user-defined varying out variable to a fragment shader color number
     * Calls glBindFragDataLocation with this program as the first parameter.
     *
     * @param 	idx 	Zero-based index of the color number to bind the user-defined varying out variable to.
     * @param 	name	The name of the user-defined varying out variable whose binding to modify.
     */
    void bind_frag_data_location(GLint idx, std::string_view name) const;

    // ----------------------------------------------------------------------------
    // Getters & Setters
    // ----------------------------------------------------------------------------
    /**
     * Checks if the shader program is linked and ready to be used.
     *
     * @return	{@p true} if the shader program is linked and ready to be used, {@p false} if it is not.
     */
    bool is_valid() const;

    /**
     * Returns the OpenGL object of the shader program.
     *
     * Note that the object may be non-zero even when is_valid returns false. This happens mainly between calls
     * to init and link. For this reason, use is_valid to check whether the program is ready to use.
     *
     * @return The OpenGL object of the shader program.
     */
    GLuint get_opengl_program() const;

    /**
     * Returns the location of an attribute variable. Calls glGetAttribLocation with this program as the first
     * parameter.
     *
     * @param 	name	The name of the attribute variable whose location is to be queried.
     * @return	The location of the attribute variable.
     */
    GLint get_attrib_location(std::string_view name) const;

    /**
     * Returns the location of a uniform variable. Calls glGetUniformLocation with this program as the first
     * parameter.
     *
     * @param 	name	The name of the uniform variable whose location is to be queried.
     * @return	The location of the uniform variable.
     */
    GLint get_uniform_location(std::string_view name) const;

    /**
     * Retrieves the index of a named uniform block. Calls glGetUniformBlockIndex with this program as the
     * first parameter.
     *
     * @param 	name	The name of the uniform block whose index to retrieve.
     * @return	The index of the specified named uniform block.
     */
    GLuint get_uniform_block_index(std::string_view name) const;

    // Similar to glProgramUniform*(program, my_program.GetUniformLocation(name)/unit, ...);
    //
    // uniform(...) covers everything from glProgramUniform{1|2|3|4}{f|i|ui}
    // uniform_matrix(...) covers everything from glProgramUniformMatrix{2|3|4}x{2|3|4}fv
    //
    // Bevare of variants with names instead of direct locations. They incur performance hit due to lookup of the location.
    //
    // General forms are: uniform(name/location, v0, [v1], [v2] [v3])
    //                    uniform(name/location, glm:tvec2/tvec3/tvec4/std::array2/3/4/std::span)
    //                    uniform_array(name/location, {glm::vec2(1.0f), glm::vec2(2.0f)})
    //                    uniform_matrix(name/location, glm::mat4(1.0f))
    //
    // std::span variant allows calling uniforms with raw c-style arrays and std dynamic containers like std::vector
    //
    // Examples: my_program.uniform("time", 0.7f);
    //           my_program.uniform(0, 2.0f, 3.0f, 4.0f);
    //           my_program.uniform(1, glm::vec3(1.0f));
    //

#define UNIFORM_FUNCTIONS_TYPE(T, GL_T)                                                                                          \
    void uniform(uint32_t location, T v0) const;                                                                                 \
    void uniform(uint32_t location, T v0, T v1) const;                                                                           \
    void uniform(uint32_t location, T v0, T v1, T v2) const;                                                                     \
    void uniform(uint32_t location, T v0, T v1, T v2, T v3) const;                                                               \
                                                                                                                                 \
    void uniform(std::string_view name, T v0) const;                                                                              \
    void uniform(std::string_view name, T v0, T v1) const;                                                                       \
    void uniform(std::string_view name, T v0, T v1, T v2) const;                                                                 \
    void uniform(std::string_view name, T v0, T v1, T v2, T v3) const;                                                           \
                                                                                                                                 \
    void uniform(uint32_t location, glm::tvec2<T> v) const;                                                                      \
    void uniform(uint32_t location, glm::tvec3<T> v) const;                                                                      \
    void uniform(uint32_t location, glm::tvec4<T> v) const;                                                                      \
                                                                                                                                 \
    void uniform(std::string_view name, glm::tvec2<T> v) const;                                                                  \
    void uniform(std::string_view name, glm::tvec3<T> v) const;                                                                  \
    void uniform(std::string_view name, glm::tvec4<T> v) const;                                                                  \
                                                                                                                                 \
    void uniform(uint32_t location, std::array<T, 2> v) const;                                                                   \
    void uniform(uint32_t location, std::array<T, 3> v) const;                                                                   \
    void uniform(uint32_t location, std::array<T, 4> v) const;                                                                   \
                                                                                                                                 \
    void uniform(std::string_view name, std::array<T, 2> v) const;                                                               \
    void uniform(std::string_view name, std::array<T, 3> v) const;                                                               \
    void uniform(std::string_view name, std::array<T, 4> v) const;                                                               \
    void uniform(uint32_t location, std::span<T> v) const;                                                                       \
    void uniform(std::string_view name, std::span<T> v) const;

    UNIFORM_FUNCTIONS_TYPE(float, f);

    UNIFORM_FUNCTIONS_TYPE(int32_t, i);

    UNIFORM_FUNCTIONS_TYPE(uint32_t, ui);

#define UNIFORM_ARRAY_FUNCTIONS_TYPE(T, GL_T)                                                                                    \
    void uniform_array(uint32_t location, std::span<T> values) const;                                                            \
    void uniform_array(uint32_t location, std::span<glm::tvec2<T>> values) const;                                                \
    void uniform_array(uint32_t location, std::span<glm::tvec3<T>> values) const;                                                \
    void uniform_array(uint32_t location, std::span<glm::tvec4<T>> values) const;                                                \
    void uniform_array(std::string_view name, std::span<T> values) const;                                                        \
    void uniform_array(std::string_view name, std::span<glm::tvec2<T>> values) const;                                            \
    void uniform_array(std::string_view name, std::span<glm::tvec3<T>> values) const;                                            \
    void uniform_array(std::string_view name, std::span<glm::tvec4<T>> values) const;

    UNIFORM_ARRAY_FUNCTIONS_TYPE(float, f);

    UNIFORM_ARRAY_FUNCTIONS_TYPE(int32_t, i);

    UNIFORM_ARRAY_FUNCTIONS_TYPE(uint32_t, ui);

#define UNIFORM_MATRIX_TYPE(T, GL_T)                                                                                             \
    void uniform_matrix(uint32_t location, T matrix, bool transpose = false) const;                                              \
    void uniform_matrix(uint32_t location, std::span<T> matrices, bool transpose = false) const;                                 \
    void uniform_matrix(std::string_view name, T matrix, bool transpose = false) const;                                          \
    void uniform_matrix(std::string_view name, std::span<T> matrices, bool transpose = false) const;

    UNIFORM_MATRIX_TYPE(glm::mat2x2, 2);

    UNIFORM_MATRIX_TYPE(glm::mat3x3, 3);

    UNIFORM_MATRIX_TYPE(glm::mat4x4, 4);

    UNIFORM_MATRIX_TYPE(glm::mat2x3, 2x3);

    UNIFORM_MATRIX_TYPE(glm::mat3x2, 3x2);

    UNIFORM_MATRIX_TYPE(glm::mat2x4, 2x4);

    UNIFORM_MATRIX_TYPE(glm::mat4x2, 4x2);

    UNIFORM_MATRIX_TYPE(glm::mat3x4, 3x4);

    UNIFORM_MATRIX_TYPE(glm::mat4x3, 4x3);
};
