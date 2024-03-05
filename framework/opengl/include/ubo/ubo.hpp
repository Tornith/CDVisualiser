// ###############################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
//
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#pragma once

#include "opengl_object.hpp"
#include <cassert>
#include <iostream>
#include <vector>

/**
 * The base class for representing uniform buffer objects.
 *
 * @author	<a href="mailto:jan.byska@gmail.com">Jan Byška</a>
 * @author	<a href="mailto:cejka.honza@gmail.com ">Jan Čejka</a>
 */
template <class T> class UBO : public OpenGLObject {
    // ----------------------------------------------------------------------------
    // Static Variables
    // ----------------------------------------------------------------------------
  public:
    // The following constants define the default binding points of common uniform blocks.
    // Make sure these numbers corresponds to layout(binding=N) in shaders (or use glUniformBlockBinding).
    /** The default binding for UBOs with the camera data. */
    const static int DEFAULT_CAMERA_BINDING = 0;
    /** The default binding for UBOs with the model data. */
    const static int DEFAULT_MODEL_BINDING = 1;
    /** The default binding for UBOs with the lights data. */
    const static int DEFAULT_LIGHTS_BINDING = 2;
    /** The default binding for UBOs with the material data. */
    const static int DEFAULT_MATERIAL_BINDING = 3;

    // ----------------------------------------------------------------------------
    // Variables
    // ----------------------------------------------------------------------------
  protected:
    /** The buffer data. */
    std::vector<T> data;

    /** The UBO flags. */
    GLbitfield flags;

    // ----------------------------------------------------------------------------
    // Constructors & Destructors
    // ----------------------------------------------------------------------------

  public:
    /**
     * Constructs a new @link UBO and initializes the GPU buffer with default data.
     *
     * @param 	flags   	The OpenGL flags used when creating the buffer storage.
     * @param 	target  	The target to which this object can be bound.
     * @param 	cpu_only	The flag determining if the object will be CPU only (should be @p true for tests as they do
     * 						not have OpenGL context).
     */
    UBO(GLbitfield flags = 0, GLenum target = GL_UNIFORM_BUFFER, bool cpu_only = false)
        : OpenGLObject(target, cpu_only), data(1), flags(flags) {
        create_ubo_on_gpu();
    }

    /**
     * Constructs a new @link UBO and initializes the GPU buffer with custom data.
     *
     * @param 	content		The content that will be stored in this buffer.
     * @param 	flags   	The OpenGL flags used when creating the buffer storage.
     * @param 	target  	The target to which this object can be bound.
     * @param 	cpu_only	The flag determining if the object will be CPU only (should be @p true for tests as they do
     * 						not have OpenGL context).
     */
    UBO(const T& content, GLbitfield flags = 0, GLenum target = GL_UNIFORM_BUFFER, bool cpu_only = false)
        : OpenGLObject(target, cpu_only), data({1, content}), flags(flags) {
        create_ubo_on_gpu();
    }

    /**
     * Constructs a new @link UBO and initializes the GPU buffer with custom data.
     *
     * @param 	content		The data that will be stored in this buffer.
     * @param 	flags   	The OpenGL flags used when creating the buffer storage.
     * @param 	target  	The target to which this object can be bound.
     * @param 	cpu_only	The flag determining if the object will be CPU only (should be @p true for tests as they do
     * 						not have OpenGL context).
     */
    UBO(const std::vector<T>& content, GLbitfield flags = 0, GLenum target = GL_UNIFORM_BUFFER, bool cpu_only = false)
        : OpenGLObject(target, cpu_only), data(content), flags(flags) {
        create_ubo_on_gpu();
    }

    /**
     * Constructs a new @link UBO from another (copy constructor that performs a deep copy of the buffer).
     *
     * @param 	other	The other buffer that will be copied.
     */
    UBO(const UBO& other) : OpenGLObject(other), data(other.data), flags(other.flags) { create_ubo_on_gpu(); }

    /**
     * The move constructor that moves the buffer to the new object.
     * <p>
     * Zeroes out the old object so that OpenGL's destructor does nothing (the zeroing is done via initializing the
     * object with empty constructor).
     *
     * @param 	other	The other buffer that will be moved.
     */
    UBO(UBO&& other) noexcept : UBO(0, true /* no need to initialize OpenGL while moving */) { swap_fields(*this, other); }

    /**
     * Destroys this @link UBO. Note that the OpenGL counterpart is also destroyed.
     */
    virtual ~UBO() { glDeleteBuffers(1, &opengl_object); }

  private:
    /**
     * Creates a GPU representation of the UBO.
     * Note that the implementation can be found in respective OpenGL 3.3 or 4.5 modules.
     */
    void create_ubo_on_gpu();

    // ----------------------------------------------------------------------------
    // Operators
    // ----------------------------------------------------------------------------
  public:
    /**
     * The copy assignment operator using copy-and-swap idiom.
     *
     * @param other The other buffer that will be copied (i.e., swapped into this).
     *
     * @return A shallow copy of the other buffer that was moved into this object.
     */
    UBO& operator=(UBO other) {
        swap_fields(*this, other);
        return *this;
    }

    // ----------------------------------------------------------------------------
    // Methods
    // ----------------------------------------------------------------------------
  public:
    /**
     * The custom swap method that exchanges the values of fields of two buffers.
     *
     * @param 	first 	The first buffer to swap.
     * @param 	second	The second buffer to swap.
     */
    void swap_fields(UBO& first, UBO& second) noexcept {
        OpenGLObject::swap_fields(first, second);
        std::swap(first.data, second.data);
        std::swap(first.flags, second.flags);
    }

    /**
     * Copies the data from CPU to GPU.
     * Note that the implementation can be found in respective OpenGL 3.3 or 4.5 modules.
     */
    void update_opengl_data() override;

    /**
     * Binds the buffer object to a specified indexed buffer target (binding point within the array specified by @link target).
     *
     * @param 	index	The binding point within the array specified by @link target.
     */
    void bind_buffer_base(GLuint index) const { glBindBufferBase(target, index, opengl_object); }

    // ----------------------------------------------------------------------------
    // Getters & Setters
    // ----------------------------------------------------------------------------
  public:
    /** Returns the data currently stored in this buffer. */
    const std::vector<T>& get_data() { return this->data; }
};
