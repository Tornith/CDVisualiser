// ###############################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
//
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#pragma once

#include "camera.h"
#include "glm/glm.hpp"
#include "ubo.hpp"
#include "opengl_utils.hpp"
#include <iostream>

/**
 * The structure representing data for a single camera. It stores the camera position, and corresponding projection and view matrices. The
 * structure also contains various derivations of these matrix for convenience.
 *
 * @author	<a href="mailto:jan.byska@gmail.com">Jan Byška</a>
 * @author	<a href="mailto:cejka.honza@gmail.com ">Jan Čejka</a>
 */
struct CameraData {
    // ----------------------------------------------------------------------------
    // Variables
    // ----------------------------------------------------------------------------
    /** The projection matrix. */
    glm::mat4 projection;
    /** The inverted projection matrix. */
    glm::mat4 projection_inv;
    /** The view matrix. */
    glm::mat4 view;
    /** The inverted view matrix. */
    glm::mat4 view_inv;
    /**The inverse of the transpose of the top-left part 3x3 of the view matrix. There is an extra data for padding. */
    glm::mat3x4 view_it;
    /** The eye position. Don't forget the padding. There is an extra data for padding. */
    glm::vec4 eye_position;

    // ----------------------------------------------------------------------------
    // Constructors
    // ----------------------------------------------------------------------------
    /** Constructs a new @link CameraData with identity matrices. */
    CameraData() : CameraData(glm::mat4(1.0f), glm::mat4(1.0f), glm::vec3(1.0f)) {}

    /**
     * Constructs a new {@link CameraData} with a custom matrix.
     *
     * @param 	projection	The projection matrix.
     * @param 	view	  	The view matrix.
     */
    CameraData(const glm::mat4& projection, const glm::mat4& view, const glm::vec3 eye_position)
        : projection(projection), view(view), eye_position(glm::vec4(eye_position, 1.0f)) {
        projection_inv = inverse(projection);
        view_inv = inverse(view);
        view_it = glm::mat3x4(transpose(inverse(glm::mat3(view))));
    }
};

/**
 * Contains the model matrix, which defines the position of the object in the world, and derivations of
 * this matrix.
 *
 * Use this code in shaders for a single object:
 * <code>
 * layout(std140, binding = 0) uniform CameraBuffer
 * {
 *   mat4 projection;		// The projection matrix.
 *	 mat4 projection_inv;	// The inverse of the projection matrix.
 *   mat4 view;				// The view matrix
 *   mat4 view_inv;			// The inverse of the view matrix.
 *   mat3 view_it;			// The inverse of the transpose of the top-left part 3x3 of the view matrix
 *   vec3 eye_position;		// The position of the eye in world space.
 * };
 *
 * or a set of multiple objects
 *
 * struct CameraData
 * {
 *   mat4 projection;		// The projection matrix.
 *	 mat4 projection_inv;	// The inverse of the projection matrix.
 *   mat4 view;				// The view matrix
 *   mat4 view_inv;			// The inverse of the view matrix.
 *   mat3 view_it;			// The inverse of the transpose of the top-left part 3x3 of the view matrix
 *   vec3 eye_position;		// The position of the eye in world space.
 * };
 * layout(std140, binding = 0) uniform CameraBuffer
 * {
 *   CameraData cameras[#count];
 * };
 * </code>
 */
class CameraUBO : public UBO<CameraData> {
    // ----------------------------------------------------------------------------
    // Layout Asserts
    // ----------------------------------------------------------------------------
    static_assert(offsetof(CameraData, projection) == 0, "Incorrect CameraData layout");
    static_assert(offsetof(CameraData, projection_inv) == 64, "Incorrect CameraData layout");
    static_assert(offsetof(CameraData, view) == 128, "Incorrect CameraData layout");
    static_assert(offsetof(CameraData, view_inv) == 192, "Incorrect CameraData layout");
    static_assert(offsetof(CameraData, view_it) == 256, "Incorrect CameraData layout");
    static_assert(offsetof(CameraData, eye_position) == 304, "Incorrect CameraData layout");
    static_assert(sizeof(CameraData) == 320, "Incorrect CameraData layout");

    // ----------------------------------------------------------------------------
    // Constructors
    // ----------------------------------------------------------------------------
  public:
    /** Constructs a new @link CameraUBO. */
    CameraUBO() : UBO<CameraData>(OpenGLUtils::get_opengl_version() >= 4.5f ? GL_DYNAMIC_STORAGE_BIT : GL_DYNAMIC_DRAW) {}

    /**
     * Constructs a new @link CameraUBO. The constructors validates the buffer layout.
     *
     * @param 	data		The data that will be stored in this buffer.
     */
    CameraUBO(std::vector<CameraData> data) : UBO<CameraData>(data, OpenGLUtils::get_opengl_version() >= 4.5f ? GL_DYNAMIC_STORAGE_BIT : GL_DYNAMIC_DRAW, GL_UNIFORM_BUFFER, cpu_only = false) {}

    // ----------------------------------------------------------------------------
    // Getters & Setters
    // ----------------------------------------------------------------------------
  public:
    /**
     * Sets a new projection matrix (and its derivations) to the first position in the buffer.
     *
     * @param 	projection_matrix	The projection matrix to set.
     */
    void set_projection(const glm::mat4& projection_matrix) { set_projection(0, projection_matrix); }

    /**
     * Sets a new projection matrix (and its derivations) at a specified position in the buffer.
     *
     * @param 	idx  	Zero-based index of position in the buffer that will be updated.
     * @param 	projection_matrix	The projection matrix to set.
     */
    void set_projection(int idx, const glm::mat4& projection_matrix) {
        data[idx].projection = projection_matrix;
        data[idx].projection_inv = inverse(projection_matrix);
    }

    /**
     * Sets a new view matrix (and its derivations) to the first position in the buffer.
     *
     * @param 	view_matrix	The view matrix to set.
     */
    void set_view(const glm::mat4& view_matrix) { set_view(0, view_matrix); }

    /**
     * Sets a new view matrix (and its derivations) at a specified position in the buffer.
     *
     * @param 	idx  	Zero-based index of position in the buffer that will be updated.
     * @param 	view_matrix	The view matrix to set.
     */
    void set_view(int idx, const glm::mat4& view_matrix) {
        data[idx].view = view_matrix;
        data[idx].view_inv = inverse(view_matrix);
        data[idx].view_it = glm::mat3x4(transpose(inverse(glm::mat3(data[idx].view))));
        data[idx].eye_position = glm::vec4(glm::vec3(data[idx].view_inv[3]), 1.0f);
    }

    /**
     * Sets a new view matrix (and its derivations) to the first position in the buffer. The view matrix is extracted from the specified
     * camera.
     *
     * @param 	camera	The camera that will be used to define the view matrix.
     */
    void set_camera(const Camera& camera) { set_camera(0, camera); }

    /**
     * Sets a new view matrix (and its derivations) at a specified position in the buffer. The view matrix is extracted from the specified
     * camera.
     *
     * @param 	idx  	Zero-based index of position in the buffer that will be updated.
     * @param 	camera	The camera that will be used to define the view matrix.
     */
    void set_camera(int idx, const Camera& camera) {
        data[idx].view = camera.get_view_matrix();
        data[idx].view_inv = inverse(data[idx].view);
        data[idx].view_it = glm::mat3x4(transpose(inverse(glm::mat3(data[idx].view))));
        data[idx].eye_position = glm::vec4(camera.get_eye_position(), 1.0f);
    }
};
