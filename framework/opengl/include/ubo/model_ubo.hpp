// ###############################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
//
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#pragma once

#include "glm/glm.hpp"
#include "opengl_utils.hpp"
#include "ubo.hpp"

/**
 * The structure representing data for a single model/geometry/object. It stores the position of the
 * object in the world via {@link ModelData::model} matrix. The structure also contains derivations of
 * this matrix for convenience.
 *
 * @author	<a href="mailto:jan.byska@gmail.com">Jan Byška</a>
 * @author	<a href="mailto:cejka.honza@gmail.com ">Jan Čejka</a>
 */
struct ModelData {
    // ----------------------------------------------------------------------------
    // Variables
    // ----------------------------------------------------------------------------
    /** The model matrix.*/
    glm::mat4 model;
    /** The inverse of the model matrix. */
    glm::mat4 model_inv;
    /**The inverse of the transpose of the top-left part 3x3 of the model matrix.*/
    glm::mat3x4 model_it;

    // ----------------------------------------------------------------------------
    // Constructors
    // ----------------------------------------------------------------------------
    /** Constructs a new @link ModelData with identity matrix. */
    ModelData() : ModelData(glm::mat4(1.0f)) {}

    /**
     * Constructs a new {@link ModelData} with a custom matrix.
     *
     * @param 	model	The model matrix.
     */
    ModelData(const glm::mat4& model) : model(model) {
        model_inv = inverse(model);
        model_it = glm::mat3x4(transpose(inverse(glm::mat3(model))));
    }
};

/**
 * Contains the model matrix, which defines the position of the object in the world, and derivations of
 * this matrix.
 *
 * Use this code in shaders for a single object:
 * <code>
 * layout(std140, binding = 1) uniform ModelDataBuffer
 * {
 *  mat4 model;			// The model matrix.
 *  mat4 model_inv;		// The inverse of the model matrix.
 *  mat3 model_it;		// The inverse of the transpose of the top-left part 3x3 of the model matrix.
 * };
 *
 * or a set of multiple objects
 *
 * struct ModelData
 * {
 *  mat4 model;			// The model matrix.
 *  mat4 model_inv;		// The inverse of the model matrix.
 *  mat3 model_it;		// The inverse of the transpose of the top-left part 3x3 of the model matrix.
 * };
 * layout(std140, binding = 1) uniform ModelDataBuffer
 * {
 *  ModelData models[#count];
 * };
 * </code>
 */
class ModelUBO : public UBO<ModelData> {
    // ----------------------------------------------------------------------------
    // Layout Asserts
    // ----------------------------------------------------------------------------
    static_assert(offsetof(ModelData, model) == 0, "Incorrect ModelData layout");
    static_assert(offsetof(ModelData, model_inv) == 64, "Incorrect ModelData layout");
    static_assert(offsetof(ModelData, model_it) == 128, "Incorrect ModelData layout");
    static_assert(sizeof(ModelData) == 176, "Incorrect ModelData layout");

    // ----------------------------------------------------------------------------
    // Constructors
    // ----------------------------------------------------------------------------
  public:
    /** Constructs a new @link ModelUBO with identity matrix. */
    ModelUBO() : ModelUBO(glm::mat4(1.0)) {}

    /** Constructs a new @link ModelUBO with custom matrix. Note that the data are immediately copied to the GPU. */
    ModelUBO(const glm::mat4& model)
        : UBO<ModelData>(OpenGLUtils::get_opengl_version() >= 4.5f ? GL_DYNAMIC_STORAGE_BIT : GL_DYNAMIC_DRAW) {
        set_matrix(model);
        update_opengl_data();
    }

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
    ModelUBO& operator=(ModelUBO other) {
        swap_fields(*this, other);
        return *this;
    }

    // ----------------------------------------------------------------------------
    // Getters & Setters
    // ----------------------------------------------------------------------------
  public:
    /**
     * Sets a new model matrix (and its derivations) to the first position in the buffer.
     *
     * @param 	model	The model matrix to set.
     */
    void set_matrix(const glm::mat4& model) { set_matrix(0, model); }

    /**
     * Sets a new model matrix (and its derivations) at a specified position in the buffer.
     *
     * @param 	idx  	Zero-based index of position in the buffer that will be updated.
     * @param 	model	The model matrix to set.
     */
    void set_matrix(int idx, const glm::mat4& model) { data[idx] = ModelData(model); }

    [[nodiscard]] glm::mat4 get_matrix() const { return data[0].model; }
};
