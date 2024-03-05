// ###############################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
//
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#pragma once

#include "cube.hpp"
#include "geometry.hpp"
#include "model_ubo.hpp"
#include "phong_material_ubo.hpp"
#include <memory>

/**
 * The most basic representation of a scene object.
 *
 * @author	<a href="mailto:jan.byska@gmail.com">Jan Byška</a>
 */
class SceneObject {
    // ----------------------------------------------------------------------------
    // Variables
    // ----------------------------------------------------------------------------
  protected:
    /** The geometry representation of the scene object. */
    std::shared_ptr<Geometry> geometry;
    /** The UBO containing the model data. */
    std::shared_ptr<ModelUBO> model_ubo;
    /** The UBO containing the material. */
    std::shared_ptr<PhongMaterialUBO> material;
    /** The texture that should be used when rendering this object. */
    GLuint texture;

    // ----------------------------------------------------------------------------
    // Constructors
    // ----------------------------------------------------------------------------
  public:
    /**
     * Constructs a default @link SceneObject representing a cube.
     */
    SceneObject() : SceneObject(Cube(), ModelUBO(), PhongMaterialUBO()) {}

    /**
     * Constructs a new @link SceneObject with custom values.
     *
     * @param 	geometry 	The geometry representation of the scene object.
     * @param 	model_ubo	The UBO containing the model data.
     * @param 	material 	The UBO containing the material of the scene object.
     */
    SceneObject(const Geometry& geometry, const ModelUBO& model_ubo, const PhongMaterialUBO& material, GLuint texture = 0)
        : SceneObject(std::make_shared<Geometry>(geometry), std::make_shared<ModelUBO>(model_ubo),
                      std::make_shared<PhongMaterialUBO>(material), texture) {}

    /**
     * Constructs a new @link SceneObject with custom values.
     *
     * @param 	geometry 	The geometry representation of the scene object.
     * @param 	model_ubo	The UBO containing the model data.
     * @param 	material 	The UBO containing the material of the scene object.
     */
    SceneObject(std::shared_ptr<Geometry> geometry, const ModelUBO& model_ubo, const PhongMaterialUBO& material, GLuint texture = 0)
        : SceneObject(geometry, std::make_shared<ModelUBO>(model_ubo), std::make_shared<PhongMaterialUBO>(material), texture) {}

    /**
     * Constructs a new @link SceneObject with custom values (using directly the shared pointers).
     *
     * @param 	geometry 	The geometry representation of the scene object.
     * @param 	model_ubo	The UBO containing the model data.
     * @param 	material 	The UBO containing the material of the scene object.
     */
    SceneObject(std::shared_ptr<Geometry> geometry, std::shared_ptr<ModelUBO> model_ubo, std::shared_ptr<PhongMaterialUBO> material,
                GLuint texture = 0)
        : geometry(geometry), model_ubo(model_ubo), material(material), texture(texture) {}

    // ----------------------------------------------------------------------------
    // Getters & Setters
    // ----------------------------------------------------------------------------
    /**
     * Returns the geometry representation of the scene object.
     *
     * @return	The geometry.
     */
    Geometry& get_geometry() const { return *this->geometry; }

    /**
     * Returns the geometry representation of the scene object as pointer.
     *
     * @return	The geometry.
     */
    std::shared_ptr<Geometry>& get_geometry_ptr() { return this->geometry; }

    /**
     * Returns the UBO containing the model data.
     *
     * @return	The model UBO.
     */
    ModelUBO& get_model_ubo() const { return *this->model_ubo; }

    /**
     * Returns the UBO containing the model data as pointer.
     *
     * @return	The model UBO.
     */
    std::shared_ptr<ModelUBO>& get_model_ubo_ptr() { return this->model_ubo; }

    /**
     * Returns the UBO containing the material.
     *
     * @return	The material UBO.
     */
    PhongMaterialUBO& get_material() const { return *this->material; }

    /**
     * Returns the UBO containing the material as pointer.
     *
     * @return	The material UBO.
     */
    std::shared_ptr<PhongMaterialUBO>& get_material_ptr() { return this->material; }

    /**
     * Checks if the scene object has a texture defined.
     *
     * @return	@p true if the object has a texture defined, @p false if not.
     */
    bool has_texture() const { return this->texture != 0; }

    /**
     * Returns the texture that should be used when rendering this object.
     *
     * @return	The texture.
     */
    GLuint get_texture() const { return this->texture; }
};
