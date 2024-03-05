// ###############################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#pragma once
#include "material_ubo.hpp"

/**
 * The structure holding the information about a phong material.
 *
 * @author	<a href="mailto:jan.byska@gmail.com">Jan Byška</a>
 * @author	<a href="mailto:cejka.honza@gmail.com ">Jan Čejka</a>
 */
struct PhongMaterialData {
    /** The ambient part of the material. */
    glm::vec3 ambient;
    /** The padding to avoid problems with layouts - see PV112 lecture. */
    float padding1;
    /** The diffuse part of the material. */
    glm::vec3 diffuse;
    /** The alpha (transparency) of the material. */
    float alpha;
    /** The specular part of the material. */
    glm::vec3 specular;
    /** The shininess of the material. */
    float shininess;

    // ----------------------------------------------------------------------------
    // Constructors
    // ----------------------------------------------------------------------------
    /**
     * Constructs a simplified @link PhongMaterialData with default values. Sets ambient and diffuse to red and specular to white with shining of 200.
     */
    PhongMaterialData()
        : PhongMaterialData(glm::vec3(1.0, 0, 0), 1.0f, 200.f, true) {}

    /**
     * Constructs a simplified @link PhongMaterialData with specified values. Sets ambient and diffuse to the same basic
     * color. Specular is black if shininess is zero, white if white_specular is true, or the basic color otherwise.
     *
     * @param 	color		  	The material diffuse color.
     * @param 	alpha		  	The alpha (transparency) of the material.
     * @param 	shininess	  	The shininess of the material.
     * @param 	white_specular	If @p true the specular color will be white, otherwise the material color is used.
     */
    PhongMaterialData(const glm::vec3& color, float alpha, float shininess, bool white_specular)
        : PhongMaterialData(color, color, alpha, (shininess == 0.0f) ? glm::vec3(0.0f) : (white_specular ? glm::vec3(1.0f) : color), shininess) {}

    /**
     * Constructs a new @link PhongMaterialData with specified values.
     *
     * @param 	ambient  	The ambient part of the material.
     * @param 	diffuse  	The diffuse part of the material.
     * @param 	alpha	 	The alpha (transparency) of the material.
     * @param 	specular 	The specular part of the material.
     * @param 	shininess	The shininess of the material.
     */
    PhongMaterialData(const glm::vec3& ambient, const glm::vec3& diffuse, float alpha, const glm::vec3& specular, float shininess)
        : ambient(ambient), padding1(0), diffuse(diffuse), alpha(alpha), specular(specular), shininess(shininess) {}
};

/**
 * Contains the information about a material used in the Phong (or Blinn-Phong) illumination model.
 *
 * Use this code in shaders for a single material:
 * <code>
 * layout(std140, binding = 3) uniform PhongMaterialBuffer
 * {
 *    vec3 ambient;     // The ambient part of the material.
 *    vec3 diffuse;     // The diffuse part of the material.
 *    float alpha;      // The alpha (transparency) of the material.
 *    vec3 specular;    // The specular part of the material.
 *    float shininess;  // The shininess of the material.
 * } material;
 *
 * or a set of multiple materials
 *
 * struct PhongMaterialData
 * {
 *    vec3 ambient;     // The ambient part of the material.
 *    vec3 diffuse;     // The diffuse part of the material.
 *    float alpha;      // The alpha (transparency) of the material.
 *    vec3 specular;    // The specular part of the material.
 *    float shininess;  // The shininess of the material.
 * };
 * layout(std140, binding = 3) uniform PhongMaterialBuffer
 * {
 *    PhongMaterialData materials[#count];
 * };
 * </code>
 */
class PhongMaterialUBO : public MaterialUBO<PhongMaterialData> {
    // ----------------------------------------------------------------------------
    // Layout Asserts
    // ----------------------------------------------------------------------------
    static_assert(offsetof(PhongMaterialData, ambient) == 0, "Incorrect PhongMaterialData layout.");
    static_assert(offsetof(PhongMaterialData, diffuse) == 16, "Incorrect PhongMaterialData layout.");
    static_assert(offsetof(PhongMaterialData, alpha) == 28, "Incorrect PhongMaterialData layout.");
    static_assert(offsetof(PhongMaterialData, specular) == 32, "Incorrect PhongMaterialData layout.");
    static_assert(offsetof(PhongMaterialData, shininess) == 44, "Incorrect PhongMaterialData layout.");
    static_assert(sizeof(PhongMaterialData) == 48, "Incorrect PhongMaterialData layout.");

    // ----------------------------------------------------------------------------
    // Constructors
    // ----------------------------------------------------------------------------
public:
    /** Inherits the constructors. */
    using MaterialUBO<PhongMaterialData>::MaterialUBO;
};
