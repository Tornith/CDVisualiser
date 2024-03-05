// ###############################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#pragma once

#include "material_ubo.hpp"
#include "glm/glm.hpp"

/**
 * The structure holding the information about a PBR material.
 *
 * @author	<a href="mailto:jan.byska@gmail.com">Jan Byška</a>
 * @author	<a href="mailto:cejka.honza@gmail.com ">Jan Čejka</a>
 */
struct PBRMaterialData {
    /** The diffuse color of the material. */
    glm::vec3 diffuse;
    /** The roughness of the material. */
    float roughness;
    /** The Fresnel reflection at 0°. */
    glm::vec3 f0;
    /** The padding to avoid problems with layouts - see PV112 lecture. */
    float padding1;

    // ----------------------------------------------------------------------------
    // Constructors
    // ----------------------------------------------------------------------------
    /**
     * Constructs a simplified @link PBRMaterialData with default values representing a white plastic material.
     */
    PBRMaterialData()
        : PBRMaterialData(glm::vec3(1.0), glm::vec3(0.04f), 1.0f) {}

    /**
     * Constructs a simplified @link PBRMaterialData with specified values. 
     *
     * @param 	color		  	The material diffuse color.
     * @param 	f0		  	    The Fresnel reflection at 0°.
     * @param 	roughness	  	The roughness of the material.
     */
    PBRMaterialData(const glm::vec3& color, const glm::vec3& f0, float roughness)
        : diffuse(color), roughness(roughness), f0(f0), padding1(0) {}
};

/**
 * Contains the information about a material used in the PBR illumination models.
 *
 * Use this code in shaders for a single material:
 * <code>
 * layout(std140, binding = 3) uniform PBRMaterialBuffer
 * {
 *    vec3 diffuse;     // The diffuse color of the material.
 *    float roughness;  // The roughness of the material.
 *    vec3 f0;          // The Fresnel reflection at 0°.
 * } material;
 *
 * or a set of multiple materials
 *
 * struct PBRMaterialData
 * {
 *    vec3 diffuse;     // The diffuse color of the material.
 *    float roughness;  // The roughness of the material.
 *    vec3 f0;          // The Fresnel reflection at 0°.
 * };
 * layout(std140, binding = 3) uniform PBRMaterialBuffer
 * {
 *    PBRMaterialData materials[#count];
 * };
 * </code>
 */
class PBRMaterialUBO : public MaterialUBO<PBRMaterialData> {
    // ----------------------------------------------------------------------------
    // Layout Asserts
    // ----------------------------------------------------------------------------
    static_assert(offsetof(PBRMaterialData, diffuse) == 0, "Incorrect PBRMaterialData layout.");
    static_assert(offsetof(PBRMaterialData, roughness) == 12, "Incorrect PBRMaterialData layout.");
    static_assert(offsetof(PBRMaterialData, f0) == 16, "Incorrect PBRMaterialData layout.");
    static_assert(sizeof(PBRMaterialData) == 32, "Incorrect PBRMaterialData layout.");

    // ----------------------------------------------------------------------------
    // Constructors
    // ----------------------------------------------------------------------------
public:
    /** Inherits the constructors. */
    using MaterialUBO<PBRMaterialData>::MaterialUBO;
};
