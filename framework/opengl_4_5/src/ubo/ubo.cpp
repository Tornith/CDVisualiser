// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#include "ubo.hpp"
#include "ubo_impl.hpp"
#include "camera_ubo.hpp"
#include "light_ubo.hpp"
#include "model_ubo.hpp"
#include "pbr_material_ubo.hpp"
#include "phong_material_ubo.hpp"

// A dummy function that forces the compiler to generate and compile functions from ubo_impl.hpp for each required type.
void CompileFunctionsForFollowingTypes ()
{
    UBO<ModelData> ModelData;
    UBO<CameraData> CameraData;
    UBO<PhongLightData> PhongLightData;
    UBO<PhongMaterialData> PhongMaterialData;
    UBO<PBRMaterialUBO> PBRMaterialUBO;
}