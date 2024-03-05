// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#include "light_ubo.hpp"

void PhongLightsUBO::create_ubo_on_gpu() {
    glCreateBuffers(1, &opengl_object);
    glNamedBufferStorage(opengl_object, sizeof(PhongLightsMetaData) + sizeof(PhongLightData) * max_lights_count, nullptr, flags);
}

void PhongLightsUBO::update_opengl_data() {
    // Crashes if the buffer disallows copying.
    assert((flags & GL_DYNAMIC_STORAGE_BIT) == GL_DYNAMIC_STORAGE_BIT);

    glNamedBufferSubData(opengl_object, 0, sizeof(PhongLightsMetaData), &header);
    if (!data.empty()) {
        if (data.size() > max_lights_count) {
            std::vector<PhongLightData>::const_iterator first = data.begin();
            std::vector<PhongLightData>::const_iterator last = data.begin() + max_lights_count;
            std::vector<PhongLightData> subset(first, last);
            glNamedBufferSubData(opengl_object, sizeof(PhongLightsMetaData), sizeof(PhongLightData) * subset.size(), subset.data());
        } else {
            glNamedBufferSubData(opengl_object, sizeof(PhongLightsMetaData), sizeof(PhongLightData) * data.size(), data.data());
        }
    }
}
