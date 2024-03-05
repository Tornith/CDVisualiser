// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#include "ubo.hpp"
#include "camera_ubo.hpp"
#include "light_ubo.hpp"
#include "model_ubo.hpp"
#include "pbr_material_ubo.hpp"
#include "phong_material_ubo.hpp"

template <class T>
void UBO<T>::create_ubo_on_gpu() {
    glCreateBuffers(1, &opengl_object);
    glNamedBufferStorage(opengl_object, sizeof(T) * data.size(), data.empty() ? nullptr : data.data(), flags);
}

template <class T> void UBO<T>::update_opengl_data() {
    // Crashes if the buffer disallows copying.
    assert((flags & GL_DYNAMIC_STORAGE_BIT) == GL_DYNAMIC_STORAGE_BIT);

    if (!data.empty()) {
        glNamedBufferSubData(opengl_object, 0, sizeof(T) * data.size(), data.data());
    }
}