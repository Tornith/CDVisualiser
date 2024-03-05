// ###############################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
//
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#include "texture.hpp"
#include "texture_utils.hpp"
#include <iostream>

// ----------------------------------------------------------------------------
// Methods
// ----------------------------------------------------------------------------

void Texture::bind(GLuint unit) const { glBindTextureUnit(unit, opengl_object); }

void Texture::create_gpu_representation() {
    if (width > 0 && height > 0 && !cpu_only) {
        glCreateTextures(target, 1, &opengl_object);
        glTextureStorage2D(opengl_object, 1, internal_format, width, height);
        update_opengl_data();
    }
}

void Texture::update_opengl_data() {
    if (opengl_object != 0) {
        glTextureSubImage2D(opengl_object, 0, 0, 0, width, height, format, type, texture_data.data());
    } else {
        // fail silently
    }
}

void Texture::update_cpu_data() const {
    if (opengl_object != 0) {
        glGetTextureImage(opengl_object, 0, format, type, width * height * nrChannels * sizeof(float), (void*)&texture_data.data()[0]);
    } else {
        // fail silently
    }
}