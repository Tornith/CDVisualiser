// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#include "texture_utils.hpp"
#include "stb_image.h"
#include <iostream>
#include <cmath>
#include <vector>

void TextureUtils::set_texture_2d_parameters(GLuint texture, GLint wrap_s, GLint wrap_t, GLint min_filter, GLint mag_filter) {
    if (texture != 0) {
        glTextureParameteri(texture, GL_TEXTURE_WRAP_S, wrap_s);
        glTextureParameteri(texture, GL_TEXTURE_WRAP_T, wrap_t);
        glTextureParameteri(texture, GL_TEXTURE_MIN_FILTER, min_filter);
        glTextureParameteri(texture, GL_TEXTURE_MAG_FILTER, mag_filter);
    } else {
        // fail silently
    }
}

void TextureUtils::set_texture_3d_parameters(GLuint texture, GLint wrap_s, GLint wrap_t, GLint wrap_r, GLint min_filter, GLint mag_filter) {
    if (texture != 0) {
        glTextureParameteri(texture, GL_TEXTURE_WRAP_S, wrap_s);
        glTextureParameteri(texture, GL_TEXTURE_WRAP_T, wrap_t);
        glTextureParameteri(texture, GL_TEXTURE_WRAP_R, wrap_r);
        glTextureParameteri(texture, GL_TEXTURE_MIN_FILTER, min_filter);
        glTextureParameteri(texture, GL_TEXTURE_MAG_FILTER, mag_filter);
    } else {
        // fail silently
    }
}

GLuint TextureUtils::load_texture_2d(const std::filesystem::path filename, bool flip_y) {
    GLuint texture;
    glCreateTextures(GL_TEXTURE_2D, 1, &texture);

    GLsizei width, height;
    unsigned char* loaded_data = load_image(filename, flip_y, width, height);

    glTextureStorage2D(texture, static_cast<GLsizei>(std::log2(width) + 1), GL_RGBA8, width, height);
    glTextureSubImage2D(texture, 0, 0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, loaded_data);

    // Frees the memory - if flip is true then the memory was allocated with new[].
    if (flip_y) {
        delete[] loaded_data;
    } else {
        stbi_image_free(loaded_data);
    }

    glGenerateTextureMipmap(texture);
    return texture;
}

GLuint TextureUtils::load_texture_cube(const int width, const int height, const std::filesystem::path filename_px,
                                       const std::filesystem::path filename_nx, const std::filesystem::path filename_py,
                                       const std::filesystem::path filename_ny, const std::filesystem::path filename_pz,
                                       const std::filesystem::path filename_nz) {

    const bool flip_y = true;
    GLuint texture;

    //glCreateTextures(GL_TEXTURE_CUBE_MAP_ARRAY, 1, &texture);
    glCreateTextures(GL_TEXTURE_CUBE_MAP, 1, &texture);
    // glTextureStorage3D(texture, static_cast<GLsizei>(std::log2(width + 1)), GL_RGBA8, width, height, 6);
    glTextureStorage2D(texture, static_cast<GLsizei>(std::log2(width + 1)), GL_RGBA8, width, height);

    std::vector<std::filesystem::path> files = {filename_px, filename_nx, filename_py, filename_ny, filename_pz, filename_nz};
    for (int i = 0; i < 6; i++) {
        GLsizei w, h;
        unsigned char* loaded_data = load_image(files[i], flip_y, w, h);

        if (w != width || h != width) {
            std::cout << "The image (" << files[i] << ") does not have required resolution (" << width << "x" << height << ")." <<
                std::endl;
        }
        glTextureSubImage3D(texture, 0, 0, 0, i, width, height, 1, GL_RGBA, GL_UNSIGNED_BYTE, loaded_data);
        // Frees the memory - if flip is true then the memory was allocated with new[].
        if (flip_y) {
            delete[] loaded_data;
        } else {
            stbi_image_free(loaded_data);
        }
    }

    glGenerateTextureMipmap(texture);
    return texture;
}
