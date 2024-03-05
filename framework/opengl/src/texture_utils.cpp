// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#include "texture_utils.hpp"
#include "stb_image.h"

// ----------------------------------------------------------------------------
// This file provides the implementation for methods that are common for both
// OpenGL 3.3 and OpenGL 4.5. The other methods are implemented in their
// respective modules.
// ----------------------------------------------------------------------------

unsigned char* TextureUtils::load_image(const std::filesystem::path filename, bool flip_y, int& width, int& height) {
    int channels, required_channels = 4;
    unsigned char* loaded_data = stbi_load(filename.generic_string().data(), &width, &height, &channels, required_channels);

    // Flips the loaded pixels along the Y-axis if requested.
    if (flip_y && loaded_data != nullptr) {
        auto* flipped_data = new unsigned char[width * height * required_channels];
        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                const int index = (y * width + x) * required_channels;
                const int flipped_y = height - y - 1;
                const int flipped_index = (flipped_y * width + x) * required_channels;
                flipped_data[flipped_index] = loaded_data[index];
                flipped_data[flipped_index + 1] = loaded_data[index + 1];
                flipped_data[flipped_index + 2] = loaded_data[index + 2];
                if (channels > 3) {
                    flipped_data[flipped_index + 3] = loaded_data[index + 3];
                } else {
                    flipped_data[flipped_index + 3] = 255;
                }
            }
        }
        stbi_image_free(loaded_data); // release the other data
        loaded_data = flipped_data;
    }
    return loaded_data;
}
