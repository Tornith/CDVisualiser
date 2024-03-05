// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
//
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#include "texture.hpp"
#include <iostream>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include "texture_utils.hpp"

// ----------------------------------------------------------------------------
// This file provides the implementation for methods that are common for both
// OpenGL 3.3 and OpenGL 4.5. The other methods are implemented in their
// respective modules.
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Constructors & Destructors
// ----------------------------------------------------------------------------

Texture::Texture() : Texture(0, 0) {}

Texture::Texture(std::filesystem::path path, bool cpu_only) : OpenGLObject(GL_TEXTURE_2D, cpu_only) {
    int realNRChannels;
    unsigned char* data = stbi_load(path.generic_string().data(), &width, &height, &realNRChannels, 4);

    if (width < 1 || height < 1) {
        std::cout << "Could not load texture: " << path << std::endl;
        return;
    }

    nrChannels = 4; // ensures the RGBA
    internal_format = GL_RGBA8;
    format = GL_RGBA;
    type = GL_FLOAT;

    // Creates the CPU representation of the data.
    texture_data = std::vector<float>(width * height * nrChannels);
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            unsigned char r = data[get_index(x, y)];
            unsigned char g = data[get_index(x, y) + 1];
            unsigned char b = data[get_index(x, y) + 2];
            unsigned char a = 255;
            if (nrChannels > 3) {
                a = data[get_index(x, y) + 3];
            }
            set_pixel_color(x, height - y - 1 /* we flip the y axis to have 0,0 at the bottom*/,
                            Color(r / 255.f, g / 255.f, b / 255.f, a / 255.f));
        }
    }

    create_gpu_representation();
}

Texture::Texture(int width, int height, bool cpu_only) : Texture(width, height, GL_RGBA8, GL_RGBA, cpu_only) {}

Texture::Texture(int width, int height, GLint internal_format, GLenum format, bool cpu_only)
    : OpenGLObject(GL_TEXTURE_2D, cpu_only), width(width), height(height), internal_format(internal_format), format(format),
      type(GL_FLOAT) {
    // Creates the CPU representation of the data.
    texture_data = std::vector<float>(width * height * nrChannels);
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            set_pixel_color(x, y, Color::WHITE);
        }
    }

    // Determine the number or channels used by the texture based on the specified format.
    switch (format) {
    case GL_DEPTH_COMPONENT:
        nrChannels = 1;
        break;
    case GL_RGBA:
        nrChannels = 4;
        break;
    default:
        std::cout << "The texture class currently supports only GL_DEPTH_COMPONENT and GL_RGBA formats." << std::endl;
    }

    create_gpu_representation();
}

Texture::Texture(const Texture& other)
    : OpenGLObject(GL_TEXTURE_2D, other.cpu_only), width(other.width), height(other.height), internal_format(other.internal_format),
      format(other.format), type(GL_FLOAT), texture_data(other.texture_data) {

    create_gpu_representation();
}

Texture::Texture(Texture&& other) : Texture(0, 0) { swap_fields(*this, other); }

Texture::~Texture() {
    if (!cpu_only) {
        glDeleteTextures(1, &opengl_object);
    }
}

// ----------------------------------------------------------------------------
// Operators
// ----------------------------------------------------------------------------
Texture& Texture::operator=(Texture other) {
    swap_fields(*this, other);
    return *this;
}

// ----------------------------------------------------------------------------
// Methods
// ----------------------------------------------------------------------------

void Texture::swap_fields(Texture& first, Texture& second) noexcept {
    // Swaps the fields in the base class.
    OpenGLObject::swap_fields(first, second);
    std::swap(first.texture_data, second.texture_data);
    std::swap(first.width, second.width);
    std::swap(first.height, second.height);
    std::swap(first.internal_format, second.internal_format);
    std::swap(first.format, second.format);
    std::swap(first.type, second.type);
    std::swap(first.nrChannels, second.nrChannels);
}

int Texture::get_width() const { return this->width; }

int Texture::get_height() const { return this->height; }

int Texture::get_index(int x, int y) const { return (y * width + x) * nrChannels; }

void Texture::set_pixel_color(int x, int y, const Color& color) {
    if (x < 0 || x >= get_width() || y < 0 || y >= get_height()) {
        std::cout << "The coordinates " << x << ", " << y << " are out of bounds - the color was not modified." << std::endl;
        return;
    }

    const size_t index = this->get_index(x, y);

    if (nrChannels >= 1) {
        texture_data[index] = color.r;
    }
    if (nrChannels >= 2) {
        texture_data[index + 1] = color.g;
    }
    if (nrChannels >= 3) {
        texture_data[index + 2] = color.b;
    }
    if (nrChannels == 4) {
        texture_data[index + 3] = color.a;
    }
}

void Texture::set_pixel_gray_scale(int x, int y, float intensity) {
    if (x < 0 || x >= this->get_width() || y < 0 || y >= this->get_height()) {
        std::cout << "The coordinates " << x << ", " << y << " are out of bounds - the color was not modified." << std::endl;
    }

    const size_t index = this->get_index(x, y);
    if (nrChannels >= 1) {
        texture_data[index] = intensity;
    }
    if (nrChannels >= 2) {
        texture_data[index + 1] = intensity;
    }
    if (nrChannels >= 3) {
        texture_data[index + 2] = intensity;
    }
    if (nrChannels == 4) {
        texture_data[index + 3] = 1;
    }
}

Color Texture::get_pixel_color(int x, int y) const {
    if (x < 0 || x >= this->get_width() || y < 0 || y >= this->get_height()) {
        std::cout << "The coordinates " << x << ", " << y << " are out of bounds - black will be returned." << std::endl;
        return Color::BLACK;
    }

    Color color;
    get_pixel_color_fast(x, y, color.r, color.g, color.b, color.a);
    return color;
}

void Texture::get_pixel_color_fast(int x, int y, float& r, float& g, float& b, float& a) const {
    const size_t index = this->get_index(x, y);

    // TODO consider using format variable instead of nrChannels
    switch (nrChannels) {
    case 1:
        r = texture_data[index];
        g = texture_data[index];
        b = texture_data[index];
        a = 1.0;
        break;
    case 3:
        r = texture_data[index];
        g = texture_data[index + 1];
        b = texture_data[index + 2];
        a = 1.0;
        break;
    case 4:
        r = texture_data[index];
        g = texture_data[index + 1];
        b = texture_data[index + 2];
        a = texture_data[index + 3];
        break;
    default:
        std::cout << "Cannot retrieve the pixel color, the texture has unsupported number of channels." << std::endl;
    }
}

float Texture::get_pixel_gray_scale(int x, int y) const {
    if (x < 0 || x >= this->get_width() || y < 0 || y >= this->get_height()) {
        std::cout << "The coordinates " << x << ", " << y << " are out of bounds - zero will be returned." << std::endl;
        return 0;
    }

    const Color color = this->get_pixel_color(x, y);
    return (color.r + color.g + color.b) / 3.0f;
}

void Texture::set_texture_2d_parameters(GLint wrap_s, GLint wrap_t, GLint min_filter, GLint mag_filter) const {
    TextureUtils::set_texture_2d_parameters(opengl_object, wrap_s, wrap_t, min_filter, mag_filter);
}