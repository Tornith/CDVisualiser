// ###############################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#pragma once

#include "color.hpp"
#include "opengl_object.hpp"
#include <filesystem>
#include <vector>

/**
 * The base class for representing textures.
 *
 * @author	<a href="mailto:jan.byska@gmail.com">Jan Byška</a>
 */
class Texture : public OpenGLObject {
    // ----------------------------------------------------------------------------
    // Variables
    // ----------------------------------------------------------------------------
protected:
    /** The texture width. */
    int width;

    /** The texture height. */
    int height;

    /** The sized internal format used to store texture image data. */
    GLenum internal_format;

    /** The format of the pixel data. */
    GLenum format;

    /** The type of the pixel data. Current implementation supports only GL_FLOAT. */
    GLenum type;

    /** The number of color channels. */
    int nrChannels = 4;

    /**
     * The array of pixels - the array is organized as list of rows with 'nrChannels' values per pixel. See
     * {@link get_index} method for more details.
     */
    std::vector<float> texture_data;

    // ----------------------------------------------------------------------------
    // Constructors
    // ----------------------------------------------------------------------------
public:
    /** Constructs a new @link Texture. */
    Texture();

    /**
     * Constructs a new @link Texture from a specified file.
     *
     * @param 	path		The path for the file with image.
     * @param 	cpu_only	The flag determining if the texture will be CPU only (should be @p true for tests as they do
     * 						not have OpenGL context).
     */
    Texture(std::filesystem::path path, bool cpu_only = false);

    /**
     * Constructs a new @link Texture with specified size and default formats.
     *
     * @param 	width   	The texture width.
     * @param 	height  	The texture height.
     * @param 	cpu_only	The flag determining if the texture will be CPU only (should be @p true for tests as they do
     * 						not have OpenGL context).
     */
    Texture(int width, int height, bool cpu_only = false);

    /**
     * Constructs a new custom @link Texture with specified size and formats.
     *
     * @param 	width		   	The texture width.
     * @param 	height		   	The texture height.
     * @param 	internal_format	The internal texture format that specifies the number of color components in the texture.
     * @param 	format		   	The texture format that specifies the format of the pixel data.
     * @param 	cpu_only	   	The flag determining if the texture will be CPU only (should be @p true for tests as they
     * 							do not have OpenGL context).
     */
    Texture(int width, int height, GLint internal_format, GLenum format, bool cpu_only = false);

    /**
     * Constructs a new @link Texture from another object (copy constructor that performs a deep copy).
     *
     * @param 	other	The other texture to copy from.
     */
    Texture(const Texture& other);

    /**
    * Move constructor. Moves the Texture to the new object. Zeroes out the old object so that
    * OpenGL's destructor does nothing.
    *
    * @param 	other The other texture.
    */
    Texture(Texture&& other);

    /** Destroys this @link Texture. */
    virtual ~Texture();

    // ----------------------------------------------------------------------------
    // Operators
    // ----------------------------------------------------------------------------
    /**
    * The copy assignment using copy-and-swap idiom.
    *
    * @param other The other texture that will be copied (i.e., swapped into this).
    * 				
    * @return A shallow copy of the other texture that was moved into this object.
    */
    Texture& operator=(Texture other);

    // ----------------------------------------------------------------------------
    // Methods
    // ----------------------------------------------------------------------------

    /**
     * The custom swap method that exchanges the values of fields of two textures.
     *
     * @param 	first 	The first texture.
     * @param 	second	The second texture.
     */
    void swap_fields(Texture& first, Texture& second) noexcept;

    /**
     * Bind the texture to the specified texture unit.
     *
     * @param 	unit	The texture unit to which the texture object will be bound to.
     */
    virtual void bind(GLuint unit) const;

    /** @copydoc OpenGLObject::update_opengl_data */
    void update_opengl_data() override;

    /** Copies the data from GPU to CPU. */
    void update_cpu_data() const;

    /**
     * Sets the texture parameters by calling glTextureParameteri.
     *
     * @param 	wrap_s	  	The wrap parameter value for GL_TEXTURE_WRAP_S.
     * @param 	wrap_t	  	The wrap parameter value for GL_TEXTURE_WRAP_T.
     * @param 	min_filter	A filter parameter value for GL_TEXTURE_MIN_FILTER.
     * @param 	mag_filter	A filter parameter value for GL_TEXTURE_MAG_FILTER.
     */
    virtual void set_texture_2d_parameters(GLint wrap_s, GLint wrap_t, GLint min_filter, GLint mag_filter) const;

private:
    /**
     * Returns the first index (i.e., red channel) of the pixel at x,y in the data array. Use: getIndex(x,y) to get
     * index for red value of pixel at x,y getIndex(x,y) + 1 to get index for green value of pixel at x,y getIndex(x,y)
     * + 2 to get index for blue value of pixel at x,y
     *
     * @param 	x	The x coordinate in the texture (zero-based index).
     * @param 	y	The y coordinate in the texture (zero-based index).
     *
     * @return	The corresponding index in the data array.
     */
    int get_index(int x, int y) const;

    /** Creates the GPU representation of the texture. */
    void create_gpu_representation(); 

    // ----------------------------------------------------------------------------
    // Getters & Setters
    // ----------------------------------------------------------------------------
public:
    /**
     * Returns the texture width.
     *
     * @return The texture width.
     */
    int get_width() const;

    /**
     * Returns the texture height.
     *
     * @return The texture height.
     */
    int get_height() const;

    /**
     * Sets the color of a pixel identified by the x (column) and y (row) coordinates.
     *
     * @param 	x	 	The X-coordinate of the pixel to be changed (zero-based index).
     * @param 	y	 	The Y-coordinate of the pixel to be changed (zero-based index).
     * @param 	color	The new color.
     */
    void set_pixel_color(int x, int y, const Color& color);

    /**
     * Sets the intensity (i.e., monotone color) of a pixel identified by the x (column) and y (row)
     * coordinates.
     *
     * @param 	x		 	The X-coordinate of the pixel to be changed (zero-based index).
     * @param 	y		 	The Y-coordinate of the pixel to be changed (zero-based index).
     * @param 	intensity	The intensity.
     */
    void set_pixel_gray_scale(int x, int y, float intensity);

    /**
     * Returns the current color of the raster pixel identified by the x (column) and y (row)
     * coordinates.
     *
     * @param 	x	The X-coordinate of the requested pixel (zero-based index).
     * @param 	y	The Y-coordinate of the requested pixel (zero-based index).
     *
     * @return	The pixel color.
     */
    Color get_pixel_color(int x, int y) const;

    /**
     * Returns the current color of the raster pixel identified by the x (column) and y (row)
     * coordinates. The value is returned via return parameters wich is faster then creating a color object.
     *
     * @param 	x	The x coordinate (zero-based index).
     * @param 	y	The y coordinate (zero-based index).
     * @param 	r	The return parameter for red channel.
     * @param 	g	The return parameter for green channel.
     * @param 	b	The return parameter for blue channel.
     * @param 	a	The return parameter for alpha channel.
     */
    void get_pixel_color_fast(int x, int y, float& r, float& g, float& b, float& a) const;

    /**
     * Returns the current intensity (i.e., monotone color) of the raster pixel identified by the x (column) and y (row)
     * coordinates.
     *
     * @param 	x	The X-coordinate of the requested pixel (zero-based index).
     * @param 	y	The Y-coordinate of the requested pixel (zero-based index).
     *
     * @return	The pixel gray scale.
     */
    float get_pixel_gray_scale(int x, int y) const;
};

// Backward compatibility as Texture_3_3 was replaced by more generic Texture class.
typedef Texture Texture_3_3;