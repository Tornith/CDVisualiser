#pragma once

#include "glad/glad.h"
#include <filesystem>

/**
 * The class providing a collection of static utility methods for working with textures.
 *
 * @author	<a href="mailto:jan.byska@gmail.com">Jan Byška</a>
 * @author	<a href="mailto:cejka.honza@gmail.com ">Jan Čejka</a>
 */
class TextureUtils {

public:
    /**
     * Sets the texture parameters by calling glTextureParameteri.
     *
     * @param 	texture   	The OpenGL texture name.
     * @param 	wrap_s	  	The wrap parameter value for GL_TEXTURE_WRAP_S.
     * @param 	wrap_t	  	The wrap parameter value for GL_TEXTURE_WRAP_T.
     * @param 	min_filter	A filter parameter value for GL_TEXTURE_MIN_FILTER.
     * @param 	mag_filter	A filter parameter value for GL_TEXTURE_MAG_FILTER.
     */
    static void set_texture_2d_parameters(GLuint texture, GLint wrap_s, GLint wrap_t, GLint min_filter, GLint mag_filter);

    /**
     * Sets the texture parameters by calling glTextureParameteri.
     *
     * @param 	texture   	The OpenGL texture name.
     * @param 	wrap_s	  	The wrap parameter value for GL_TEXTURE_WRAP_S.
     * @param 	wrap_t	  	The wrap parameter value for GL_TEXTURE_WRAP_T.
     * @param 	wrap_t	  	The wrap parameter value for GL_TEXTURE_WRAP_R.
     * @param 	min_filter	A filter parameter value for GL_TEXTURE_MIN_FILTER.
     * @param 	mag_filter	A filter parameter value for GL_TEXTURE_MAG_FILTER.
     */
    static void set_texture_3d_parameters(GLuint texture, GLint wrap_s, GLint wrap_t, GLint wrap_r, GLint min_filter, GLint mag_filter);

    /**
     * Loads an image from the given path. As sometimes the OpenGL texture coordinate do not correspond
     * to the image coordinates, the method allows to flip the image along the Y axis.
     *
     * @param 	filename	The filename of an image to load.
     * @param flip_y        If @p true the texture will be flipped along the Y-axis.
     * @param width         The output variable where the width of the loaded image will be stored.
     * @param height        The output variable where the height of the loaded image will be stored.
     */
    static unsigned char* load_image(std::filesystem::path filename, bool flip_y, int& width, int& height);

    /**
     * Loads an image and creates OpenGL texture from it.
     *
     * @param 	filename	The filename of an image to load.
     * @param 	flip_y  	If @p true the texture will be flipped along the Y-axis (by default should be true for OpenGL).
     *
     * @return	The created OpenGL texture.
     */
    static GLuint load_texture_2d(std::filesystem::path filename, bool flip_y = true);

    /**
     * Loads and 6 images and creates OpenGL cube texture.
     * @param 	width	   	the expected width of the loaded cube map texture.
     * @param 	height	   	the expected height of the loaded cube map texture.
     * @param 	filename_px	The filename of an image to load.
     * @param 	filename_nx	The filename of an image to load.
     * @param 	filename_py	The filename of an image to load.
     * @param 	filename_ny	The filename of an image to load.
     * @param 	filename_pz	The filename of an image to load.
     * @param 	filename_nz	The filename of an image to load.
     * @return	The created OpenGL texture.
     */
    static GLuint load_texture_cube(int width, int height, std::filesystem::path filename_px, std::filesystem::path filename_nx,
                                    std::filesystem::path filename_py, std::filesystem::path filename_ny, std::filesystem::path filename_pz,
                                    std::filesystem::path filename_nz);
};