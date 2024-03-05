// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#pragma once

#include "advanced_scene_object.hpp"
#include "opengl_application.hpp"
#include "texture.hpp"

class PA213Application : public OpenGLApplication {

    // ----------------------------------------------------------------------------
    // Variables
    // ----------------------------------------------------------------------------
protected:
    /** The list of scene objects. */
    std::vector<std::shared_ptr<AdvancedSceneObject>> scene_objects;

    /** The shader program used to display textures. */
    ShaderProgram display_texture_program;

    // ----------------------------------------------------------------------------
    // Default objects
    // ----------------------------------------------------------------------------
protected:
    /** Empty VAO. */
    GLuint base_vao;

    // ----------------------------------------------------------------------------
    // Constructors
    // ----------------------------------------------------------------------------
public:
    /**
     * Constructs a new @link PA213Application with a custom width and height.
     * @param 	initial_width 	The initial width of the window.
     * @param 	initial_height	The initial height of the window.
     * @param 	arguments	  	The command line arguments used to obtain the application directory.
     */
    PA213Application(int initial_width, int initial_height, std::vector<std::string> arguments);

    /** The destructor that releases all resources. */
    virtual ~PA213Application();

    // ----------------------------------------------------------------------------
    // Methods
    // ----------------------------------------------------------------------------

    /**
     * Activates a custom frame buffer. The methods also automatically sets the full screen size viewport and possibly
     * also clears the buffer and sets the depth test.
     *
     * @param 	buffer	  	The buffer to activate (0 is the main screen).
     * @param 	clear	  	If @p true the bounded frame buffer will be cleared.
     * @param 	depth_test	if @p true the depth test will be activated; if @p false the depth test will be deactivated.
     */
    void activate_render_buffer(GLint buffer = 0, bool clear = true, bool depth_test = true) const;

    /**
     * Activates a custom-sized viewport.
     *
     * @param 	x	   	The X-coordination of the lower left corner of the viewport rectangle (0 by default).
     * @param 	y	   	The Y-coordination of the lower left corner of the viewport rectangle (0 by default).
     * @param 	width	The width of the viewport (value<=0 is used to indicate full screen).
     * @param 	height	The height of the viewport (value<=0 is used to indicate full screen).
     */
    void activate_viewport(GLint x = 0, GLint y = 0, GLsizei width = 0, GLsizei height = 0) const;

    /**
     * Activates a viewport such that it is positioned within a grid.
     *
     * @param 	x	  	The grid x coordinate (position of the requested viewport in the grid).
     * @param 	y	  	The grid y coordinate (position of the requested viewport in the grid).
     * @param 	grid_size_x 	The grid maximum grid size (the assumed number of viewports along x axis).
     * @param 	grid_size_y	The grid maximum grid size (the assumed  number of viewports along y axis).
     */
    void activate_viewport_grid(int x, int y, int grid_size_x, int grid_size_y) const;

    /**
     * Prepares the OpenGL context for rendering. A convenience method for calling @link activate_render_buffer and
     * @link activate_viewport.
     *
     * @param 	x		  	The X-coordination of the lower left corner of the viewport rectangle (0 by default).
     * @param 	y		  	The Y-coordination of the lower left corner of the viewport rectangle (0 by default).
     * @param 	width	  	The width of the viewport (value<=0 is used to indicate full screen).
     * @param 	height	  	The height of the viewport (value<=0 is used to indicate full screen).
     * @param 	buffer	  	The buffer to activate (0 is the main screen).
     * @param 	clear	  	If @p true the bounded frame buffer will be cleared.
     * @param 	depth_test	if @p true the depth test will be activated; if @p false the depth test will be deactivated.
     */
    void prepare_rendering(int x = 0, int y = 0, int width = 0, int height = 0, GLint buffer = 0, bool clear = true, bool depth_test = true) const;

    /**
     * Prepares the OpenGL context for rendering withing a grid of viewports. A convenience method for calling @link
     * activate_render_buffer and
     * @link activate_viewport.
     *
     * @param 	x		   	The grid x coordinate (position of the requested viewport in the grid).
     * @param 	y		   	The grid y coordinate (position of the requested viewport in the grid).
     * @param 	grid_size_x	The grid maximum grid size (the assumed number of viewports along x axis).
     * @param 	grid_size_y	The grid maximum grid size (the assumed  number of viewports along y axis).
     * @param 	buffer	   	The buffer to activate (0 is the main screen).
     * @param 	clear	   	If @p true the bounded frame buffer will be cleared.
     * @param 	depth_test 	if @p true the depth test will be activated; if @p false the depth test will be deactivated.
     */
    void prepare_rendering_grid(int x, int y, int grid_size_x, int grid_size_y, GLint buffer = 0, bool clear = false, bool depth_test = true) const;

    /**
     * Uploads a projection and view matrices to a specified program. Note that the method also binds the program.
     *
     * @param 	program   	The program.
     * @param 	projection	The projection matrix.
     * @param 	view	  	The view matrix.
     */
    void upload_projection_view(const ShaderProgram& program, glm::mat4 projection, glm::mat4 view) const;

    /**
     * Displays the given texture.
     */
    void display_texture(const Texture& texture, bool depth_texture);
};
