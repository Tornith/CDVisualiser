// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#pragma once
#include <glm/glm.hpp>

/**
 * This is a VERY SIMPLE class that allows to very simply move with the camera. Use left mouse button to change the
 * point of view. Use right mouse button to zoom in and zoom out.
 *
 * @author	<a href="mailto:jan.byska@gmail.com">Jan Byška</a>
 * @author	<a href="mailto:matus.talcik@gmail.com">Matúš Talčík</a>
 */
class Camera {

    // ----------------------------------------------------------------------------
    // Variables
    // ----------------------------------------------------------------------------
protected:
    /** The minimum  elevation in radians. */
    static const float min_elevation;
    /** The maximum camera elevation in radians. */
    static const float max_elevation;
    /** The minimum distance from the point of interest. */
    static const float min_distance;
    /** The sensitivity of the mouse when changing elevation or direction angles. */
    static const float angle_sensitivity;
    /** The sensitivity of the mouse when changing zoom. */
    static const float zoom_sensitivity;
    /**
     * The angle_direction is an angle which determines into which direction in xz plane the camera looks.
     *   - 0 degrees .. we look in -z direction
     *   - 90 degrees .. we look in -x direction
     *   - 180 degrees .. we look in +z direction
     *   - 270 degrees .. we look in +x direction
     */
    float angle_direction;
    /**
     * The angle_elevation is an angle which determines from which "height" the camera looks.
     *   - positive elevation .. we look from above the xz plane
     *   - negative elevation .. we look from below the xz plane
     */
    float angle_elevation;
    /** The distance from (0,0,0), the point at which the camera looks. */
    float distance;
    /** The final position of the eye in world space coordinates, for LookAt or shaders. */
    glm::vec3 eye_position;
    /** The last X and Y coordinates of the mouse cursor. */
    int last_x, last_y;
    /** True or false if moused buttons are pressed and the user rotates/zooms the camera. */
    bool is_rotating, is_zooming;
    /** Flag determining if we have switched axes (TODO: remove after we fix PB009#5 next year). */
    bool switch_axes;

    // ----------------------------------------------------------------------------
    // Constructors
    // ----------------------------------------------------------------------------
public:

    /**
     * Constructs a new {@link Camera} with default location and orientation.
     *
     * @param 	switch_axes	if true the camera will be setup for to work good with UP vector {0,0,1}.
     */
    Camera(bool switch_axes = false);

    // ----------------------------------------------------------------------------
    // Methods
    // ----------------------------------------------------------------------------
private:
    /** Recomputes 'eye_position' from 'angle_direction', 'angle_elevation', and 'distance'. */
    void update_eye_pos();

    // ----------------------------------------------------------------------------
    // Input Events
    // ----------------------------------------------------------------------------
public:
    /**
     * This method is invoked when the mouse cursor is moved.
     *
     * @param 	x	The X-coordinate of the cursor relative to the upper-left corner of the window.
     * @param 	y	The Y-coordinate of the cursor relative to the upper-left corner of the window.
     */
    void on_mouse_move(double x, double y);

    /**
     * This method is invoked when a mouse button is pressed or released.
     *
     * @param 	button	The mouse button that was pressed or released.
     * @param 	action	One of GLFW_PRESS or GLFW_RELEASE.
     * @param 	mods  	Bit field describing which modifier keys were held down.
     */
    void on_mouse_button(int button, int action, int mods);

    // ----------------------------------------------------------------------------
    // Getters & Setters
    // ----------------------------------------------------------------------------
public:
    /**
     * Sets a new position of the eye in world space coordinates.
     *
     * @param 	angle_direction	    The angle which determines into which direction in xz plane the camera looks.
     * @param 	angle_elevation	    The angle_elevation is an angle which determines from which "height" the camera looks.
     * @param 	distance	    The distance from (0,0,0), the point at which the camera looks.
     */
    void set_eye_position(float angle_direction, float angle_elevation, float distance);

    /**
     * Returns the position of the eye in world space coordinates.
     *
     * @return	The eye position.
     */
    glm::vec3 get_eye_position() const;

    /**
     * Computes and returns the view matrix.
     *
     * @return	The view matrix.
     */
    glm::mat4x4 get_view_matrix() const;
};
