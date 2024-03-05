// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
//
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#pragma once

#define GLFW_INCLUDE_NONE
#include "GLFW/glfw3.h"
#include "iapplication.h"
#include <string>

/**
 * This is a factory class that allows us to easily initialize the OpenGL context.
 *
 * @author <a href="mailto:jan.byska@gmail.com">Jan Byška</a>
 */
class ApplicationManager {

    // ----------------------------------------------------------------------------
    // Variables
    // ----------------------------------------------------------------------------
public:
  static float requested_opengl_version;

  protected:
    /** The created OpenGL window. */
    GLFWwindow* window = nullptr;

    /** The last measured time step (in milliseconds). The value is used to determine time elapsed between two frames. */
    double last_glfw_time = 0;

    /** The flag determining if the creation of the OpenGL window failed. */
    bool fail = false;

    /** The number of sampling points per fragment, to be used with GLFW_SAMPLES. */
    int samples_per_pixel = 1;

    // ----------------------------------------------------------------------------
    // Methods
    // ----------------------------------------------------------------------------
  public:
    /**
     * Initializes the OpenGL window based on the specified values.
     *
     * @param 	width 	The requested window width.
     * @param 	height	The requested windows height.
     * @param 	title 	The requested window title.
     * @param 	major 	The requested major version of the OpenGL core context.
     * @param 	minor 	The requested minor version of the OpenGL core context.
     */
    virtual void init(int width, int height, std::string title, int major, int minor);

    /** This method invokes an infinite loop and renders the provided application every frame. */
    virtual void run(IApplication& application);

    /** Terminates the GLFW and free the allocated resource. */
    void terminate();

     /** Checks if the initialization of the OpenGL context failed. */
    bool is_fail() const;

    /** Makes the windows not resizable. */
    void disable_resize() const;

    /**
     * Sets how many sampling points are to be used when multisampling is used.
     *
     * @param 	samples	The number of sampling points per pixel.
     */
    void set_multisampling_per_pixel(int samples);

  protected:
    /** This method is invoked right before the infinite render loop is executed. */
    virtual void pre_render_loop(IApplication& application);

    /** This method is invoked every frame before calling the render method in application. */
    virtual void pre_frame_render();

    /** This method is invoked every frame after calling the render method in application. */
    virtual void post_frame_render();

    /** Prints some basic information about the current HW and loaded OpenGL context. */
    void print_info() const;

    /**
     * The OpenGL debug message callback.
     *
     * @param 	source	  	The messages source.
     * @param 	type	  	The type of the error.
     * @param 	id		  	The identifier.
     * @param 	severity  	The severity.
     * @param 	length	  	The length.
     * @param 	message   	The message.
     * @param 	user_param	The user parameter.
     */
    static void GLAPIENTRY opengl_message_callback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length,
                                                   const GLchar* message, const void* user_param);

    /**
     * The GLFW debug message callback.
     *
     * @param 	error_code 	The error code.
     * @param 	description	The error description.
     */
    static void glfw_message_callback(int error_code, const char* description);

    // ----------------------------------------------------------------------------
    // Input Events
    // ----------------------------------------------------------------------------
    /**
     * This method is invoked when the window size changes. The method attempts to cast the specified window to an
     * instance of application and calls the corresponding method.
     *
     * @param 	window	The window that received the event.
     * @param 	width 	The new width.
     * @param 	height	The new height.
     */
    static void on_resize(GLFWwindow* window, int width, int height) {
        auto* application = static_cast<IApplication*>(glfwGetWindowUserPointer(window));
        application->on_resize(width, height);
    }

    /**
     * This method is invoked when the mouse cursor is moved. The method attempts to cast the specified window to an
     * instance of this application and calls the corresponding method.
     *
     * @param 	window	The window that received the event.
     * @param 	x	The X-coordinate of the cursor relative to the upper-left corner of the window.
     * @param 	y	The Y-coordinate of the cursor relative to the upper-left corner of the window.
     */
    static void on_mouse_move(GLFWwindow* window, double x, double y) {
        auto* application = static_cast<IApplication*>(glfwGetWindowUserPointer(window));
        application->on_mouse_move(x, y);
    }

    /**
     * This method is invoked when a mouse button is pressed or released. The method attempts to cast the specified
     * window to an instance of this application and calls the corresponding method.
     *
     * @param 	window	The window that received the event.
     * @param 	button	The mouse button that was pressed or released.
     * @param 	action	One of GLFW_PRESS or GLFW_RELEASE.
     * @param 	mods  	Bit field describing which modifier keys were held down.
     */
    static void on_mouse_button(GLFWwindow* window, int button, int action, int mods) {
        auto* application = static_cast<IApplication*>(glfwGetWindowUserPointer(window));
        application->on_mouse_button(button, action, mods);
    }

    /**
     * This method is invoked when a keyboard key is pressed, released, or repeated.
     *
     * @param 	window  	The window that received the event.
     * @param 	key		The keyboard key that was pressed or released.
     * @param 	scancode	The system-specific scancode of the key.
     * @param 	action  	One of GLFW_PRESS, GLFW_RELEASE or GLFW_REPEAT.
     * @param 	mods		Bit field describing which modifier keys were held down.
     */
    static void on_key_pressed(GLFWwindow* window, int key, int scancode, int action, int mods) {
        auto* application = static_cast<IApplication*>(glfwGetWindowUserPointer(window));
        application->on_key_pressed(key, scancode, action, mods);
    }
};
