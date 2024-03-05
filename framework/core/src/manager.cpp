// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#include "manager.h"
#include "GLFW/glfw3.h"
#include "glad/glad.h"
#include <iostream>
#include <ostream>

float ApplicationManager::requested_opengl_version = 0.0;

// ----------------------------------------------------------------------------
// Methods
// ----------------------------------------------------------------------------


void ApplicationManager::init(int width = 1280, int height = 720, std::string title = "Application", int major = 4, int minor = 5) {
    // Sets up the GLFW error messages.
    glfwSetErrorCallback(&glfw_message_callback);

    ApplicationManager::requested_opengl_version = major + minor/10.f;

    // Initializes GLFW
    if (!glfwInit()) {
        std::cerr << "Could not initialize GLFW!" << std::endl;
        fail = true;
        return;
    }

    // Before creating the window, we request the specified OpenGL core context.
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, major);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, minor);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, true); // this should be ignored on GL < 4.3 (at least I think so)
    glfwWindowHint(GLFW_SAMPLES, samples_per_pixel);

    // Creates the window.
    window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
    if (!window) {
        std::cerr << "Could not create a window!" << std::endl;

        // We failed but lets try to create the best possible context we can so we can print some information about available OpenGL.
        glfwDefaultWindowHints();
        window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
        if (window) {
            glfwMakeContextCurrent(window);
            if (gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
                print_info();
            }
        }

        glfwTerminate();
        fail = true;
        return;
    }

    // Makes window's context current.
    glfwMakeContextCurrent(window);

    // Loads OpenGL functions.
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Could not initialize OpenGL context!" << std::endl;
        fail = true;
        return;
    }

    print_info();

    // Enables the debugging layer of OpenGL if GL version is 4.3 or greater.
    if (major >= 4 && minor >= 3) {
        // GL_DEBUG_OUTPUT - Faster version but not useful for breakpoints
        // GL_DEBUG_OUTPUT_SYNCHRONUS - Callback is in sync with errors, so a breakpoint can be placed on the callback in order to
        // get a stacktrace for the GL error.
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);

        // We do not care about all notifications.
        glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DEBUG_SEVERITY_NOTIFICATION, 0, nullptr, GL_FALSE);

        // Set the function that will be triggered by the callback, the second parameter is the data parameter of the callback.
        // It can be useful for different contexts but isn't necessary for our simple use case.
        glDebugMessageCallback(opengl_message_callback, nullptr);
    }
}

void ApplicationManager::run(IApplication& application) {
    application.set_window(window);

    // Setup the callbacks.
    glfwSetWindowUserPointer(window, &application);
    glfwSetWindowSizeCallback(window, on_resize);
    glfwSetCursorPosCallback(window, on_mouse_move);
    glfwSetMouseButtonCallback(window, on_mouse_button);
    glfwSetKeyCallback(window, on_key_pressed);

    // Runs the extension hook.
    this->pre_render_loop(application);

    while (!glfwWindowShouldClose(window)) {
        // Measures the elapsed time.
        const double current_time = glfwGetTime() * 1000.0; // from seconds to milliseconds
        const double elapsed_time = current_time - last_glfw_time;
        last_glfw_time = current_time;

        // Poll for and process events.
        glfwPollEvents();

        // The preprocess hook.
        this->pre_frame_render();

        // Application render
        application.update(static_cast<float>(elapsed_time));
        application.render();
        application.render_ui();

        // Rendering
        this->post_frame_render();

        // Swap front and back buffers
        glfwSwapBuffers(window);
    }
}

void ApplicationManager::terminate() {
    // Frees allocated resource associated with GLFW.
    glfwTerminate();
}

bool ApplicationManager::is_fail() const { return fail; }

void ApplicationManager::disable_resize() const { glfwSetWindowAttrib(window, GLFW_RESIZABLE, GLFW_FALSE); }

void ApplicationManager::set_multisampling_per_pixel(int samples) { samples_per_pixel = samples; }

void ApplicationManager::pre_render_loop(IApplication& application) {
}

void ApplicationManager::pre_frame_render() {
}

void ApplicationManager::post_frame_render() {
}

void ApplicationManager::print_info() const {
    std::cout << "----------------------------------------------" << std::endl;
    std::cout << "OpenGL: " << glGetString(GL_VERSION) << std::endl;
    std::cout << "Vendor: " << glGetString(GL_VENDOR) << std::endl;
    std::cout << "GLSL: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << std::endl;
    std::cout << "Renderer: " << glGetString(GL_RENDERER) << std::endl;
    std::cout << "----------------------------------------------" << std::endl;
}

void GLAPIENTRY ApplicationManager::opengl_message_callback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length,
                                                            const GLchar* message, const void* userParam) {
    const auto src_str = [source]() {
        switch (source) {
        case GL_DEBUG_SOURCE_API:
            return "API";
        case GL_DEBUG_SOURCE_WINDOW_SYSTEM:
            return "WINDOW SYSTEM";
        case GL_DEBUG_SOURCE_SHADER_COMPILER:
            return "SHADER COMPILER";
        case GL_DEBUG_SOURCE_THIRD_PARTY:
            return "THIRD PARTY";
        case GL_DEBUG_SOURCE_APPLICATION:
            return "APPLICATION";
        case GL_DEBUG_SOURCE_OTHER:
        default:
            return "OTHER";
        }
    }();

    const auto type_str = [type]() {
        switch (type) {
        case GL_DEBUG_TYPE_ERROR:
            return "ERROR";
        case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
            return "DEPRECATED_BEHAVIOR";
        case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:
            return "UNDEFINED_BEHAVIOR";
        case GL_DEBUG_TYPE_PORTABILITY:
            return "PORTABILITY";
        case GL_DEBUG_TYPE_PERFORMANCE:
            return "PERFORMANCE";
        case GL_DEBUG_TYPE_MARKER:
            return "MARKER";
        case GL_DEBUG_TYPE_OTHER:
        default:
            return "OTHER";
        }
    }();

    if (type == GL_DEBUG_TYPE_PERFORMANCE) {
        return;
    }

    const auto severity_str = [severity]() {
        switch (severity) {
        case GL_DEBUG_SEVERITY_NOTIFICATION:
            return "NOTIFICATION";
        case GL_DEBUG_SEVERITY_LOW:
            return "LOW";
        case GL_DEBUG_SEVERITY_MEDIUM:
            return "MEDIUM";
        case GL_DEBUG_SEVERITY_HIGH:
            return "HIGH";
        default:
            return "UNSPECIFIED";
        }
    }();

    std::cerr << src_str << ", " << type_str << ", " << severity_str << ", " << id << ": " << message << '\n';
}

void ApplicationManager::glfw_message_callback(int error_code, const char* description) {
    std::cerr << "Error Code: " << error_code << " Message: " << description << std::endl;
}
