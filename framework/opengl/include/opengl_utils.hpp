#pragma once
#include <glad/glad.h>
#include <sstream>
#include <cmath>
#include <cstring>
#include "manager.h"

/**
 * The class providing a collection of static utility methods.
 *
 * @author	<a href="mailto:jan.byska@gmail.com">Jan By�ka</a>
 */
class OpenGLUtils {
public:
    /**
     * Checks if a specified OpenGL extension exists.
     *
     * @param 	extension	The extension to test.
     *
     * @return	@p true if OpenGL exists, @p false if not.
     */
    static bool is_open_gl_extension_present(const char* extension) {
        // Gets the number of extensions.
        GLint num_extension;
        glGetIntegerv(GL_NUM_EXTENSIONS, &num_extension);

        // Gos through all extensions and check whether the given extension is present.
        for (int i = 0; i < num_extension; i++) {
            if (strcmp(reinterpret_cast<const char*>((glGetStringi(GL_EXTENSIONS, i))), extension) == 0) {
                // We found the extension.
                return true;
            }
        }
        // We did not found the extension.
        return false;
    }

    static float get_opengl_version() {
        //int major, minor;
        //glGetIntegerv(GL_MAJOR_VERSION, &major);
        //glGetIntegerv(GL_MINOR_VERSION, &minor);
        // return major + minor/10.f;
        // The above does not work on Fedora as it returns the maximum available version.
        return ApplicationManager::requested_opengl_version;
    }
};

// Backward compatibility as Utils was replaced by more generic OpenGLUtils class.
typedef OpenGLUtils Utils;