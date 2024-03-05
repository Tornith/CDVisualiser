// ###############################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#pragma once
#include <glad/glad.h>
#include <string> // TODO use <cstring> instead of string as workaround for unix-based systems?
#include <iostream>

/**
 * The class providing a collection of static utility methods for working with frame buffers.
 *
 * @author	<a href="mailto:jan.byska@gmail.com">Jan Byška</a>
 * @author	<a href="mailto:cejka.honza@gmail.com ">Jan Čejka</a>
 */
class FBOUtils {

public:
    /**
     * Checks the status of a currently bind framebuffer and prints an error if the framebuffer is not complete.
     *
     * @param 	framebuffer	The framebuffer to check.
     * @param 	buffer_name	The name of the framebuffer that will be used in the error message.
     *
     * @return	True if it succeeds, false if it fails.
     */
    static bool check_framebuffer_status(GLuint framebuffer, std::string buffer_name) {
        GLenum status = glCheckNamedFramebufferStatus(framebuffer, GL_FRAMEBUFFER);
        if (status != GL_FRAMEBUFFER_COMPLETE) {
            std::cout << "Failed to create a complete framebuffer (" << buffer_name << ")." << std::endl;
            return false;
        }
        return true;
    }

    /**
     * When setting a framebuffer, we need to "connect" fragment output data with color attachments. We do this by
     * calling glDrawBuffers with {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, ...} as the second parameter. The
     * following constant just saves repeating the definition of the array again and again.
     */
    static const GLenum draw_buffers_constants[];
};