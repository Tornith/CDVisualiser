// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#pragma once
#include "manager.h"

/**
 * An extension for ImGuiManager TODO
 *
 * @author <a href="mailto:jan.byska@gmail.com">Jan Byška</a>
 */
class ImGuiManager : public ApplicationManager {

    // ----------------------------------------------------------------------------
    // Methods
    // ----------------------------------------------------------------------------
public:
    /** @copydoc ImGuiManager::init */
    void init(int width, int height, std::string title, int major, int minor) override;

protected:
    /** @copydoc ImGuiManager::init_render_loop */
    void pre_render_loop(IApplication& application) override;

    /** @copydoc ImGuiManager::init_frame_render */
    void pre_frame_render() override;

    /** @copydoc ImGuiManager::post_frame_render */
    void post_frame_render() override;
};
