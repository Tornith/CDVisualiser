// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#include "configuration.h"

// ----------------------------------------------------------------------------
// Constructors & Destructors
// ----------------------------------------------------------------------------

Configuration::Configuration(const std::string& application_directory)
    : runtime_directory(std::filesystem::path{application_directory}) {
    runtime_directory.remove_filename();
    std::filesystem::path path{runtime_directory / "configuration.toml"};
    path.make_preferred();

    if (exists(path)) {
        configuration = toml::parse(path.generic_string());
    } else {
        std::cerr << "Failed to load configuration file from: " << path << std::endl;
    }
}
