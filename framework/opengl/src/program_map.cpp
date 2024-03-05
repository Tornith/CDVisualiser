// ###############################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#include "program_map.hpp"

void ShaderProgramMap::add(int key, ShaderProgram* program) {
    if (!program) {
        remove(key);
    } else {
        shader_map[key] = program;
    }
}

void ShaderProgramMap::remove(int key) {
    shader_map.erase(key);
}

void ShaderProgramMap::remove_all() {
    shader_map.clear();
}

ShaderProgram* ShaderProgramMap::get_program(int key) {
    const auto iter = shader_map.find(key);
    return (iter == shader_map.end()) ? nullptr : iter->second;
}

bool ShaderProgramMap::use(int key) {
    ShaderProgram* program = get_program(key);
    if (!program) {
        return false;
    }
    program->use();
    return true;
}
