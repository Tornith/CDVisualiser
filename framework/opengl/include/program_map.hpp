// ###############################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#include "program.hpp"
#include <map>

class ShaderProgramMap {

    // ----------------------------------------------------------------------------
    // Variables
    // ----------------------------------------------------------------------------
protected:
    std::map<int, ShaderProgram*> shader_map;

    // ----------------------------------------------------------------------------
    // Methods
    // ----------------------------------------------------------------------------
public:
    /**
     * Adds a shader program into the map.
     *
     * @param 	key	   	The key.
     * @param 	program	The program.
     */
    void add(int key, ShaderProgram* program);

    /**
     * Removes a shader program from the map.
     *
     * @param 	key	The key.
     */
    void remove(int key);

    /** Removes all shader programs from the map */
    void remove_all();

    /**
     * Returns a shader program at given key or nullptr if no such program exists.
     *
     * @param 	key	The key.
     * @return	The program associated with the key, or @p null if the program does not exist.
     */
    ShaderProgram* get_program(int key);

    /**
     * Tries to use the shader program. If the shader program exists and is valid, it is used and the function returns
     * @p true. If the shader program is not present or it is not valid, it is not used and @p false is returned.
     *
     * @param 	key	The key.
     * @return	True if it succeeds, false if it fails.
     */
    bool use(int key);
};
