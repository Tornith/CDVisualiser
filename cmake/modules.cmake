################################################################################
# Common Framework for Computer Graphics Courses at FI MUNI.
#
# Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
# All rights reserved.
#
# The CMake file defining macros and functions for compiling individual 
# framework modules.
################################################################################

#--------------------------------------------------------------------
# Creates a new module.
#--------------------------------------------------------------------
# @param retval       - The variable where the internal module name 
#                       will be stored.
# @param NAME         - The module name (optional).
# @param HEADERS      - The list of header files (optional).
# @param SOURCES      - The list of source files (optional).
# @param INCLUDE_DIRS - The list of include directories (not implemented yet).
#--------------------------------------------------------------------
function(visitlab_create_module retval)
    # Processes the parameters.
    set(options "")
    set(oneValueArgs NAME)
    set(multiValueArgs HEADERS SOURCES INCLUDE_DIRS)
    cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
   
    # Generates the module name.
    if("${ARG_NAME}" STREQUAL "")
        visitlab_generate_module_name(module_name)
    else()
        set(module_name ${ARG_NAME})
    endif()
    visitlab_get_module_path(module_path ${module_name})
    visitlab_to_internal_module_name(module_target ${module_name})

    # Returns the target name for the module.
    set(${retval} ${module_target} PARENT_SCOPE)
      
    # Adds a new library for the module.
    message(STATUS "Target: ${module_target}.dll")
    if("${ARG_HEADERS}" STREQUAL "" AND "${ARG_SOURCES}" STREQUAL "" AND "${ARG_PLATFORM_SOURCES}" STREQUAL "")
        message(STATUS "No files to build.")
        add_library(${module_target} STATIC)
    else()
        message(STATUS "Headers: ${ARG_HEADERS}")
        message(STATUS "Sources: ${ARG_SOURCES}")
       
        add_library(${module_target} STATIC ${ARG_HEADERS} ${ARG_SOURCES})
    endif()

    # Sets the build properties.
    # CXX_STANDARD - the C++ standard whose features are requested.
    # CXX_EXTENSIONS - no compiler specific extensions are requested.
    set_target_properties(${module_target} PROPERTIES 
        CXX_STANDARD 20             
        CXX_EXTENSIONS OFF
    )

    # Solves the module dependencies.
    visitlab_get_module_dependencies(dependencies ${module_name}) #TODO: reuse the already identified dependencies in application
    message(STATUS "Dependencies: ${dependencies}")

    # Links the required modules.
    target_link_libraries(${module_target} PUBLIC ${dependencies})

    # Sets the include directories.
    target_include_directories(${module_target} PUBLIC "${module_path}/include")
endfunction()

#--------------------------------------------------------------------
# Generates a module name based on the current CMake list directory.
#--------------------------------------------------------------------
# @param retval - The variable where the new name will be stored.
#--------------------------------------------------------------------
function(visitlab_generate_module_name retval)
    get_filename_component(module_name "${CMAKE_CURRENT_LIST_DIR}" NAME)
    visitlab_to_internal_module_name(module_internal_name ${module_name})
    message(STATUS "Generating a new module name: '${module_internal_name}'")
    
    set(${retval} ${module_internal_name} PARENT_SCOPE)
endfunction()

#--------------------------------------------------------------------
# Includes 'extra.cmake' file with module specific build instructions.
# The method sets a global variable ${visitlab_current_target} that 
# can be used in the extra.cmake file.
#--------------------------------------------------------------------
# @param target_name - The current target beeing build.
#--------------------------------------------------------------------
function(visitlab_add_module_specific_cmake target_name)
    set(visitlab_current_target ${target_name} CACHE INTERNAL "Current Target")
    foreach(module ${ARGN})
        visitlab_get_module_path(module_path ${module})
        set(extra_cmake_file "${module_path}/cmake/extra.cmake")
        if(EXISTS "${extra_cmake_file}")
            message(STATUS "The 'extra.cmake' file found in '${module}', including it into the build sript for '${app_name}'.")
            include("${extra_cmake_file}")
        endif()  
    endforeach()
endfunction()

#--------------------------------------------------------------------
# Activates the specified modules.
#--------------------------------------------------------------------
# @param ARGN - The method accepts variable number of modules.
#--------------------------------------------------------------------
function(visitlab_add_modules)
    foreach(module_name ${ARGN})
        message("----------------------------------------------------------------------")
        message("Adding '${module_name}' module into the build.")
        message("----------------------------------------------------------------------")
        visitlab_get_module_path(module_path ${module_name})
        add_subdirectory("${module_path}")
     endforeach()
endfunction()

#--------------------------------------------------------------------
# Resolves recursively dependencies for the specified modules.
#--------------------------------------------------------------------
# @param retval - The variable where the retrieved dependencies will 
#                 be stored. 
# @param ARGN   - The method accepts variable number of modules.
# @param KEEP   - The flag used internally to keep or remove 
#                 procesed modules during the recursive calls.
#--------------------------------------------------------------------
function(visitlab_get_module_dependencies retval)
    # Processes the parameters.
    set(options KEEP)
    set(one_value_args "")
    set(multi_value_args "")
    cmake_parse_arguments(ARG "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})    

    if(NOT ARG_KEEP)
        message(STATUS "Cleaning the cached dependencies.")
        set(visitlab_solved_mod_deps "" CACHE INTERNAL "Solved Dependencies")
    endif()
    
    foreach(module_name ${ARG_UNPARSED_ARGUMENTS})
        if(NOT ${module_name} IN_LIST visitlab_solved_mod_deps)
            list(APPEND visitlab_solved_mod_deps ${module_name})
            set(visitlab_solved_mod_deps ${visitlab_solved_mod_deps} CACHE INTERNAL "Solved Module Dependencies")

            visitlab_get_module_path(module_path ${module_name})  
            visitlab_get_dependencies(dependencies ${module_path})

            # Solving the dependencies recursively.
            visitlab_get_module_dependencies(indirect_dependencies KEEP ${dependencies})

            list(APPEND all_dependencies_list ${dependencies})
            list(APPEND all_dependencies_list ${indirect_dependencies})
        endif()
    endforeach()
    list(REMOVE_DUPLICATES all_dependencies_list)
    set(${retval} ${all_dependencies_list} PARENT_SCOPE)
endfunction()

#--------------------------------------------------------------------
# Gets the absolute path to the specified module.
#--------------------------------------------------------------------
# @param retval      - The variable where the path will be stored.
# @param module_name - The name of a module for which the path is 
#                      requested. 
#--------------------------------------------------------------------
function(visitlab_get_module_path retval module_name)
    visitlab_to_module_dir_name(module_name ${module_name})

    set(module_path "${VISITLAB_MODULE_DIR}/${module_name}")
    if(NOT EXISTS ${module_path})
        message(FATAL_ERROR "The module directory '${module_path}' was not found.")
    endif()
    set(${retval} ${module_path} PARENT_SCOPE)
endfunction()