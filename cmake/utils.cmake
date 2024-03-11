################################################################################
# Common Framework for Computer Graphics Courses at FI MUNI.
#
# Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
# All rights reserved.
#
# The CMake file defining global utility macros and functions.
################################################################################

#--------------------------------------------------------------------
# Prepares the build process, resets global variables, etc.
#--------------------------------------------------------------------
function(visitlab_prepare_build)
    set(visitlab_current_target "" CACHE INTERNAL "Current Target")
    set(visitlab_apps_list "" CACHE INTERNAL "Registered Applications")
    set(visitlab_required_modules "" CACHE INTERNAL "Required Modules")
endfunction()

#--------------------------------------------------------------------
# Retrives dependencies from 'depends.cmake'.
#--------------------------------------------------------------------
# @param retval        - The variable where the retrieved dependencies will be stored.
# @param absolute_path - The path to the directory with dependencies.
#--------------------------------------------------------------------
function(visitlab_get_dependencies retval absolute_path)
    set(dependencies "")
    set(file_with_dependencies "${absolute_path}/depends.cmake")
    if(EXISTS ${file_with_dependencies})
        include(${file_with_dependencies})
        #message(STATUS "Attempting to include dependencies from '${file_with_dependencies}' file.")
    else()
        message(STATUS "No dependency file 'depends.cmake' found in '${absolute_path}' directory.")
    endif()

    set(${retval} ${dependencies} PARENT_SCOPE)
    unset(dependencies)
endfunction()

#--------------------------------------------------------------------
# Resolves dependencies for the specified target including indirect 
# dependencies that are imposed by the required modules.
#--------------------------------------------------------------------
# @param target_name   - The target for which the dependencies 
#                        should be resolved.
# @param absolute_path - The path to the directory with dependencies.
#--------------------------------------------------------------------
function(visitlab_resolve_dependencies target_name absolute_path)
    # Resolves the direct dependencies.
    visitlab_get_dependencies(dependencies ${absolute_path})
    message(STATUS "Dependencies: '${dependencies}'")
    # Resolves the indirect dependencies imposed by the required modules.
    visitlab_get_module_dependencies(indirect_dependencies ${dependencies})
    message(STATUS "Indirect Dependencies: '${indirect_dependencies}'")
    list(APPEND dependencies ${indirect_dependencies})
    list(REMOVE_DUPLICATES dependencies)

     # Add the all the dependencies into the list of required modules for later use.
    list(APPEND visitlab_required_modules ${dependencies})
    list(REMOVE_DUPLICATES visitlab_required_modules)
    set(visitlab_required_modules ${visitlab_required_modules} CACHE INTERNAL "Required Modules")
    message(STATUS "Updating the list of all required modules: '${visitlab_required_modules}'")

    # Links the required modules. 
    message(STATUS "Setting the link libraries for '${target_name}' to: '${dependencies}'")
    target_link_libraries(${target_name} PRIVATE ${dependencies})
endfunction()

#--------------------------------------------------------------------
# Macros for adding compile and link flags.
#--------------------------------------------------------------------
macro(visitlab_append_compile_flags)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ARGN}")
endmacro()
macro(visitlab_append_linker_flags)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ARGN}")
endmacro()

#--------------------------------------------------------------------
# Naming conventions
# analyst                : Name of an application directory should be lowercase with no spaces.
# PV112                  : Name of a course directory should be uppercase with no spaces.
# core                   : Name of a module directory should be lowercase with no spaces.
# CORE_MODULE            : Internal module name should be uppercase with _MODULE sufix and no spaces.
#--------------------------------------------------------------------

#--------------------------------------------------------------------
# Converts one or multiple names to application directory names.
#--------------------------------------------------------------------
# @param retval - The variable where the updated names will be stored.
#--------------------------------------------------------------------
function(visitlab_to_application_dir_name retval)
    foreach(item ${ARGN})
        string(TOLOWER ${item} app_name)
        list(APPEND result_list "${app_name}")
    endforeach()
    set(${retval} ${result_list} PARENT_SCOPE)
endfunction()

#--------------------------------------------------------------------
# Converts one or multiple names to course directory names.
#--------------------------------------------------------------------
# @param retval - The variable where the updated names will be stored.
#--------------------------------------------------------------------
function(visitlab_to_course_dir_name retval)
    foreach(item ${ARGN})
        string(TOUPPER ${item} course_name)
        list(APPEND result_list "${course_name}")
    endforeach()
    set(${retval} ${result_list} PARENT_SCOPE)
endfunction()

#--------------------------------------------------------------------
# Converts one or multiple names to module directory names.
#--------------------------------------------------------------------
# @param retval - The variable where the updated names will be stored.
#--------------------------------------------------------------------
function(visitlab_to_module_dir_name retval)
    foreach(item ${ARGN})
        string(TOLOWER ${module_name} module_name)
        # Strips the posible module sufix.
        string(REPLACE "_module" "" module_name ${module_name})
        list(APPEND result_list "${module_name}")
    endforeach()
    set(${retval} ${result_list} PARENT_SCOPE)
endfunction()

#--------------------------------------------------------------------
# Converts one or multiple names to internal module names.
#--------------------------------------------------------------------
# @param retval - The variable where the updated names will be stored.
#--------------------------------------------------------------------
function(visitlab_to_internal_module_name retval)
    foreach(item ${ARGN})
        string(TOUPPER ${item} module_name_u)
        string(FIND ${module_name_u} "_MODULE" has_module_sufix)
        if(${has_module_sufix} EQUAL "-1")
            set(module_name_u "${module_name_u}_MODULE")
        endif()
        list(APPEND result_list "${module_name_u}")
    endforeach()
    set(${retval} ${result_list} PARENT_SCOPE)
endfunction()