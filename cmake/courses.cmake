################################################################################
# Common Framework for Computer Graphics Courses at FI MUNI.
#
# Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
# All rights reserved.
#
# The CMake file defining macros and functions for compiling individual courses.
################################################################################

#--------------------------------------------------------------------
# Registers the specified course to be build. 
#--------------------------------------------------------------------
# @param course - The name of the course to build.
#--------------------------------------------------------------------
function(visitlab_register_course course)
    message("----------------------------------------------------------------------")
    message("Setting up '${course}' course.")
    message("----------------------------------------------------------------------")
    
    # Resolves the application name.
    visitlab_to_course_dir_name(course ${course})
     # Resolves the course absolute path.
    visitlab_get_course_path(${course} course_path)

    if((${course_path} STREQUAL "") OR (NOT EXISTS "${course_path}/CMakeLists.txt"))
        message(STATUS "Skipping...")
    else()
        message(STATUS "Adding course directory '${course_path}'.")
        add_subdirectory(${course_path})
    endif()
endfunction()

#--------------------------------------------------------------------
# Adds a specified lecture subdirectory, if available.
#--------------------------------------------------------------------
# @param lecture_dir - The lecture directory to add.
#--------------------------------------------------------------------
function(visitlab_add_directory lecture_dir)
    set(lecture_path "${CMAKE_CURRENT_LIST_DIR}/${lecture_dir}")
    if(EXISTS ${lecture_path})
        add_subdirectory(${lecture_path})
    else()
        message("----------------------------------------------------------------------")
        message("The directory for '${lecture_dir}' was not found.")
        message("----------------------------------------------------------------------")
        message(STATUS "${lecture_path}")
    endif()
endfunction()

#--------------------------------------------------------------------
# Gets the absolute path to the specified application.
#--------------------------------------------------------------------
# @param retval - The variable where the path will be stored.
# @param course - The name of an course for which the path is 
#                 requested. 
#--------------------------------------------------------------------
function(visitlab_get_course_path course retval)
    visitlab_to_course_dir_name(course ${course})

    set(course_path "${VISITLAB_COURSE_DIR}/${course}")
    if(NOT EXISTS ${course_path})
        message(STATUS "The course folder '${course_path}' was not found.")
        set(course_path "")
    endif()
    set(${retval} ${course_path} PARENT_SCOPE)
endfunction()

#--------------------------------------------------------------------
# Generates a single lecture.
#--------------------------------------------------------------------
# @param course   - The course into which the lecture belongs.
# @param lecture  - The lecture name.
# @param SOLUTION - The flag that can be used to build a solution.
#                   Note that it will NOT work in skeletons :).
#--------------------------------------------------------------------
function(visitlab_generate_lecture course lecture)
    # Processes the parameters.
    set(options SOLUTION)
    set(oneValueArgs "")
    set(multiValueArgs EXTRA_FILES STUDENT)
    cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    message("----------------------------------------------------------------------")
    message("Setting up '${lecture}' in '${course}' course (SOLUTION: ${ARG_SOLUTION}).")
    message("Extra files: ${ARG_EXTRA_FILES}")
    message("----------------------------------------------------------------------")

    # Resolves the application name.
    visitlab_to_course_dir_name(course ${course})
    # Resolves the course absolute path.
    visitlab_get_course_path(${course} course_path)
    # Resolves the lecture absolute path.
    set(lecture_path "${course_path}/${lecture}")

    # Adds a new executable target - if solution flag is used cmake tries to look for the source files in solution folder.
    if(ARG_STUDENT)
        set(executable_target "${course}_${ARG_STUDENT}")
        add_executable(${executable_target} "application.hpp" "main.cpp" ${ARG_EXTRA_FILES} ${ARG_STUDENT})
    elseif(ARG_SOLUTION AND EXISTS "${lecture_path}/solution/solution.cpp")
        set(executable_target "${course}_${lecture}_solution")
        add_executable(${executable_target} "application.hpp" "solution/solution.cpp" "main.cpp" ${ARG_EXTRA_FILES})
    elseif(NOT ARG_SOLUTION)
        set(executable_target "${course}_${lecture}")
        add_executable(${executable_target} "application.hpp" "application.cpp" "main.cpp" ${ARG_EXTRA_FILES})
    else()
        message(STATUS "Solution not available, terminating...")
        RETURN()
    endif()
    message(STATUS "Generating ${executable_target} target.")
    
    # Sets the build properties.
    # CXX_STANDARD - the C++ standard whose features are requested.
    # CXX_EXTENSIONS - no compiler specific extensions are requested.
    set_target_properties(${executable_target}
        PROPERTIES CXX_STANDARD 20
                   CXX_EXTENSIONS OFF
    )

    # Solves the lecture dependencies.
    visitlab_resolve_dependencies(${executable_target} ${course_path})
    if(NOT (ARG_SOLUTION OR ARG_STUDENT))
        # Adds a new executable target for testing the solution if it exists.
        if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests")
            enable_testing()
            # Adds the folder with tests.
            add_subdirectory(tests)
        endif()

        # Adds a new executable target for generating the tests if it exists.
        if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test_generators")
            # Adds the folder with test generators.
            add_subdirectory(test_generators)
        endif()

        # Generates the 'configuration.toml' file.
        file(GENERATE
            OUTPUT "$<TARGET_FILE_DIR:${executable_target}>/configuration.toml"
            CONTENT "
framework_dir = \"${CMAKE_SOURCE_DIR}/framework/\" 
lecture_dir = \"${CMAKE_CURRENT_SOURCE_DIR}\"")
    endif()
endfunction()


#--------------------------------------------------------------------
# Generates a tests for the specified lecture.
#--------------------------------------------------------------------
# @param course  - The course into which the lecture belongs.
# @param lecture - The lecture name.
#--------------------------------------------------------------------
function(visitlab_generate_lecture_tests course lecture)
    set(options SOLUTION)
    set(oneValueArgs "")
    set(multiValueArgs EXTRA_FILES STUDENT)
    cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    message("----------------------------------------------------------------------")
    message("Setting up tests for '${lecture}' in '${course}' course (SOLUTION: ${ARG_SOLUTION}).")
    message("Extra files: ${ARG_EXTRA_FILES}")
    message("----------------------------------------------------------------------")

    # Resolves the course absolute path.
    visitlab_get_course_path(${course} course_path)
    # Resolves the lecture absolute path.
    set(lecture_path "${course_path}/${lecture}")

    
    # Adds a new executable target - if solution flag is used cmake tries to look for the source files in solution folder.
    if(ARG_STUDENT)
        set(test_target_name ${course}_test_${ARG_STUDENT})
        add_executable(${test_target_name} "../application.hpp" "../${ARG_STUDENT}" "test.cpp" ${ARG_EXTRA_FILES})
    elseif(ARG_SOLUTION AND EXISTS "${lecture_path}/solution/solution.cpp")
        set(test_target_name ${course}_${lecture}_test_solution)
        add_executable(${test_target_name} "../application.hpp" "../solution/solution.cpp" "test.cpp" ${ARG_EXTRA_FILES})
    elseif(NOT ARG_SOLUTION)
        set(test_target_name ${course}_${lecture}_test)
        add_executable(${test_target_name} "../application.hpp" "../application.cpp" "test.cpp" ${ARG_EXTRA_FILES})
    else()
        message(STATUS "Solution not available, terminating...")
        RETURN()
    endif()

    # Solves the lecture dependencies.
    visitlab_resolve_dependencies(${test_target_name} ${course_path})

    # Exposes the test.
    add_test(NAME ${test_target_name} COMMAND ${test_target_name})

    # Sets the build properties.
    # CXX_STANDARD - the C++ standard whose features are requested.
    # CXX_EXTENSIONS - no compiler specific extensions are requested.
    set_target_properties(${test_target_name}
        PROPERTIES CXX_STANDARD 20
                   CXX_EXTENSIONS OFF
    )

    if(NOT (ARG_SOLUTION OR ARG_STUDENT))
        file(
            GENERATE
            OUTPUT "$<TARGET_FILE_DIR:${test_target_name}>/configuration.toml"
            CONTENT "
csv_files = \"${VISITLAB_COURSE_DIR}/${course}/${lecture}/tests/csv\"
data = \"${VISITLAB_COURSE_DIR}/${course}/${lecture}/tests/data\"
test_dir = \"${VISITLAB_COURSE_DIR}/${course}/${lecture}/tests\"")
    endif()
endfunction()


#--------------------------------------------------------------------
# Generates a tests generators for the specified lecture.
#--------------------------------------------------------------------
# @param course  - The course into which the lecture belongs.
# @param lecture - The lecture name.
#--------------------------------------------------------------------
function(visitlab_generate_lecture_tests_generators course lecture)
    message("----------------------------------------------------------------------")
    message("Setting up tests generators for '${lecture}' in '${course}' course.")
    message("----------------------------------------------------------------------")

    # Resolves the course absolute path.
    visitlab_get_course_path(${course} course_path)

    # Sets the executable name.
    set(gen_test_target_name ${course}_${lecture}_gen_tests)

    # Adds a new executable target.
    add_executable(${gen_test_target_name} "generate_tests.cpp" "../application.hpp" "../solution/solution.cpp")

    # Solves the lecture dependencies.
    visitlab_resolve_dependencies(${gen_test_target_name} ${course_path})

    # Sets the build properties.
    # CXX_STANDARD - the C++ standard whose features are requested.
    # CXX_EXTENSIONS - no compiler specific extensions are requested.
    set_target_properties(${gen_test_target_name} PROPERTIES 
        CXX_STANDARD 20             
        CXX_EXTENSIONS OFF
    )

    # Generats the configuration file storing the absolute path for csv files with expected test resutls.
    file(
        GENERATE
        OUTPUT "$<TARGET_FILE_DIR:${gen_test_target_name}>/configuration.toml"
        CONTENT "csv_files = \"${VISITLAB_COURSE_DIR}/${course}/${lecture}/tests/csv\""
    )

    # Copies the files with test scenarios.
    foreach(csv_file ${ARGN})
        file(
            GENERATE
            OUTPUT "$<TARGET_FILE_DIR:${gen_test_target_name}>/${csv_file}"
            INPUT "${csv_file}"
        )
    endforeach()
endfunction()