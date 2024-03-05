################################################################################
# Common Framework for Computer Graphics Courses at FI MUNI.
#
# Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
# All rights reserved.
#
# Module: GEOMETRY
################################################################################

# Creates the module.
visitlab_create_module(module_name)

# Finds the external libraries and load their settings.
find_package(tinyobjloader CONFIG REQUIRED)

# Specifies external libraries to link with the module.
target_link_libraries(${module_name} PUBLIC tinyobjloader::tinyobjloader)

# Specifies the include directories to use when compiling the library target defined above.
target_include_directories(${module_name} PUBLIC include geometries)

# Collects the source files and specified them as target sources.
target_sources(
    ${module_name} 
    PRIVATE 
    include/capsule.hpp
    include/cube.hpp
    include/cylinder.hpp
    include/geometry.hpp
    include/geometry_base.hpp
    include/sphere.hpp
    include/teapot.hpp
    include/torus.hpp
    src/geometry_base.cpp
)