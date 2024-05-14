################################################################################
# Common Framework for Computer Graphics Courses at FI MUNI.
#
# Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
# All rights reserved.
#
# Module: PV227
################################################################################

# Creates the module.
visitlab_create_module(module_name)

# Specifies the include directories to use when compiling the module.
target_include_directories(${module_name} PUBLIC include)

# Collects the source files and specified them as target sources.
target_sources(
    ${module_name} 
    PRIVATE
        include/collision_detector.hpp
        include/collider.hpp
        include/collision_data.hpp
        include/utils.hpp

        include/aabb.hpp
        include/ray.hpp

        include/convex_polyhedron.hpp
        src/convex_polyhedron.cpp

        include/voronoi.hpp
        src/voronoi.cpp

        include/gjk2.hpp
        include/epa.hpp
        include/sap.hpp
        include/vclip.hpp
        include/aabb_tree.hpp
        include/bruteforce.hpp

        src/gjk2.cpp
        src/epa.cpp
        src/sap.cpp
        src/vclip.cpp
        src/aabb_tree.cpp
        src/bruteforce.cpp
)