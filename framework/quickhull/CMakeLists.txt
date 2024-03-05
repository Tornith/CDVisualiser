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
        include/Mesh.hpp
        include/Plane.hpp
        include/Pool.hpp
        include/Ray.hpp
        include/Vector3.hpp
        include/VertexDataSource.hpp

        include/ConvexHull.hpp
        include/HalfEdgeMesh.hpp
        include/MathUtils.hpp
        include/QuickHull.hpp

        src/QuickHull.cpp
)