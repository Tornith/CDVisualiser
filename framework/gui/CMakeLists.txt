################################################################################
# Common Framework for Computer Graphics Courses at FI MUNI.
#
# Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
# All rights reserved.
#
# Module: GUI
################################################################################

# Creates the module.
visitlab_create_module(module_name)

# Finds the external libraries and load their settings.
find_package(imgui CONFIG REQUIRED)
# Replace the code below with following code for older VCPKG: find_path(STB_INCLUDE_DIRS "stb.h") 
find_path(STB_INCLUDE_DIRS "stb_c_lexer.h")

# Specifies external libraries to link with the module.
target_link_libraries(${module_name} PUBLIC imgui::imgui)

# Specifies the include directories to use when compiling the module.
target_include_directories(${module_name} PUBLIC include ${STB_INCLUDE_DIRS})

# A hack to build the module witout any actual *.cpp file.
target_sources(${module_name} PRIVATE 
	"include/camera.h" 
	"include/gui_application.h" 
	"include/gui_manager.h"
	"src/camera.cpp" 
	"src/gui_application.cpp" 
	"src/gui_manager.cpp"
)