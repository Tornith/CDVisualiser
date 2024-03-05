#version 330 core

//-----------------------------------------------------------------------------
// Input Variables
// ----------------------------------------------------------------------------
in VertexData
{
	vec3 position_ws;	  // The vertex position in world space.
	vec3 normal_ws;		  // The vertex normal in world space.
	vec2 tex_coord;		  // The vertex texture coordinates.
	vec4 fragment_color;  // The output color that will be interpolated in fragment shader. 
} in_data;

// The texture that will be displayed.
uniform sampler2D texture_to_depict;

// ----------------------------------------------------------------------------
// Output Variables
// ----------------------------------------------------------------------------
// The final output color.
layout(location = 0) out vec4 final_color;

// ----------------------------------------------------------------------------
// Main Method
// ----------------------------------------------------------------------------
void main() {
    final_color = vec4(texture(texture_to_depict, in_data.tex_coord).rgb, 1.0);

    // shows texture cords if needed
    // final_color = vec4(in_data.tex_coord,0,1);
}