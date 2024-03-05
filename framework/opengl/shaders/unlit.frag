#version 330 core

//----------------------------------------------------------------------------
// Input Variables
// ----------------------------------------------------------------------------
in VertexData
{
	vec3 position_ws;	  // The vertex position in world space.
	vec3 normal_ws;		  // The vertex normal in world space.
	vec2 tex_coord;		  // The vertex texture coordinates.
	vec4 fragment_color;  // The output color that will be interpolated in fragment shader. 
} in_data;
uniform vec4 mat_diffuse;      // The material diffuse component.
uniform bool use_vertex_color; // The variable determining if the vertex color will be used.

// ----------------------------------------------------------------------------
// Output Variables
// ----------------------------------------------------------------------------
// The final output color.
layout(location = 0) out vec4 final_color;

// ----------------------------------------------------------------------------
// Main Method
// ----------------------------------------------------------------------------
void main()
{
	if(use_vertex_color){
		final_color = vec4(in_data.fragment_color);
	}else{
		final_color = vec4(mat_diffuse);
	}
}