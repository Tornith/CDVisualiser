#version 330 core

// ----------------------------------------------------------------------------
// Input Variables
// ----------------------------------------------------------------------------
layout(location = 0) in vec4 position;  // The vertex position.
layout(location = 1) in vec3 normal;    // The vertex normal.
layout(location = 2) in vec2 tex_coord; // The vertex texture coordinates.
layout(location = 5) in vec3 color;     // The vertex color.
	
uniform mat4 projection;	// The projection matrix.
uniform mat4 view;			// The view matrix.
uniform mat4 model;			// The model matrix.

// ----------------------------------------------------------------------------
// Output Variables
// ----------------------------------------------------------------------------
out VertexData
{
	vec3 position_ws;	  // The vertex position in world space.
	vec3 normal_ws;		  // The vertex normal in world space.
	vec2 tex_coord;		  // The vertex texture coordinates.
	vec4 fragment_color;  // The output color that will be interpolated in fragment shader.   
} out_data;

// ----------------------------------------------------------------------------
// Main Method
// ----------------------------------------------------------------------------
void main()
{
	out_data.tex_coord = tex_coord;
	out_data.position_ws = vec3(model * position);
	out_data.normal_ws = vec3(normalize(transpose(inverse(model)) * vec4(normal, 0.0)));
	out_data.fragment_color = vec4(color, 1.0);

	gl_Position = projection * view * model * position;
}
