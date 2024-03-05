#version 430 core

// ----------------------------------------------------------------------------
// Input Variables
// ----------------------------------------------------------------------------
layout(location = 0) in vec4 position;  // The vertex position.
layout(location = 1) in vec3 normal;    // The vertex normal.
layout(location = 2) in vec2 tex_coord; // The vertex texture coordinates.
layout(location = 5) in vec3 color;     // The vertex color.
	
uniform mat4 projection;	// The projection matrix.
uniform mat4 view;			// The view matrix.

// The UBO with the model data.
layout(std140, binding = 1) uniform ModelData
{
   mat4 model;			// The model matrix.
   mat4 model_inv;		// The inverse of the model matrix.
   mat3 model_it;		// The inverse of the transpose of the top-left part 3x3 of the model matrix.
};

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
	out_data.normal_ws = normalize(model_it * normal);

	gl_Position = projection * view * model * position;
}
