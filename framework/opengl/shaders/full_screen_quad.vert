#version 330 core

// ----------------------------------------------------------------------------
// Local Variables
// ----------------------------------------------------------------------------
// Texture coordinates for the full screen quad.
const vec2 tex_coords[3] = vec2[3] (
	vec2(0.0, 0.0),
	vec2(2.0, 0.0),
	vec2(0.0, 2.0)
);

// ----------------------------------------------------------------------------
// Input Variables
// ----------------------------------------------------------------------------
// There are no input variables for this shader.


// ----------------------------------------------------------------------------
// Output Variables
// ----------------------------------------------------------------------------
out VertexData
{
	vec2 tex_coord;	// The vertex texture coordinates.
} out_data;

// ----------------------------------------------------------------------------
// Main Method
// ----------------------------------------------------------------------------
void main()
{
	// The gl_VertexID holds the index of the vertex that is being processed.
	vec2 tex_coord = tex_coords[gl_VertexID];

	// We render a triangle that is double the window size.
	// The middle part of the triangle thus covers the whole screen.
	out_data.tex_coord = tex_coord;
	gl_Position = vec4(tex_coord * 2.0 - 1.0, 0.0, 1.0);
}
