#version 450 core

//----------------------------------------------------------------------------
// Input Variables
// ----------------------------------------------------------------------------
in VertexData
{
	vec3 position_ws;	  // The vertex position in world space.
	vec3 normal_ws;		  // The vertex normal in world space.
	vec2 tex_coord;		  // The vertex texture coordinates.
} in_data;

// The material data.
layout (std140, binding = 3) uniform PhongMaterialBuffer
{
    vec3 ambient;     // The ambient part of the material.
    vec3 diffuse;     // The diffuse part of the material.
    float alpha;      // The alpha (transparency) of the material.
    vec3 specular;    // The specular part of the material.
    float shininess;  // The shininess of the material.
} material;

// The flag determining whether a texture should be used.
uniform bool has_texture;
// The texture that will be used (if available).
layout(binding = 0) uniform sampler2D material_diffuse_texture;

// ----------------------------------------------------------------------------
// Output Variables
// ----------------------------------------------------------------------------
// The final output color.
layout (location = 0) out vec4 final_color;

// ----------------------------------------------------------------------------
// Main Method
// ----------------------------------------------------------------------------
void main()
{
	// Extracts the diffuse color either from material or texture depending on the value of has_texture variable. 
	vec3 color = material.diffuse;
	if(has_texture){
		color = texture(material_diffuse_texture, in_data.tex_coord).rgb;
	}
	final_color = vec4(color, 1.0);
}