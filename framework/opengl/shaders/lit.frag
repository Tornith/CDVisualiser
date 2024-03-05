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

uniform vec3 eye_position;     // The position of the eye in world space.
uniform vec4 light_position;   // The position of the light in world space (w=0 for directional lights).
uniform vec4 mat_diffuse;      // The material diffuse component.

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
	// Computes the lighting.
	vec3 N = normalize(in_data.normal_ws);
	vec3 V = normalize(eye_position - in_data.position_ws);
	vec3 L = normalize(light_position.xyz - in_data.position_ws * light_position.w);
	vec3 H = normalize(L + V);

	// Calculates the basic Phong factors.
	float Iamb = 1.0;
	float Idif = max(dot(N, L), 0.0);
	float Ispe = (Idif > 0.0) ? pow(max(dot(N, H), 0.0), 50) : 0.0;

	// Applies the factors to light color.
	vec3 amb = Iamb * vec3(1.0,1.0,1.0);
	vec3 dif = Idif * vec3(1.0,1.0,1.0);
	vec3 spe = Ispe * vec3(0.1,0.1,0.1);

	// Computes the material.
	vec3 mat_ambient = vec3(mat_diffuse.rgb * 0.2);
	vec3 mat_specular = vec3(1.0,1.0,1.0);

	// Computes the final color.
	final_color = vec4(mat_ambient * amb + mat_diffuse.rgb * dif + mat_specular * spe, mat_diffuse.a);
}