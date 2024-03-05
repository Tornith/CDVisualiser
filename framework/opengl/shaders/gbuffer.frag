#version 330 core

#pragma include phong.glsl 

//----------------------------------------------------------------------------
// Input Variables
// ----------------------------------------------------------------------------
in VertexData
{
	vec3 position_ws;	// The vertex position in world space.
	vec3 normal_ws;		// The vertex normal in world space.
	vec2 tex_coord;		// The vertex texture coordinates.
} in_data;

uniform vec3 eye_position;     // The position of the eye in world space.
uniform vec3 light_position;   // The position of the light in world space.

// ----------------------------------------------------------------------------
// Output Variables
// ----------------------------------------------------------------------------
// The final output color.
layout(location = 0) out vec4 position_ws;
layout(location = 1) out vec4 normal_ws;
layout(location = 2) out vec4 albedo;

// ----------------------------------------------------------------------------
// Main Method
// ----------------------------------------------------------------------------
void main()
{
	// Computes the lighting.
	vec3 N = normalize(in_data.normal_ws);
	vec3 V = normalize(eye_position - in_data.position_ws);
	vec3 L = normalize(light_position - in_data.position_ws);
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
	vec3 mat_ambient = vec3(0.3,0.3,0.3);
	vec3 mat_diffuse = vec3(0.5,0.0,0.0);
	vec3 mat_specular = vec3(1.0,1.0,1.0);

	position_ws = vec4(in_data.position_ws, 1.0);
	normal_ws = vec4(normalize(in_data.normal_ws), 0.0);
	albedo = vec4(mat_diffuse, 1.0);
}