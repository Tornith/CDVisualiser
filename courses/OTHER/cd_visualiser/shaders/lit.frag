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

// The UBO with camera data.	
layout (std140, binding = 0) uniform CameraBuffer
{
	mat4 projection;	  // The projection matrix.
	mat4 projection_inv;  // The inverse of the projection matrix.
	mat4 view;			  // The view matrix
	mat4 view_inv;		  // The inverse of the view matrix.
	mat3 view_it;		  // The inverse of the transpose of the top-left part 3x3 of the view matrix
	vec3 eye_position;	  // The position of the eye in world space.
};

// The structure holding the information about a single Phong light.
struct PhongLight
{
	vec4 position;                   // The position of the light. Note that position.w should be one for point lights and spot lights, and zero for directional lights.
	vec3 ambient;                    // The ambient part of the color of the light.
	vec3 diffuse;                    // The diffuse part of the color of the light.
	vec3 specular;                   // The specular part of the color of the light. 
	vec3 spot_direction;             // The direction of the spot light, irrelevant for point lights and directional lights.
	float spot_exponent;             // The spot exponent of the spot light, irrelevant for point lights and directional lights.
	float spot_cos_cutoff;           // The cosine of the spot light's cutoff angle, -1 point lights, irrelevant for directional lights.
	float atten_constant;            // The constant attenuation of spot lights and point lights, irrelevant for directional lights. For no attenuation, set this to 1.
	float atten_linear;              // The linear attenuation of spot lights and point lights, irrelevant for directional lights.  For no attenuation, set this to 0.
	float atten_quadratic;           // The quadratic attenuation of spot lights and point lights, irrelevant for directional lights. For no attenuation, set this to 0.
};

// The UBO with light data.
layout (std140, binding = 2) uniform PhongLightsBuffer
{
	vec3 global_ambient_color;		 // The global ambient color.
	int lights_count;				 // The number of lights in the buffer.
	PhongLight lights[8];			 // The array with actual lights.
};

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
	// Computes the lighting.
	vec3 N = normalize(in_data.normal_ws);
	vec3 V = normalize(eye_position - in_data.position_ws);
	
	// Sets the starting coefficients.
	vec3 amb = global_ambient_color;
	vec3 dif = vec3(0.0);
	vec3 spe = vec3(0.0);

	// Processes all the lights.
	for (int i = 0; i < lights_count; i++)
	{
		vec3 L_not_normalized = lights[i].position.xyz - in_data.position_ws * lights[i].position.w;
		vec3 L = normalize(L_not_normalized);
		vec3 H = normalize(L + V);

		// Calculates the basic Phong factors.
		float Iamb = 1.0;
		float Idif = max(dot(N, L), 0.0);
		float Ispe = (Idif > 0.0) ? pow(max(dot(N, H), 0.0), material.shininess) : 0.0;

		// Calculates spot light factor.
		if (lights[i].spot_cos_cutoff != -1.0)
		{
			float spot_factor;
			float spot_cos_angle = dot(-L, lights[i].spot_direction);
			if (spot_cos_angle > lights[i].spot_cos_cutoff)
			{
				spot_factor = pow(spot_cos_angle, lights[i].spot_exponent);
			}
			else spot_factor = 0.0;

			Iamb *= 1.0;
			Idif *= spot_factor;
			Ispe *= spot_factor;
		}

		// Calculates attenuation point/spot lights.
		if (lights[i].position.w != 0.0)
		{
			float distance_from_light = length(L_not_normalized);
			float atten_factor =
				lights[i].atten_constant +
				lights[i].atten_linear * distance_from_light + 
				lights[i].atten_quadratic * distance_from_light * distance_from_light;
			atten_factor = 1.0 / atten_factor;

			Iamb *= atten_factor;
			Idif *= atten_factor;
			Ispe *= atten_factor;
		}

		// Applies the factors to light color.
		amb += Iamb * lights[i].ambient;
		dif += Idif * lights[i].diffuse;
		spe += Ispe * lights[i].specular;
	}

	// Computes the material - we either use material or texture as ambient and diffuse parts.
	vec3 mat_ambient = has_texture ? texture(material_diffuse_texture, in_data.tex_coord).rgb :  material.ambient;
	vec3 mat_diffuse = has_texture ? texture(material_diffuse_texture, in_data.tex_coord).rgb :  material.diffuse;
	vec3 mat_specular = material.specular;

	// Computes the final light color.
	vec3 final_light = mat_ambient * amb + mat_diffuse * dif + material.specular * spe;

	// Outputs the final light color.
	final_color = vec4(final_light, material.alpha);
}