#version 430 core

//-----------------------------------------------------------------------------
// Input Variables
// ----------------------------------------------------------------------------
in VertexData {
    vec2 tex_coord; // The vertex texture coordinates.
}
in_data;

// The texture that will be displayed.
layout(binding = 0) uniform sampler2D texture_to_depict;

// The flag determining if a standard or depth texture is being rendered.
layout(location = 0) uniform bool depth_texture;

// ----------------------------------------------------------------------------
// Output Variables
// ----------------------------------------------------------------------------
// The final output color.
layout(location = 0) out vec4 final_color;

// ----------------------------------------------------------------------------
// Main Method
// ----------------------------------------------------------------------------
void main() {
    // We can use 'texture' as the condition is based on uniform variable and
    // such a control flow is always uniform.
    if (depth_texture) {
        final_color = vec4(texture(texture_to_depict, in_data.tex_coord).rrr, 1.0);
    } else {
        final_color = vec4(texture(texture_to_depict, in_data.tex_coord).rgb, 1.0);
    }
}