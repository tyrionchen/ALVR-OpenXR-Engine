#version 400
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

#pragma fragment

layout(binding = 0) uniform sampler2D tex_sampler;

layout(location = 0) in vec2 UV;
layout(location = 0) out vec4 FragColor;

const float delta = 1.0 / 12.92;
const float alpha = 1.0 / 1.055;

float srgb_to_linear_rgb_scalar(float x)
{
    return (x < 0.04045) ?
        (x * delta) : pow((x + 0.055) * alpha, 2.4);
}

// conversion based on: https://www.khronos.org/registry/DataFormat/specs/1.3/dataformat.1.3.html#TRANSFER_SRGB
vec4 srgb_to_linear_rgb(vec3 lrgb)
{
    const float r = srgb_to_linear_rgb_scalar(lrgb.r);
    const float g = srgb_to_linear_rgb_scalar(lrgb.g);
    const float b = srgb_to_linear_rgb_scalar(lrgb.b);
    return vec4(r, g, b, 0.6f);
}

void main()
{
    FragColor = srgb_to_linear_rgb(texture(tex_sampler, UV).rgb);
}
