#version 400
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

#pragma fragment

layout(binding=0) uniform sampler2D tex_sampler;

layout (location = 0) in vec2 UV;   
layout (location = 0) out vec4 FragColor;

void main()
{
    FragColor = texture(tex_sampler, UV);
}
