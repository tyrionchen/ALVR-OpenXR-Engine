#version 460
#ifdef ENABLE_ARB_INCLUDE_EXT
    #extension GL_ARB_shading_language_include : require
#else
    // required by glslangValidator
    #extension GL_GOOGLE_include_directive : require
#endif
#pragma fragment

// TODO: Make this into a specialization constant.
const vec3 key_color = vec3(0.01, 0.01, 0.01);

#include "common/baseVideoFrag.glsl"

layout(location = 0) out vec4 FragColor;

void main()
{
    vec4 color = SampleVideoTexture();
    color.a = all(lessThan(color.rgb, key_color)) ? 0.3f : 1.0f;
    FragColor = color;
}
