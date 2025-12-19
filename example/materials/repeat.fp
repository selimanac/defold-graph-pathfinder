#version 140

in highp vec4             var_position;
in mediump vec3           var_normal;
in mediump vec2           var_texcoord0;

out vec4                  out_fragColor;

uniform mediump sampler2D tex0;

uniform fs_uniforms
{
    mediump vec4 uv_repeat;
};

void main()
{
    // Repeat the texture coordinates
    mediump vec2 uv = fract(var_texcoord0 * uv_repeat.xy);

    // Sample texture with repeated UVs
    vec4 color = texture(tex0, uv);

    out_fragColor = color;
}
