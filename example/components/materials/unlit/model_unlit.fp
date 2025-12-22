#version 140

in highp vec4 var_position;
in mediump vec3 var_normal;
in mediump vec2 var_texcoord0;

out vec4 out_fragColor;

uniform mediump sampler2D tex0;

void main() {
  // Pre-multiply alpha since all runtime textures already are

  vec4 color = texture(tex0, var_texcoord0.xy);

  out_fragColor = color;
}
