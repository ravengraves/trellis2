precision highp float;
varying vec2 v_uv;
uniform sampler2D u_albedo_tex;
uniform int u_use_texture;
uniform float u_znear;
uniform float u_zfar;
const float ALPHA_CUTOFF = 0.5;
const float DEPTH01_MAX = 1.0 - (1.0 / 16777216.0);

float linearize_projected_depth(float depth_buffer) {
  float z_ndc = depth_buffer * 2.0 - 1.0;
  return (2.0 * u_znear * u_zfar) /
    (u_zfar + u_znear - z_ndc * (u_zfar - u_znear));
}

void main() {
  if (u_use_texture != 0) {
    float alpha = texture2D(u_albedo_tex, v_uv).a;
    if (alpha < ALPHA_CUTOFF) {
      discard;
    }
  }
  float projected_depth = linearize_projected_depth(gl_FragCoord.z);
  float depth01 = clamp((projected_depth - u_znear) / (u_zfar - u_znear), 0.0, DEPTH01_MAX);
  vec4 enc = vec4(1.0, 255.0, 65025.0, 16581375.0) * depth01;
  enc = fract(enc);
  enc -= enc.yzww * vec4(1.0/255.0, 1.0/255.0, 1.0/255.0, 0.0);
  gl_FragColor = enc;
}
