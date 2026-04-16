precision highp float;
uniform vec4 u_color;
uniform sampler2D u_albedo_tex;
uniform int u_use_texture;
varying vec2 v_uv;
const float ALPHA_CUTOFF = 0.5;
void main() {
  float alpha = 1.0;
  if (u_use_texture != 0) {
    alpha = texture2D(u_albedo_tex, v_uv).a;
    if (alpha < ALPHA_CUTOFF) {
      discard;
    }
  }
  gl_FragColor = vec4(u_color.rgb, u_color.a * alpha);
}
