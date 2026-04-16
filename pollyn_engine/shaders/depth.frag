precision highp float;
varying float v_depth;
varying vec2 v_uv;
uniform sampler2D u_albedo_tex;
uniform int u_use_texture;
const float ALPHA_CUTOFF = 0.5;
void main() {
  if (u_use_texture != 0) {
    float alpha = texture2D(u_albedo_tex, v_uv).a;
    if (alpha < ALPHA_CUTOFF) {
      discard;
    }
  }
  vec4 enc = vec4(1.0, 255.0, 65025.0, 16581375.0) * v_depth;
  enc = fract(enc);
  enc -= enc.yzww * vec4(1.0/255.0, 1.0/255.0, 1.0/255.0, 0.0);
  gl_FragColor = enc;
}
