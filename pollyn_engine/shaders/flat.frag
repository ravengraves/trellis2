precision highp float;
varying vec3 v_normal;
varying vec2 v_uv;
uniform vec3 u_light_dir;
uniform vec3 u_light_color;
uniform vec4 u_base_color;
uniform sampler2D u_albedo_tex;
uniform int u_use_texture;
void main() {
  vec3 base = u_base_color.rgb;
  if (u_use_texture != 0) {
    base = texture2D(u_albedo_tex, v_uv).rgb;
  }
  vec3 n = normalize(v_normal);
  float ndotl = max(dot(n, u_light_dir), 0.0);
  float intensity = 0.15 + ndotl * 0.85;
  vec3 col = base * u_light_color * intensity;
  gl_FragColor = vec4(col, u_base_color.a);
}
