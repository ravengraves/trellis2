precision highp float;
varying vec3 v_normal;
varying vec2 v_uv;
uniform vec3 u_light_dir;
uniform vec3 u_light_color;
uniform vec4 u_base_color;
uniform sampler2D u_albedo_tex;
uniform int u_use_texture;

float luma(vec3 c) {
  return dot(c, vec3(0.2126, 0.7152, 0.0722));
}

float quantize(float v) {
  if (v > 0.85) {
    return 1.0;
  }
  if (v > 0.6) {
    return 0.75;
  }
  if (v > 0.35) {
    return 0.5;
  }
  return 0.2;
}

void main() {
  vec3 base = u_base_color.rgb;
  if (u_use_texture != 0) {
    base = texture2D(u_albedo_tex, v_uv).rgb;
  }
  vec3 n = normalize(v_normal);
  float ndotl = abs(dot(n, normalize(u_light_dir)));
  vec3 radiance = u_light_color * ndotl;
  float intensity = clamp(0.15 + luma(radiance), 0.0, 1.0);
  float shade = quantize(intensity);
  gl_FragColor = vec4(base * shade, u_base_color.a);
}
