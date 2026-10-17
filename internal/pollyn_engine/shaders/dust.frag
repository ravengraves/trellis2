precision highp float;

varying vec2 v_uv;
uniform sampler2D u_color_tex;
uniform sampler2D u_depth_tex;
uniform vec2 u_texel;
uniform vec3 u_dust_color;
uniform float u_znear;
uniform float u_zfar;
uniform float u_start;
uniform float u_end;
uniform float u_falloff_distance;
uniform float u_density;
uniform float u_noise_strength;
uniform float u_noise_scale;
uniform float u_blur_px;
uniform float u_time;

float decode_depth01(vec4 enc) {
  return dot(enc, vec4(1.0, 1.0 / 255.0, 1.0 / 65025.0, 1.0 / 16581375.0));
}

float hash12(vec2 p) {
  vec3 p3 = fract(vec3(p.xyx) * 0.1031);
  p3 += dot(p3, p3.yzx + 33.33);
  return fract((p3.x + p3.y) * p3.z);
}

void main() {
  vec4 depth_enc = texture2D(u_depth_tex, v_uv);
  float depth01 = clamp(decode_depth01(depth_enc), 0.0, 1.0);
  float depth = u_znear + depth01 * (u_zfar - u_znear);

  float denom = max(u_end - u_start, 0.0001);
  if (u_falloff_distance > 0.0) {
    denom = max(u_falloff_distance, 0.0001);
  }
  float fog = smoothstep(0.0, 1.0, clamp((depth - u_start) / denom, 0.0, 1.0));
  fog = clamp(fog * max(u_density, 0.0), 0.0, 1.0);

  float n = hash12(v_uv * max(u_noise_scale, 0.001) + vec2(u_time * 0.071, u_time * 0.037));
  fog = clamp(fog + (n - 0.5) * u_noise_strength, 0.0, 1.0);

  vec3 center = texture2D(u_color_tex, v_uv).rgb;
  float radius = max(u_blur_px, 0.0) * fog;
  vec2 o = u_texel * radius;
  vec3 blur = center * 0.40;
  blur += texture2D(u_color_tex, v_uv + vec2( o.x, 0.0)).rgb * 0.15;
  blur += texture2D(u_color_tex, v_uv + vec2(-o.x, 0.0)).rgb * 0.15;
  blur += texture2D(u_color_tex, v_uv + vec2(0.0,  o.y)).rgb * 0.15;
  blur += texture2D(u_color_tex, v_uv + vec2(0.0, -o.y)).rgb * 0.15;

  gl_FragColor = vec4(mix(blur, u_dust_color, fog), 1.0);
}
