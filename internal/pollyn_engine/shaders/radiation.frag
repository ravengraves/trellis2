precision highp float;

varying vec2 v_uv;
uniform sampler2D u_tex;
uniform float u_density;
uniform float u_min_value;
uniform float u_max_value;
uniform float u_seed;
uniform int u_overlay;

float hash12(vec2 p) {
  vec3 p3 = fract(vec3(p.xyx) * vec3(0.1031, 0.11369, 0.13787));
  p3 += dot(p3, p3.yzx + 19.19);
  return fract((p3.x + p3.y) * p3.z);
}

float hash12_wide(vec2 p) {
  float a = hash12(p);
  float b = hash12(p + vec2(a * 37.17 + 11.13, a * 19.91 + 73.73));
  return fract(a + b * 0.61803398875);
}

void main() {
  vec4 src = vec4(0.0, 0.0, 0.0, 0.0);
  if (u_overlay == 0) {
    src = texture2D(u_tex, v_uv);
  }
  float density = clamp(u_density, 0.0, 1.0);
  float min_value = clamp(min(u_min_value, u_max_value), 0.0, 1.0);
  float max_value = clamp(max(u_min_value, u_max_value), 0.0, 1.0);

  vec2 seed_jitter = vec2(
    hash12(vec2(u_seed, 17.17)),
    hash12(vec2(91.31, u_seed))) * 4096.0;
  vec2 p = gl_FragCoord.xy + seed_jitter;
  float hit = hash12_wide(p);
  if (hit < density) {
    float channel = hash12_wide(p + vec2(19.19, 73.73));
    float elevated = mix(min_value, max_value, hash12_wide(p + vec2(41.41, 11.11)));
    if (channel < 0.333333) {
      src.r = max(src.r, elevated);
    } else if (channel < 0.666667) {
      src.g = max(src.g, elevated);
    } else {
      src.b = max(src.b, elevated);
    }
    if (u_overlay != 0) {
      src.a = 0.75;
    }
  }

  gl_FragColor = vec4(src.rgb, u_overlay != 0 ? src.a : 1.0);
}
