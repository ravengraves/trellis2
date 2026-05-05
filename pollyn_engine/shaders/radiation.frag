precision highp float;

varying vec2 v_uv;
uniform sampler2D u_tex;
uniform float u_density;
uniform float u_min_value;
uniform float u_max_value;
uniform float u_seed;

float hash12(vec2 p) {
  vec3 p3 = fract(vec3(p.xyx) * 0.1031);
  p3 += dot(p3, p3.yzx + 33.33);
  return fract((p3.x + p3.y) * p3.z);
}

void main() {
  vec4 src = texture2D(u_tex, v_uv);
  float density = clamp(u_density, 0.0, 1.0);
  float min_value = clamp(min(u_min_value, u_max_value), 0.0, 1.0);
  float max_value = clamp(max(u_min_value, u_max_value), 0.0, 1.0);

  vec2 p = gl_FragCoord.xy + vec2(u_seed * 17.17, u_seed * 91.31);
  float hit = hash12(p);
  if (hit < density) {
    float channel = hash12(p + vec2(19.19, 73.73));
    float elevated = mix(min_value, max_value, hash12(p + vec2(41.41, 11.11)));
    if (channel < 0.333333) {
      src.r = max(src.r, elevated);
    } else if (channel < 0.666667) {
      src.g = max(src.g, elevated);
    } else {
      src.b = max(src.b, elevated);
    }
  }

  gl_FragColor = vec4(src.rgb, 1.0);
}
