precision highp float;
varying vec3 v_world_pos;
varying vec3 v_world_norm;
varying vec2 v_uv;
uniform vec3 u_light_dir;
uniform vec3 u_light_color;
uniform vec3 u_light2_dir;
uniform vec3 u_light2_color;
uniform vec3 u_eye_pos;
uniform vec3 u_albedo;
uniform float u_metallic;
uniform float u_roughness;
uniform vec3 u_env_sky;
uniform vec3 u_env_ground;
uniform vec3 u_env_horizon;
uniform vec3 u_env_specular;
uniform float u_env_intensity;
uniform float u_alpha;
uniform sampler2D u_albedo_tex;
uniform int u_use_texture;

const float PI = 3.14159265;

vec3 env_irradiance(vec3 n) {
  float up = n.z * 0.5 + 0.5;
  vec3 c;
  if (up < 0.5) { c = mix(u_env_ground, u_env_horizon, up * 2.0); }
  else { c = mix(u_env_horizon, u_env_sky, (up - 0.5) * 2.0); }
  return c * u_env_intensity;
}

vec3 env_spec(vec3 r, float rough) {
  float up = r.z * 0.5 + 0.5;
  float sh = 1.0 - rough;
  float biased = 0.5 + (up - 0.5) * sh;
  vec3 c;
  if (biased < 0.5) { c = mix(u_env_ground, u_env_horizon, biased * 2.0); }
  else { c = mix(u_env_horizon, u_env_specular, (biased - 0.5) * 2.0); }
  return c * u_env_intensity;
}

vec3 pbr_direct(vec3 n, vec3 v, vec3 l, vec3 light_color, vec3 albedo,
                float metal, float rough, vec3 f0) {
  vec3 h = normalize(v + l);
  float ndotl = max(dot(n, l), 0.0);
  float ndotv = max(dot(n, v), 0.0001);
  float ndoth = max(dot(n, h), 0.0);
  float vdoth = max(dot(v, h), 0.0);
  float omv = 1.0 - vdoth;
  float omv2 = omv * omv;
  float omv5 = omv2 * omv2 * omv;
  vec3 F = f0 + (1.0 - f0) * omv5;
  float a2 = rough * rough * rough * rough;
  float dd = ndoth * ndoth * (a2 - 1.0) + 1.0;
  float D = a2 / (PI * dd * dd);
  float k = (rough + 1.0) * (rough + 1.0) / 8.0;
  float G = (ndotv / (ndotv * (1.0 - k) + k)) *
            (ndotl / (ndotl * (1.0 - k) + k));
  vec3 spec = F * D * G / (4.0 * ndotv * ndotl + 0.0001);
  float kd = 1.0 - metal;
  vec3 diff = albedo * kd * (1.0 - F) / PI;
  return (diff + spec) * light_color * ndotl;
}

void main() {
  vec3 albedo = u_albedo;
  if (u_use_texture != 0) {
    albedo = texture2D(u_albedo_tex, v_uv).rgb;
  }
  vec3 n = normalize(v_world_norm);
  vec3 v = normalize(u_eye_pos - v_world_pos);
  float rough = clamp(u_roughness, 0.04, 1.0);
  float metal = clamp(u_metallic, 0.0, 1.0);
  vec3 f0 = mix(vec3(0.04), albedo, metal);
  vec3 l1 = normalize(u_light_dir);
  vec3 l2 = u_light2_dir;
  float l2_len = length(l2);
  if (l2_len > 0.0001) { l2 = l2 / l2_len; }
  else { l2 = vec3(0.0); }
  vec3 direct = pbr_direct(n, v, l1, u_light_color, albedo, metal, rough, f0) +
                pbr_direct(n, v, l2, u_light2_color, albedo, metal, rough, f0);
  vec3 irr = env_irradiance(n);
  vec3 refl = reflect(-v, n);
  vec3 se = env_spec(refl, rough);
  float kd = 1.0 - metal;
  vec3 amb = irr * albedo * kd * 0.3 + se * f0 * 0.3;
  vec3 col = direct + amb;
  col = col / (col + 1.0);
  col = sqrt(max(col, 0.0));
  gl_FragColor = vec4(clamp(col, 0.0, 1.0), u_alpha);
}
