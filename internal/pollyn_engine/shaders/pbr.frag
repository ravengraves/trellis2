precision highp float;
varying vec3 v_world_pos;
varying vec3 v_world_norm;
varying vec2 v_uv;
varying vec3 v_world_tangent;
varying float v_tangent_sign;
uniform vec3 u_eye_pos;
uniform vec3 u_albedo;
uniform float u_metallic;
uniform float u_roughness;
uniform vec3 u_env_sky;
uniform vec3 u_env_ground;
uniform vec3 u_env_horizon;
uniform vec3 u_env_specular;
uniform float u_env_intensity;
uniform float u_env_diffuse_mix;
uniform float u_env_specular_mix;
uniform float u_alpha;
uniform sampler2D u_albedo_tex;
uniform int u_use_texture;
uniform sampler2D u_orm_tex;
uniform int u_use_orm_texture;
uniform sampler2D u_normal_tex;
uniform int u_use_normal_texture;
uniform sampler2D u_env_global_tex;
uniform int u_use_env_global_texture;
uniform sampler2D u_env_global_blur_tex;
uniform int u_use_env_global_blur_texture;
uniform sampler2D u_env_local_tex;
uniform int u_use_env_local_texture;
uniform sampler2D u_env_local_blur_tex;
uniform int u_use_env_local_blur_texture;
const int MAX_PBR_LIGHTS = 4;
uniform int u_light_count;
uniform vec4 u_light_pos_type[MAX_PBR_LIGHTS];
uniform vec4 u_light_dir_data[MAX_PBR_LIGHTS];
uniform vec4 u_light_color[MAX_PBR_LIGHTS];
const float ALPHA_CUTOFF = 0.5;

const float PI = 3.14159265;
const float INV_PI = 0.31830988618;

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

vec2 env_equirect_uv(vec3 dir) {
  vec3 d = normalize(dir);
  float u = atan(d.y, d.x) * (0.5 * INV_PI) + 0.5;
  float v = acos(clamp(d.z, -1.0, 1.0)) * INV_PI;
  return vec2(fract(u), clamp(v, 0.0, 1.0));
}

vec3 sample_env_map(sampler2D tex, vec3 dir) {
  return texture2D(tex, env_equirect_uv(dir)).rgb * u_env_intensity;
}

vec3 env_diffuse_ibl(vec3 n) {
  vec3 global = env_irradiance(n);
  if (u_use_env_global_texture != 0) {
    global = sample_env_map(u_env_global_tex, n);
  }
  if (u_use_env_local_texture != 0) {
    vec3 local = sample_env_map(u_env_local_tex, n);
    return mix(global, local, clamp(u_env_diffuse_mix, 0.0, 1.0));
  }
  return global;
}

vec3 env_specular_ibl(vec3 r, float rough) {
  vec3 global_sharp = env_spec(r, 0.0);
  if (u_use_env_global_texture != 0) {
    global_sharp = sample_env_map(u_env_global_tex, r);
  }
  vec3 global_blur = env_spec(r, 1.0);
  if (u_use_env_global_blur_texture != 0) {
    global_blur = sample_env_map(u_env_global_blur_tex, r);
  } else if (u_use_env_global_texture != 0) {
    global_blur = global_sharp;
  }
  vec3 global = mix(global_sharp, global_blur, rough);

  if (u_use_env_local_texture != 0 || u_use_env_local_blur_texture != 0) {
    vec3 local_sharp = global_sharp;
    vec3 local_blur = global_blur;
    if (u_use_env_local_texture != 0) {
      local_sharp = sample_env_map(u_env_local_tex, r);
    }
    if (u_use_env_local_blur_texture != 0) {
      local_blur = sample_env_map(u_env_local_blur_tex, r);
    } else if (u_use_env_local_texture != 0) {
      local_blur = local_sharp;
    }
    vec3 local = mix(local_sharp, local_blur, rough);
    return mix(global, local, clamp(u_env_specular_mix, 0.0, 1.0));
  }
  return global;
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

vec3 pbr_direct_sum(vec3 n, vec3 v, vec3 albedo,
                    float metal, float rough, vec3 f0) {
  vec3 direct = vec3(0.0);
  for (int i = 0; i < MAX_PBR_LIGHTS; ++i) {
    if (i >= u_light_count) {
      break;
    }
    vec3 l = vec3(0.0);
    float attenuation = 1.0;
    if (u_light_pos_type[i].w > 0.5) {
      float dir_len = length(u_light_dir_data[i].xyz);
      if (dir_len > 0.0001) {
        l = u_light_dir_data[i].xyz / dir_len;
      }
    } else {
      vec3 to_light = u_light_pos_type[i].xyz - v_world_pos;
      float dist2 = dot(to_light, to_light);
      if (dist2 > 0.0001) {
        l = to_light * inversesqrt(dist2);
        attenuation = 1.0 / (1.0 + 0.25 * dist2);
      }
    }
    direct += pbr_direct(n, v, l, u_light_color[i].rgb * attenuation,
                         albedo, metal, rough, f0);
  }
  return direct;
}

void main() {
  vec3 albedo = u_albedo;
  float alpha = 1.0;
  if (u_use_texture != 0) {
    vec4 texel = texture2D(u_albedo_tex, v_uv);
    albedo = texel.rgb;
    alpha = texel.a;
    if (alpha < ALPHA_CUTOFF) {
      discard;
    }
  }
  vec3 n = normalize(v_world_norm);
  if (u_use_normal_texture != 0) {
    vec3 t = normalize(v_world_tangent - n * dot(n, v_world_tangent));
    vec3 b = normalize(cross(n, t)) * v_tangent_sign;
    vec3 n_ts = texture2D(u_normal_tex, v_uv).xyz * 2.0 - 1.0;
    n = normalize(mat3(t, b, n) * n_ts);
  }
  vec3 v = normalize(u_eye_pos - v_world_pos);
  float nov = max(dot(n, v), 0.0);
  float rough = clamp(u_roughness, 0.04, 1.0);
  float metal = clamp(u_metallic, 0.0, 1.0);
  float ao = 1.0;
  if (u_use_orm_texture != 0) {
    vec3 orm = texture2D(u_orm_tex, v_uv).rgb;
    ao = clamp(orm.r, 0.0, 1.0);
    rough = clamp(rough * orm.g, 0.04, 1.0);
    metal = clamp(metal * orm.b, 0.0, 1.0);
  }
  vec3 f0 = mix(vec3(0.04), albedo, metal);
  vec3 direct = pbr_direct_sum(n, v, albedo, metal, rough, f0);
  vec3 irr = env_diffuse_ibl(n);
  vec3 refl = reflect(-v, n);
  vec3 se = env_specular_ibl(refl, rough);
  vec3 fresnel_env = f0 + (1.0 - f0) * pow(1.0 - nov, 5.0);
  vec3 kd = (1.0 - metal) * albedo;
  vec3 env_col = kd * irr + fresnel_env * se;
  vec3 col = direct + ao * env_col;
  col = col / (col + 1.0);
  col = sqrt(max(col, 0.0));
  gl_FragColor = vec4(clamp(col, 0.0, 1.0), u_alpha * alpha);
}
