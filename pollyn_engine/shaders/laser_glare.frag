precision highp float;

varying vec2 v_uv;
uniform sampler2D u_tex;
uniform vec2 u_screen;
uniform vec2 u_center;
uniform float u_radius;
uniform float u_decay;
uniform float u_hot_radius;
uniform float u_hot_strength;
uniform float u_glare_strength;
uniform float u_streak_strength;

void main() {
  vec3 src = texture2D(u_tex, v_uv).rgb;
  float radius = clamp(u_radius, 0.0, 1.0);
  if (radius >= 0.999) {
    gl_FragColor = vec4(1.0, 1.0, 1.0, 1.0);
    return;
  }

  vec2 center = clamp(u_center, 0.0, 1.0);
  vec2 delta_px = (v_uv - center) * u_screen;
  float d = length(delta_px);
  float radius_px = max(1.0, radius * 0.5 * length(u_screen));
  float decay = max(u_decay, 0.001);

  float norm_d = clamp(d / radius_px, 0.0, 1.0);
  float glare = pow(1.0 - norm_d, decay) * max(u_glare_strength, 0.0);

  float hot_px = max(1.0, radius_px * clamp(u_hot_radius, 0.0, 1.0));
  float hot = smoothstep(hot_px, 0.0, d) * max(u_hot_strength, 0.0);

  float x_dist = abs(delta_px.x);
  float y_dist = abs(delta_px.y);
  float axis_width = max(1.0, radius_px * 0.035);
  float axis_fade_x = pow(1.0 - clamp(x_dist / radius_px, 0.0, 1.0), decay * 0.55);
  float axis_fade_y = pow(1.0 - clamp(y_dist / radius_px, 0.0, 1.0), decay * 0.55);
  float h_streak = exp(-y_dist / axis_width) * axis_fade_x;
  float v_streak = exp(-x_dist / axis_width) * axis_fade_y;

  float diag_a = abs(delta_px.x - delta_px.y);
  float diag_b = abs(delta_px.x + delta_px.y);
  float diag_fade = pow(1.0 - norm_d, decay * 0.75);
  float diag_streak = (exp(-diag_a / (axis_width * 1.6)) +
                       exp(-diag_b / (axis_width * 1.6))) * 0.35 * diag_fade;

  float streak = (h_streak + v_streak + diag_streak) * max(u_streak_strength, 0.0);
  float intensity = clamp(max(hot, glare + streak), 0.0, 1.0);
  gl_FragColor = vec4(mix(src, vec3(1.0), intensity), 1.0);
}
