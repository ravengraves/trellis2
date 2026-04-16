attribute vec3 a_pos;
attribute vec3 a_norm;
attribute vec2 a_uv;
attribute vec4 a_tangent;
uniform mat4 u_mvp;
uniform mat4 u_model;
varying vec3 v_world_pos;
varying vec3 v_world_norm;
varying vec2 v_uv;
varying vec3 v_world_tangent;
varying float v_tangent_sign;
void main() {
  vec4 wp = u_model * vec4(a_pos, 1.0);
  v_world_pos = wp.xyz;
  v_world_norm = mat3(u_model) * a_norm;
  v_uv = a_uv;
  v_world_tangent = mat3(u_model) * a_tangent.xyz;
  v_tangent_sign = a_tangent.w;
  gl_Position = u_mvp * vec4(a_pos, 1.0);
}
