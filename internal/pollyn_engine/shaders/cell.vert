attribute vec3 a_pos;
attribute vec3 a_norm;
attribute vec2 a_uv;
uniform mat4 u_mvp;
uniform mat4 u_model;
varying vec3 v_normal;
varying vec2 v_uv;
void main() {
  gl_Position = u_mvp * vec4(a_pos, 1.0);
  v_normal = mat3(u_model) * a_norm;
  v_uv = a_uv;
}
