attribute vec3 a_pos;
attribute vec2 a_uv;
uniform mat4 u_mvp;
uniform float u_znear;
uniform float u_zfar;
varying float v_depth;
void main() {
  gl_Position = u_mvp * vec4(a_pos, 1.0);
  float linear = (gl_Position.w - u_znear) / (u_zfar - u_znear);
  v_depth = clamp(linear, 0.0, 1.0);
}
