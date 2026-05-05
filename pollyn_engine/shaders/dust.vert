attribute vec2 a_pos;
attribute vec2 a_uv;
uniform vec2 u_screen;
varying vec2 v_uv;

void main() {
  vec2 ndc;
  ndc.x = (a_pos.x / u_screen.x) * 2.0 - 1.0;
  ndc.y = 1.0 - (a_pos.y / u_screen.y) * 2.0;
  gl_Position = vec4(ndc, 0.0, 1.0);
  v_uv = a_uv;
}
