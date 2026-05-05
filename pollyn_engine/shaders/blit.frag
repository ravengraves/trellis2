precision highp float;
varying vec2 v_uv;
uniform sampler2D u_tex;
uniform int u_tiled;

void main() {
  vec2 uv = v_uv;
  if (u_tiled != 0) {
    uv = fract(v_uv);
  }
  gl_FragColor = texture2D(u_tex, uv);
}
