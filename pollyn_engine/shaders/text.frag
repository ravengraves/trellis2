precision highp float;
varying vec2 v_uv;
uniform sampler2D u_font;
uniform vec3 u_color;

void main() {
  float a = texture2D(u_font, v_uv).r;
  if (a < 0.01) {
    discard;
  }
  gl_FragColor = vec4(u_color, a);
}
