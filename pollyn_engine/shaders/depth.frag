precision highp float;
varying float v_depth;
void main() {
  vec4 enc = vec4(1.0, 255.0, 65025.0, 16581375.0) * v_depth;
  enc = fract(enc);
  enc -= enc.yzww * vec4(1.0/255.0, 1.0/255.0, 1.0/255.0, 0.0);
  gl_FragColor = enc;
}
