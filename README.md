# Trellis2 Simulator

Trellis2 is a Pika/GLESv2 rover simulation workspace. The main simulator is
`src/robot_main.pka`; reusable rendering, mesh, texture, and postprocess code
lives under `pollyn_engine/`; integration and rendering tests are under `test/`.

## Build and Test

Use Ninja from the repository root:

```sh
ninja robot_sim
ninja test_robot_motion
ninja test_texture2d
ninja check
```

`ninja robot_sim` builds the rover simulator. `ninja test_robot_motion` checks
the rover motion sampling path. `ninja test_texture2d` builds the texture test
binary, including the PNG texture loading path. `ninja check` runs the full test
suite when a headless GL/Xvfb environment is available.

Run the rover simulator with:

```sh
./build/robot_sim --render-fps 30 --physics-hz 1000
```

Rendering is capped by simulation time, defaulting to 30 FPS. Physics uses Rose
wall-clock pacing and the MuJoCo timestep; `--physics-hz` sets that timestep so
the simulator can step at 1000 Hz while rendering less frequently.

## ROS3 Motion Topics

`robot_sim` publishes motion data independently of rendering:

```text
/simulator/imu  F64[7]  [time, wx, wy, wz, ax, ay, az] at physics Hz
/simulator/vio  F64[7]  [time, x, y, z, vx, vy, vz] at 30 Hz
```

IMU angular rate and acceleration are body-frame values from MuJoCo sensors when
`imu_gyro` and `imu_accel` exist, with a frame-body fallback for older rover
XML. VIO position and velocity are in the MuJoCo world frame.

## Rover Static PBR Props

`robot_sim` has two optional static PBR prop slots. Place assets at:

```text
assets/rover_scene/rock_a.obj
assets/rover_scene/rock_a_albedo.png
assets/rover_scene/rock_a_normal.png
assets/rover_scene/rock_a_orm.png
assets/rover_scene/prop_b.obj
assets/rover_scene/prop_b_albedo.png
assets/rover_scene/prop_b_normal.png
assets/rover_scene/prop_b_orm.png
```

Missing or invalid props are skipped. The OBJ mesh is uploaded once at simulator
startup and reused by both the main view and rover camera render pass.

## Asset Export Requirements

Export static props from Blender as Wavefront OBJ with UVs and normals. The
loader intentionally supports only `v`, `vt`, `vn`, and `f` records and
triangulates polygon faces as a fan. Keep meshes modest: the engine mesh buffer
is capped at 4096 vertices.

Texture filenames are convention-based. Use RGBA for `*_albedo.png`, RGB for
`*_normal.png`, and RGB for `*_orm.png` where channels represent occlusion,
roughness, and metallic. PNG upload is exposed as
`e2_gpu_texture_load_png_rgb(...)` and `e2_gpu_texture_load_png_rgba(...)`; the
current shim implementation uses stb-image and flips PNGs vertically for the
engine's OpenGL UV convention.
