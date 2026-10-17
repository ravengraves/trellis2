# 2D Texture / Camera Feed Visualization — Checklist

## Goal
Support rendering arbitrary camera feeds (RGB/depth data) as 2D textured quads
using the pollyn engine's GPU pipeline. This enables live visualization of
depth sensor output, camera feeds, and other image data overlaid on the 3D scene.

## Implementation Steps

- [x] **Step 1**: Add texture shim functions to `internal/pollyn_engine/shim/pollyn_shim.c`
  - `shim_gl_create_texture_rgb(data, w, h)` → GLuint
  - `shim_gl_update_texture_rgb(tex, data, w, h)`
  - `shim_gl_delete_texture(tex)`
  - `shim_gl_init_blit_shader(vs, fs)` — compile blit shader + VBO
  - `shim_gl_draw_textured_quad(tex, x, y, w, h, screen_w, screen_h)`

- [x] **Step 2**: Add `@symbol` declarations in `internal/pollyn_engine/src/renderer.pka`

- [x] **Step 3**: Add `E2GpuTexture` type and pass to `internal/pollyn_engine/engine/core_engine.pka`
  - `E2GpuTexture` struct (handle, width, height, valid)
  - `e2_gpu_texture_create_rgb(data, w, h) E2GpuTexture`
  - `e2_gpu_texture_update_rgb(tex, data)`
  - `e2_gpu_texture_destroy(tex)`
  - `e2_init_blit_shader() bool`
  - `e2_pass_2d_texture(rt, tex, x, y, w, h)` — render textured quad

- [x] **Step 4**: Create blit shader pair `internal/pollyn_engine/shaders/blit.vert` + `blit.frag`

- [x] **Step 5**: Create `trellis2/cmd/urc_simulator/camera_view.pka` module
  - `CameraView` struct wrapping a DepthSensor + GPU texture
  - `camera_view_new(...)` — create sensor + configure screen placement
  - `camera_view_render(...)` — render scene to sensor, upload RGB/depth to texture
  - `camera_view_draw(...)` — draw texture as 2D overlay at screen position
  - `depth_to_rgb(...)` — depth-to-grayscale conversion for depth visualization
  - `camera_view_set_pose(...)` — update camera pose
  - `camera_view_destroy(...)` — cleanup GPU resources

- [x] **Step 6**: Create `trellis2/test/test_texture2d.pka`
  - Test 1: Texture create from synthetic RGB pattern
  - Test 2: Textured quad rendering (red/blue checkerboard visible)
  - Test 3: Texture update (green solid replaces checkerboard)
  - Test 4: CameraView RGB mode (rendered scene visible as overlay)
  - Test 5: CameraView depth mode (depth grayscale visible)

- [x] **Step 7**: Update `trellis2/build.ninja` with new targets
  - test_texture2d build + run targets
  - camera_view.pka added to main viewer sources
  - blit.vert/blit.frag added as shader dependencies
