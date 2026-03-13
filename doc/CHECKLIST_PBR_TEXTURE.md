# PBR Texture Support — Checklist

## Goal
Add optional albedo texture sampling to PBR and flat shaders. Extend vertex
format to include UV coordinates. Enable per-material texture binding.

## Implementation Steps

- [x] **Step 1**: Extend vertex format in `pollyn_engine/src/mesh.pka`
  - VERT_STRIDE 6→8, MAX_FLOATS 24576→32768
  - mesh_add_vert gains (u, v) parameters
  - All mesh_create_* emit UVs (plane/box get proper UVs, sphere/cylinder get cylindrical UVs)

- [x] **Step 2**: Update C shim vertex handling in `pollyn_engine/shim/pollyn_shim.c`
  - All draw functions: stride 24→32, add a_uv attrib pointer at offset 24
  - STL loader: write 8 floats per vertex (u=0, v=0 appended)
  - Cache a_uv locations for pbr, flat, depth shaders
  - Add `shim_gl_set_albedo_texture(tex_handle)` function

- [x] **Step 3**: Update shaders
  - `pbr.vert/frag`: add a_uv → v_uv, u_albedo_tex sampler, u_use_texture flag
  - `flat.vert/frag`: same pattern
  - `depth.vert`: add a_uv attribute (unused but must match stride)

- [x] **Step 4**: Update Pika declarations and Material type
  - `renderer.pka`: add @symbol for shim_gl_set_albedo_texture
  - `material.pka`: add `texture: u32` field to Material
  - `core_engine.pka`: call set_albedo_texture in geom draw loop

- [x] **Step 5**: Update pollyn_engine test (test_mesh.pka)
  - Fix hardcoded `* 6` offsets to use `* 8`
  - Update test_mesh_from_buffers data (24→32 floats)

- [x] **Step 6**: Create `pollyn_engine/test/test_pbr_texture.pka`
  - Generate procedural checkerboard texture (red/green 8x8)
  - Test 1: PBR without texture (solid red albedo)
  - Test 2: PBR with checkerboard texture (red+green visible)
  - Test 3: Flat shading with checkerboard texture (red+green visible)

- [x] **Step 7**: Verify all existing tests still pass
  - pollyn_engine: test_math3d, test_mesh, test_engine, test_bunny_pbr, test_depth, test_pbr_texture
  - trellis2: test_physics, test_render, test_cv, test_depth, test_texture2d
