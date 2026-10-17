# Trellis2 Implementation Checklist

- [x] Directory structure (`trellis2/`)
- [x] MuJoCo C shim (`shim/mujoco_shim.c`)
- [x] Pika MuJoCo bindings (`cmd/urc_simulator/mujoco.pka`)
- [x] Scene module (`cmd/urc_simulator/scene.pka`)
- [x] Main viewer (`cmd/urc_simulator/viewer_main.pka`)
- [x] Physics test (`test/test_physics.pka`)
- [x] Render test (`test/test_render.pka`)
- [x] Build system (`build.ninja`)
- [x] Test model (`models/ball.xml`)
- [x] Build and verify all targets
  - `make check` — both tests pass
  - `ninja trellis2` — full viewer builds
