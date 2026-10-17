# Repository Guidelines

## Project Structure & Module Organization

All paths are relative to this repository's root. See the repository layout in
[README.md](README.md) for the directory tree.

Application entry points live under `cmd/`: `urc_simulator/` contains the rover
simulator and model viewer; `image_view/` and `vjoy/` contain the image viewer
and virtual joystick. Reusable code lives under `internal/`:
`pollyn_engine/` contains the rendering engine and shaders, `shim/` contains
MuJoCo C bridges, and `bindings/ros3/` contains Pika bindings for ROS3 and Mirage.
The customized physics engine and its dependencies are in `vendor/mujoco/`.

The root `build.ninja` defines application builds and shared settings;
`test/build.ninja` defines test builds and execution. Test sources and fixtures
are in `test/`, bundled models and textures in `models/`, and implementation
notes in `doc/`. CI is defined in `.github/workflows/build.yml`. Generated
executables and intermediate files go in `build/`.

## Build, Test, and Development Commands

- Application and test targets build required shaders, MuJoCo, and bridges first.
- `ninja mujoco` builds only the vendored MuJoCo libraries.
- `ninja` builds the main simulator using installed dependencies.
- `ninja trellis2` builds the main MuJoCo viewer.
- `ninja urc_simulator` builds the robot simulation entry point.
- `ninja image_view` builds the image viewer tool.
- `ninja tests` compiles test groups defined in `test/build.ninja`, loaded by the root file.
- `ninja check` depends on `tests`, then runs automated checks in `test/build.ninja`.
- Run an individual compiled binary directly, e.g. `./build/test_physics`.
- `./build/trellis2 models/ball.xml` runs the main viewer against the sample
  model after building.

Maintain the simple direct recipes in `build.ninja`. Keep shared source lists in
ordinary Ninja variables; avoid generated build graphs and complex target chains.
Use installed Pika/Rose dependencies. Keep setup dependencies on target lines:
MuJoCo is independent of shaders; application and test targets depend directly
on the setup outputs they use. Track actual output files for incremental builds;
keep source dependency groups in sync with the compiler source lists. These
groups use timestamp files so MuJoCo restat cannot hide their changes. Bridge
header dependencies come from compiler depfiles. The Pika rule only compiles and links. The default target is `urc_simulator`.

## Coding Style & Naming Conventions

Follow the existing style in `.pka` files: tabs for indentation, lower_snake_case
functions and variables, uppercase constants, and short module-like prefixes
such as `scene_`, `camera_`, `util_`, and `mj_shim_`. Keep comments focused on
non-obvious behavior. C bridge files use C11 and are compiled with
`-Wall -Wextra -Werror`, so keep warnings clean.

## Testing Guidelines

Add tests under `test/` using the `test_<feature>.pka` naming pattern. Prefer
small executable tests with explicit `assert(...)` checks and clear pass/fail
logging, matching files such as `test/test_physics.pka`. Rendering tests may use
software GL and `xvfb-run` through `test/build.ninja`. Use `ninja tests` then
`ninja check` for automated checks,
or `ninja tests` followed by an individual binary for focused checks.

## Commit & Pull Request Guidelines

Recent commits use concise, imperative, lower-case messages such as
`fix depth shader, add rover robot sim demo`. Keep commits
focused by subsystem. Pull requests should include a short description, affected
targets, tests run, linked issues if applicable, and screenshots or terminal
captures for visual changes to rendering or terminal output.

## Agent-Specific Instructions

Do not edit generated build artifacts under `build/`. Shader embedding is handled
by `internal/pollyn_engine/shaders/embed.py`; edit shader sources instead of the
generated `internal/pollyn_engine/shim/shaders.h`.
