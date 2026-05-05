# Repository Guidelines

## Project Structure & Module Organization

This repository is a Pika/C simulator and visualization project built with Ninja.
Core application code lives in `src/`, with executable-specific code under
`cmd/` (`cmd/roseviz/`, `cmd/image_view/`). The vendored/rendering engine is in
`pollyn_engine/`, split into Pika modules in `pollyn_engine/src/`, higher-level
engine helpers in `pollyn_engine/engine/`, C shims in `pollyn_engine/shim/`, and
GLSL shaders in `pollyn_engine/shaders/`. MuJoCo bridge code is in `shim/`.
Tests are Pika files in `test/`, models are in `models/`, and project notes or
checklists are in `doc/`.

## Build, Test, and Development Commands

- `ninja trellis2` builds the main MuJoCo viewer.
- `ninja robot_sim` builds the robot simulation entry point.
- `ninja roseviz` builds the Rose/ROS3 visualization UI.
- `ninja image_view` builds the image viewer tool.
- `ninja check` builds and runs the full test suite.
- `ninja test_physics` runs one focused test target; replace with any target from
  `ninja -t targets`.
- `./build/trellis2 models/ball.xml` runs the main viewer against the sample
  model after building.

The build file assumes local sibling checkouts for Pika, Rose, and MuJoCo paths;
update `build.ninja` only when those dependency locations or target definitions
intentionally change.

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
software GL and `xvfb-run` through existing Ninja rules. Run `ninja check`
before broad changes, or the closest `ninja test_<feature>` target for narrow
edits.

## Commit & Pull Request Guidelines

Recent commits use concise, imperative, lower-case messages such as
`add roseviz` and `fix depth shader, add rover robot sim demo`. Keep commits
focused by subsystem. Pull requests should include a short description, affected
targets, tests run, linked issues if applicable, and screenshots or terminal
captures for visual changes to `roseviz`, rendering, or terminal output.

## Agent-Specific Instructions

Do not edit generated build artifacts under `build/`. Preserve the checked-in
Ninja workflow unless asked to migrate it. When touching shaders, keep
`pollyn_engine/shaders/embed.py` and `pollyn_engine/shim/shaders.h` generation
in mind because shader embedding is part of the compile rules.
