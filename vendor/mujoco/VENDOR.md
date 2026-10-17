# Vendored MuJoCo Description

Copied from `trellis/vendor/README.md`. The trimming notes below describe the
original vendoring work; build instructions have been updated for Trellis2.

## MuJoCo Physics Engine

**Location**: `vendor/mujoco/` (relative to the Trellis2 root)
**Version**: Vendored from upstream (stripped for size)
**Size**: ~34MB (source only, no build artifacts)

### What's Included

- `src/` - MuJoCo core source code
- `include/` - Public headers
- `model/` - Example models for testing
- `dependencies/` - Minimal required dependencies:
  - ccd - Collision detection
  - eigen3 - Linear algebra (test/bench kept for version comparison)
  - lodepng - PNG loading
  - qhull - Convex hull (docs/examples kept)
  - tinyobjloader - OBJ mesh loading
  - tinyxml2 - XML parsing

### What's Removed

To reduce size from 584MB to ~34MB (94% reduction):

**Removed directories**:
- `build/` - Build artifacts (regenerated on first build)
- `test/` - Testing framework
- `sample/` - Sample programs
- `simulate/` - MuJoCo's own GUI (we use pollyn engine)
- `unity/` - Unity plugin
- `python/` - Python bindings
- `wasm_test/` - WASM tests
- `doc/` - Documentation
- `example/` - Example code
- `introspect/` - Introspection utilities
- `.git/` - Git history

**Removed models** (kept essential demos):
- `mug/` - Large mesh (2.2MB)
- `composite/` - Large textures (2.1MB)
- Kept: humanoid, humanoid100, cube, balloons, flag, hammock, etc.

**Removed dependencies**:
- `abseil` - Only used for tests (14MB saved)
- `glfw3` - We use pollyn engine's GLFW
- `gtest` - Testing framework
- `benchmark` - Benchmarking framework

**Cleaned dependency content**:
- `eigen3/doc/` - Documentation (2MB saved)
- `tinyxml2/docs/` and `tinyxml2/resources/` - Documentation and images (2.8MB saved)
- `tinyobjloader/images/` and `tinyobjloader/models/` - Example assets (600KB saved)
- Qhull generated build artifacts were removed; source project files under
  `dependencies/qhull/build/` are retained.
- Build directories in all dependencies (added to .gitignore)

### Building

From the Trellis2 root, build the bundled source with Ninja:

```sh
ninja -C vendor/mujoco mujoco
ninja all
```

Generated static libraries are written to `vendor/mujoco/build/lib/` and are
ignored by Git. No separate MuJoCo download or sibling Trellis checkout is needed.

### Modifications

The original Trellis vendoring notes described CMake patches disabling sample,
test, and simulate subdirectories. This snapshot uses the bundled `build.ninja`
instead; those historical CMake instructions do not apply to the Trellis2 build.
See [TRELLIS2.md](TRELLIS2.md) for copy provenance and the current build layout.

### Upstream

Original source: https://github.com/google-deepmind/mujoco

To update to a newer version:
1. Copy fresh MuJoCo source
2. Remove the directories listed above
3. Update the bundled Ninja build for any source/dependency changes
4. Rebuild and check compatibility with Trellis2
