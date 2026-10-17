# Renderer dependency

`stb_image.h` is the existing stb_image v2.12 header previously used from
`trellis/vendor/mujoco/dependencies/tinyobjloader/examples/viewer/`. It is kept
with Pollyn so PNG loading does not depend on that unrelated checkout layout.
The original license notice is preserved in the header.
