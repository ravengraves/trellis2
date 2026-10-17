# Trellis2 vendoring notes

See [VENDOR.md](VENDOR.md) for the description copied from Trellis's
`vendor/README.md`, including included/removed components and trimming history.

Copied on 2026-10-17 from `simulator/trellis/vendor/mujoco`, at Trellis
commit `0c6eae2d7b5cfe7d127f7add3ba0df5c82b1fd9a` (source repository:
`https://git.sr.ht/~alecgraves/trellis`). The source subtree had no tracked
modifications when copied.

This is an in-repository source copy, not a submodule. It includes the original
README, licenses, bundled dependencies, build definitions, and model assets.
Generated build outputs and Ninja state were not copied. Keep upstream and
third-party license files with the source when redistributing it.

From the Trellis2 root:

```sh
ninja -C vendor/mujoco mujoco
ninja all
```

The Ninja build writes static archives under `vendor/mujoco/build/lib/`.
Those generated files are ignored by Git. The original README describes the
fork and upstream project; the commands above are the Trellis2 build path.
Pika and Rose remain separately installed dependencies.
