# Trellis2 Robot Simulator Checklist

- [x] Use `urc_simulator` as the main/default target; retain `trellis2` as the simple viewer.
- [x] Freeze MuJoCo reads behind a per-frame snapshot API so rendering can read a consistent world state while physics runs elsewhere.
- [x] Add body-pose accessors needed for robot-mounted camera sensors.
- [ ] Move MuJoCo stepping onto a dedicated physics thread.
- [ ] Read ROS3 tank-control inputs on the physics side and map them into MuJoCo actuators.
- [ ] Publish renderer RGB frames to ROS3 numericarray topics.
- [ ] Publish renderer depth frames to ROS3 numericarray topics.
- [ ] Mount a forward camera on the rover body and render it every frame.
- [ ] Add a `d` hotkey to toggle the camera overlay between RGB and depth.
- [ ] Add focused build verification for the new robot simulator target.

## Bundled model and historical first-milestone settings

- Model: `models/fruits_environment.xml`
- Control topic: `/control` with two `f64` values: left and right wheel targets in `[0, 1]`
- Left actuator: `rear_left_wheel`
- Right actuator: `rear_right_wheel`
- RGB topic: `/demo/camera/front`
- Depth topic: `/demo/camera/depth`
- Sensor body: `frame`
