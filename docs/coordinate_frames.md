# Coordinate frames and timing

The camera frame is `+x` image-right, `+y` image-down, `+z` lens-forward. The
local world frame is created by the lower computer's stationary boot
calibration; gravity fixes roll/pitch and the boot heading defines yaw zero.

V4 telemetry sends the quaternion that rotates the camera rigid-body axes into
that local world frame. `control.yaml:body_from_camera` is the calibrated
camera-to-body rotation. For ideal mechanical alignment it is the identity.
The upper computer builds a line-of-sight vector from camera yaw/pitch, applies
this extrinsic, rotates it with the exposure-time quaternion, and then computes
absolute world yaw/pitch.

Three clocks are joined as:

```text
Orbbec hardware clock -> host steady_clock -> STM32 TIM2 microseconds
```

The camera map is fitted continuously and must collect eight stable samples.
The STM32 map uses 10 Hz four-timestamp exchanges, low-RTT selection, robust
drift fitting, and TIM2 wrap unrolling. Precise mode accepts a visual update
only when an exposure midpoint, a camera-map residual within the configured
limit, calibrated IMU data, and a clock uncertainty no greater than 0.5 ms are
all available. Quaternion history uses SLERP and permits at most 2 ms gyro
extrapolation.

The current STM32 clock tree uses the internal HSI RC oscillator, so a stable
TIM2 drift of several thousand ppm is expected. The affine map compensates up
to +/-2% drift; acceptance depends on post-fit residual uncertainty, not raw
oscillator frequency error.
