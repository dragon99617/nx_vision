# nx_vision

Orin NX vision stack for a self-aiming gimbal task. The project captures RGB
frames, detects the target, estimates pose with PnP, converts the exposure-time
line of sight into a local world frame, and publishes a time-triggered 1 kHz
reference to the STM32 gimbal controller.

The first implementation is OpenCV based. The camera layer is isolated so native
Orbbec SDK depth capture can be added later without rewriting the PnP, task,
serial, pipeline, or debug-view layers.

## Features

- OpenCV RGB capture and processing pipeline.
- Target rectangle detection, preprocessing, tracking, and laser detection.
- PnP pose solving and laser-origin compensation.
- Multiple task modes for the basic and advanced requirements.
- V4 CRC32C binary transport, four-timestamp clock synchronization, and 1 kHz
  attitude history.
- World-target Kalman filtering and constrained 1 kHz MPC reference generation.
- Debug Studio views for raw frames, masks, contours, pose, and serial output.
- YAML-based configuration for camera, target geometry, serial, tasks, and debug
  layout.

## Repository Layout

```text
apps/        Executable entry points for runtime, debug, replay, calibration, and tasks.
config/      YAML configuration files used by the applications.
data/        Runtime data notes and expected data directories.
docs/        Integration, protocol, calibration, debug, and coordinate-frame notes.
include/     Public C++ headers.
scripts/     Jetson dependency installation, build, run, and serial setup helpers.
src/         Core camera, vision, pose, task, pipeline, communication, and debug code.
tests/       Lightweight C++ tests.
tools/       Developer support notes and utilities.
```

## Requirements

- CMake 3.16 or newer.
- C++17 compiler.
- OpenCV development packages with `core`, `imgproc`, `highgui`, `calib3d`,
  `videoio`, and `imgcodecs`.
- POSIX shell environment for the helper scripts.
- Orbbec SDK v2 is optional for future native depth capture.

On Jetson, install the base dependencies with:

```bash
./scripts/install_deps_jetson.sh
```

## Build

```bash
./scripts/build_release.sh
```

Equivalent manual command:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```

## Run

Start the interactive debug view:

```bash
./scripts/run_debug_studio.sh
```

Start the runtime pipeline:

```bash
./scripts/run_runtime.sh
```

Both scripts pass `config/` to the executable. Update the YAML files in
`config/` before running on new hardware.

## Tests

Tests are enabled by default through `NXVISION_BUILD_TESTS`.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

## Gimbal Protocol

The default is the time-triggered binary V4 protocol. It sends 42-byte planned
world-angle points at 1 kHz and receives 64-byte timestamped IMU states at
1 kHz. V2 and `legacy_a` are retained as explicit debug modes. See
`docs/protocol.md` for the exact layout and safety behaviour.

## Documentation

- `docs/protocol.md`: serial packet format and gimbal integration notes.
- `docs/yuntai_integration.md`: gimbal-side integration checklist.
- `docs/calibration.md`: camera and target calibration notes.
- `docs/coordinate_frames.md`: coordinate-frame conventions.
- `docs/debug_studio.md`: Debug Studio panels and shortcuts.
- `docs/topic_summary.md`: task-to-requirement mapping.
