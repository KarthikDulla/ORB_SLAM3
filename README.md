# ORB-SLAM3 — Intel RealSense D435i Port (Ubuntu 24.04)

This repository is a **modified build of [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)**
by Campos, Elvira, Rodriguez, Montiel & Tardos (original paper: *ORB-SLAM3: An Accurate Open-Source
Library for Visual, Visual-Inertial and Multi-Map SLAM*). All core SLAM algorithms (tracking, local
mapping, loop closing, DBoW2, g2o) are the original authors' work and are unchanged.

This fork exists because the original build does **not** work out of the box on Ubuntu 24.04 with a
D435i on a laptop with a mixed USB 2.1/3.0 setup. I ported it, fixed the build, and adapted it to
hardware I actually had available (a webcam that failed, then a D435i limited to USB 2.1 bandwidth).

**License:** GPLv3, inherited from the original project. See `LICENSE`.

---

## What I changed and why

| Area | Original | What I did | Why |
|---|---|---|---|
| `CMakeLists.txt` | `cmake_minimum_required(2.8)`, C++11 only | Bumped to CMake 3.5, added C++14 fallback | CMake 2.8 is deprecated on modern distros; OpenCV 4.6 needs some C++14-only APIs |
| OpenCV compatibility | Written for OpenCV 4.2 | Replaced removed macros (`CV_LOAD_IMAGE_*` to `cv::IMREAD_*`), fixed a broken `cv::Matx` reduction in `LocalMapping.cc` | Ubuntu 24.04 ships OpenCV 4.6, which removed/renamed several APIs the original code depended on |
| Missing headers | -- | Added `#include <unistd.h>` to `LocalMapping.cc`, `LoopClosing.cc`, `System.cc`, `Tracking.cc`, `Viewer.cc` | `usleep()` calls failed to compile without it under GCC 13 |
| Pangolin | Assumes clean install | Documented full clean-reinstall procedure for Pangolin v0.6 | A corrupted cmake config (`PangolinTargets-release.cmake`) is a common, undocumented failure mode |
| Camera input | Assumes RealSense SDK is `apt`-installable | Built `librealsense2` v2.55.1 from source, added udev rules manually | No prebuilt SDK packages exist for Ubuntu 24.04 (Noble) yet |
| `Examples/Monocular/mono_realsense_D435i.cc` | Does not exist upstream | New file -- monocular capture from D435i's RGB stream, with `SIGINT` handler so `Ctrl+C` shuts down cleanly and still saves the trajectory | The default examples don't support D435i in monocular mode, and none of the examples handle interrupted sessions gracefully |
| Stereo-Inertial config | `RealSense_D435i.yaml` ships with a template `IMU.Tbc` | Diagnosed and fixed a malformed `Tbc` matrix that was causing a hard crash (`-3.2e+34` garbage values) | Wrong IMU-to-camera extrinsics crash the inertial initializer instead of failing gracefully |
| Camera intrinsics | Placeholder values (`fx=600, cx=320...`) | Replaced with actual calibrated D435i intrinsics (`fx=607.855, fy=607.681, cx=319.641, cy=248.406`) | ORB-SLAM3 is highly sensitive to intrinsics; placeholders produce a distorted, drifting map |
| Bandwidth handling | Assumes USB 3.0 | Diagnosed a USB 2.1 bandwidth ceiling that made RGB-D mode fail with `Frame didn't arrive within 15000`; fell back to monocular-only capture | 640x480 Color + 640x480 Depth @ 30fps exceeds USB 2.1 throughput; this isn't documented anywhere in the original repo |
| `scripts/plot_trajectory.py` | Does not exist upstream | New file -- loads `KeyFrameTrajectory.txt` (TUM format) and plots the 3D camera path | For quick visual sanity-checking of a run without external tools |

---

## Hardware / environment this was built against

- Ubuntu 24.04 (Noble)
- Intel RealSense D435i, connected via USB 2.1 (not 3.0 -- see bandwidth note above)
- OpenCV 4.6.0 (system package)
- Pangolin v0.6 (built from source)
- librealsense2 v2.55.1 (built from source)
- GCC 13.3.0

## Known limitations of this fork

- Only monocular mode is verified working on USB 2.1. RGB-D and Stereo-Inertial modes require USB 3.0
  bandwidth and are untested on this hardware.
- Monocular SLAM has no absolute scale (standard limitation of monocular visual SLAM, not specific to
  this fork).
- The `IMU.Tbc` fix uses generic D435i factory extrinsics, not a per-unit calibration. For rigorous
  visual-inertial work, run Kalibr or a similar calibration tool.

## Build instructions

See `SETUP.md` for the full sequence (Pangolin -> librealsense2 -> ORB-SLAM3).

## Running it

\`\`\`bash
./Examples/Monocular/mono_realsense_D435i \
    Vocabulary/ORBvoc.txt \
    Examples/Monocular/RealSense_D435i.yaml
\`\`\`

`Ctrl+C` in the terminal stops cleanly and saves `KeyFrameTrajectory.txt`.

## Acknowledgments

- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) -- Campos et al., UZ-SLAMLab
- [librealsense](https://github.com/IntelRealSense/librealsense) -- Intel
- [Pangolin](https://github.com/stevenlovegrove/Pangolin) -- Steven Lovegrove
