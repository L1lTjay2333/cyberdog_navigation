# Static Reconstruction Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement inline. User selected the existing project directory and approved continuing the previously described independent static prototype; keep the running navigation server intact.

**Goal:** Build and verify a bounded, read-only static point-cloud fusion prototype, with original-stamp TF buffering, a local interactive viewer and PLY export.

**Architecture:** Transient SSH ROS subscriber sends compressed depth, intrinsics and exact-time vodom-to-camera transforms. A separate PC process decodes and integrates centimeter voxels, freezes on movement or invalid pose, and serves port8766. No controller, lifecycle, parameter or navigation calls. Existing navigation remains on8765.

**Tech Stack:** Python3.6-compatible robot capture; Python3.11 and NumPy PC fusion; stdlib HTTP/SSH; local Canvas viewer. No new camera driver, SLAM stack, SDK or firmware install.

**Spec:** User-approved static-first approach in this conversation; docs/3D_PREFLIGHT_20261027.md and docs/3D_REFERENCES_20261027.md provide observed inputs and limitations.

## Constraints and review focus

- Static local geometry only: no room-scale/absolute-accuracy claim, RGB or mesh in this increment.
- Preserve original depth timestamp and exact TF; bounded queue6, expiry2s, no latest-TF fallback.
- Reuse current16UC1-mm/32FC1-m ROS convention and label scale unvalidated; reject other encodings and malformed calibration/layout.
- Bounded depth payload and decompression, positive depths0.15–4m, at most200000 model voxels; 1cm voxel size is not a precision claim.
- Stop fusion beyond3cm/5deg camera-pose change, lost stream, or full capacity. Stop never sends robot movement or task cancellation.
- Reject duplicate/older frame stamps, coordinate-frame changes, bad quaternion and malformed data before model mutation.
- Local-only HTTP; no file-path input, finite request bodies, local Origin required for capture controls.
- Raw data remains in ignored evidence; code, tests, plan and usage go toGit.

## Tasks

- [x] 1. Write failing core tests for image encoding/endian/padding, metric projection and full-pose transform, duplicate timestamps, pose changes, voxel averaging and memory bounds; implement reconstruction_core.py and rerun.
- [x] 2. Test original-stamp bounded queue and timeout behavior; implement reconstruction_capture.py without publishers/services. Verify robot3.6 syntax and no control entrypoints.
- [x] 3. Add standalone reconstruction_server.py and reconstruction.html: start/stop read-only30s capture, live model/stats, rotation, zoom, PLY export. Do not restart or attach control to8765.
- [x] 4. Run focused tests and existing offline suite; start8766, perform bounded static captures after implementation changes, inspect metrics and exported PLY. Three non-motion captures completed (30/15/8 seconds). Full suite has6 baseline failures, not introduced here.
- [ ] 5. Record actual results and limitations, commit scoped changes, push if connection permits; do not claim GitHub sync without confirmation.

Review: two important abnormal-input/setup findings were reproduced and fixed RED→GREEN; independent re-review confirmed18 focused tests passing and no remaining important issue from those fixes. No motion or robot access by reviewer.
