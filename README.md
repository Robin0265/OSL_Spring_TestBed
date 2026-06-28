# OSL_Spring_TestBed

## Pixi environment

For Windows development, use Pixi instead of the old `uv` environment:

```bash
pixi run python -s --version
pixi run python -s test_calibration_plotter.py
```

This workspace is pinned to Python 3.12 in `pixi.toml`, and `-s` is important on this machine because the Windows user site-packages contain a separate NumPy install that can override the Pixi environment if user-site imports are left enabled.

## Calibration pipeline notes

The camera calibration path has been updated to make the optical angle estimate more stable before it is used as an encoder replacement.

Key changes:

- Fiducial detection in `Vision/video_reading_test.py` now uses blob isolation, contour filtering, mask-edge quality checks, and tracker prediction instead of relying on raw Hough-circle detections.
- Camera angle reconstruction now uses a continuity-based periodic unwrap. The fiducial layout repeats every `pi/4`, so the modulo angle can jump from about `45 deg` to `0 deg`; the pipeline now chooses the equivalent angle closest to the previous absolute angle instead of using a threshold-based rotation counter.
- `test_calibration_plotter.py` builds direction-aware inverse maps, with separate positive and negative motion branches.
- The inverse map now uses direct unwrapped lookup tables, `camera_angle_unwrapped -> encoder_angle`, rather than a wrapped `2*pi` residual correction. This avoids averaging different forward cycles into the same phase bin.
- A bounded residual timing correction refines camera/encoder alignment after a preliminary inverse map is built.
- A branch-gap time-shift sweep now checks whether a small camera/encoder timing offset reduces the low-angle forward/reverse branch separation. Accepted shifts are saved in `cal_folder/inverse_time_shift_metadata.json`.
- The active calibration path uses only absolute blue and red inverse maps. Spring deflection is computed later as the difference between calibrated blue/red side angles; the older direct deflection inverse map is no longer generated or used.
- SVD-regularized blue/red calibration models are still generated as backup diagnostics, but their visual curves are commented out because the local lookup map has been more accurate on the current datasets.
- Figure 6 reports inverse-map residuals in degrees and prints branch-specific statistics.
- `test_testbed_plotter.py` now loads the direction-aware `.npz` inverse models instead of treating `inv_blue.csv` and `inv_red.csv` as two-column lookup files.

### Camera measurement flow

The raw camera measurement is produced by `Vision/video_reading_test.py`. The output is:

```text
blue_cam_angs, red_cam_angs, cam_time
```

During calibration, these are saved to:

```text
cal_folder/camera_enabled_angles.csv
```

The video measurement path is:

```text
video frame
-> crop frame
-> apply inner/outer masks
-> isolate bright white fiducial blobs
-> fit blob centers/radii
-> associate blobs with persistent trackers
-> convert tracker centers to per-fiducial angular coordinates
-> combine fiducial angles into one ring angle
-> unwrap the repeated pi/4 sector angle into a continuous absolute camera angle
```

Fiducial isolation uses thresholding plus morphology:

```text
mask
median blur
bright threshold
morphological open
morphological close
contour extraction
blob validation
```

Blob validation rejects detections with invalid area, radius, aspect ratio, fill ratio, mask-edge visibility, or mask-edge clearance. This is intended to keep the white fiducial balls while rejecting lighting edges and partial edge artifacts.

Each valid blob is passed to a `CircleTracker`. A tracker stores the current center/radius, a simple velocity prediction, last-update frame, and the fitted calibration transform. If a tracker is briefly missed, the prediction keeps it alive for a few frames; if it is stale too long, it is dropped.

Each tracker converts its image location into a local angular coordinate:

```python
xi = SinvUT @ ([a, b] - x0)
theta = atan2(-xi[0], xi[1])
```

Because the fiducials repeat every `pi/4`, each tracker angle is first reduced modulo `pi/4`. The current aggregate ring angle uses a weighted circular mean of those modulo angles. It then reconstructs the absolute angle by choosing the modulo-equivalent candidate closest to the previous absolute angle.

For example:

```text
previous absolute angle: 179.8 deg
new modulo angle:          0.3 deg
candidates:              135.3, 180.3, 225.3 deg
selected angle:          180.3 deg
```

This continuity-based unwrap replaced the previous threshold-based rotation counter. The old threshold logic produced red camera spikes when the modulo angle crossed between about `45 deg` and `0 deg`.

### Camera diagnostics

When camera angles are regenerated during calibration, the pipeline writes:

```text
cal_folder/red_camera_diagnostics.csv
cal_folder/blue_camera_diagnostics.csv
```

These files contain one row per contributing tracker per frame. Important columns include:

```text
frame
aggregate_angle
prev_angle_before
raw_avg_mod_angle
n_rotations
jump_flag
n_updated
tracker_id
tracker_theta
tracker_mod_angle
tracker_weight
tracker_a, tracker_b, tracker_r
```

These diagnostics are useful for checking whether a spike came from raw tracker geometry, modulo-sector unwrapping, missing trackers, or later calibration logic.

### Calibration flow

`test_calibration_plotter.py` maps optical camera angles onto encoder-equivalent angles.

The calibration path is:

```text
load or regenerate camera_enabled_angles.csv
load encoder angles from the actuator log
align camera time to encoder time
optionally refine timing with residual-based correction
run a branch-gap time-shift sweep for blue/red camera channels
resample camera angles onto encoder timestamps
split samples by motion direction
build direction-aware inverse maps
verify with Figure 6 residuals
save runtime inverse models as .npz files
```

Encoder angles are loaded through `SEATestbedPlotter`:

```python
red_enc_angs = -stp.theta_1
blue_enc_angs = -stp.theta_0
enc_time = stp.a0_t
```

If the actuator log includes the camera-start timestamp, the camera time vector is shifted into the same elapsed-time frame:

```python
cam_time = stp.camera_start_elapsed + cam_time
```

The script trims camera and encoder data to the active moving region before alignment. It then aligns the camera timeline to the encoder timeline using event-pair landmarks, such as peak/trough structure in the motion.

After coarse alignment, a small residual timing correction can be applied. The correction is estimated from a preliminary inverse map:

```text
time correction ~= angle residual / encoder velocity
```

This correction is bounded so it cannot distort the timeline arbitrarily.

After the coarse/residual timing alignment, the script performs a small per-channel branch-gap time-shift sweep. The sweep asks whether shifting the camera samples by a fraction of a video frame makes the positive and negative calibration branches agree better near low angles. A shift is accepted only if it:

```text
meaningfully reduces low-angle branch gap
stays within the configured frame-shift limit
is not at the edge of the sweep
has enough overlapping samples
```

For the current calibration set, the blue channel accepted a small positive shift while the red channel stayed at zero because the red improvement was too small to trust. The selected shifts and guardrail metrics are stored in:

```text
cal_folder/inverse_time_shift_metadata.json
```

### Inverse mapping

The inverse model is direction-aware. Separate maps are built for:

```text
combined
positive motion
negative motion
```

The current inverse map uses direct unwrapped lookup tables:

```text
camera_angle_unwrapped -> encoder_angle
```

This replaced the earlier wrapped residual model:

```text
camera_phase mod 2*pi -> residual correction
```

The unwrapped lookup is important because repeated forward cycles were not perfectly identical. Wrapping by phase forced different cycles into the same bin and increased forward-rotation residuals.

The active model is absolute-side based:

```text
blue_camera_angle -> blue_encoder_angle
red_camera_angle  -> red_encoder_angle
spring_deflection = blue_calibrated_angle - red_calibrated_angle
```

Calibration diagnostics report the blue-minus-red convention above. The testbed torque-deflection plot currently uses the opposite sign, `red_calibrated_angle - blue_calibrated_angle`, to match the plotted torque convention.

The older direct deflection inverse map:

```text
blue_camera_angle - red_camera_angle -> blue_encoder_angle - red_encoder_angle
```

is no longer generated or used. This keeps the calibration source aligned with the physical measurement path: each optical side is calibrated independently, then spring deflection is formed from the two calibrated side angles.

SVD-regularized calibration models are also written:

```text
cal_folder/svd_blue_cal.npz
cal_folder/svd_red_cal.npz
```

These are diagnostic backup models only. They use a global low-dimensional basis and are useful as a smooth sanity check, but they currently underfit local direction-dependent corrections. Their plots are intentionally commented out in both calibration and testbed figures.

Figure 6 plots:

```text
calibrated camera angle - encoder angle
calibrated blue-red deflection - encoder blue-red deflection
```

in degrees. It also prints branch-specific residual statistics for:

```text
blue all / positive / negative
red all / positive / negative
blue-red deflection
SVD diagnostic blue/red/blue-red
```

### Runtime use in test data

`test_testbed_plotter.py` uses the calibration results in place of encoder measurements for real testbed data. It loads:

```text
cal_folder/inv_blue_directional.npz
cal_folder/inv_red_directional.npz
```

and applies the same direction-aware inverse mapping. The diagnostic CSV files are not used as runtime calibration sources.

The active optical spring deflection in test data is:

```text
optical_deflection = red_calibrated_angle - blue_calibrated_angle
```

The sign is chosen to match the current torque-deflection plot convention. The final test export is:

```text
meas_folder/defl_torque.csv
```

and contains the active optical deflection and torque. SVD candidate curves are available in code but visually disabled.

Generated calibration artifacts include:

```text
cal_folder/camera_enabled_angles.csv
cal_folder/inv_blue.csv
cal_folder/inv_red.csv
cal_folder/inv_blue_directional.npz
cal_folder/inv_red_directional.npz
cal_folder/inverse_time_shift_metadata.json
cal_folder/smooth_blue_cal.npz
cal_folder/smooth_red_cal.npz
cal_folder/svd_blue_cal.npz
cal_folder/svd_red_cal.npz
cal_folder/red_camera_diagnostics.csv
cal_folder/blue_camera_diagnostics.csv
```

`inv_blue.csv` and `inv_red.csv` are diagnostic exports. The runtime calibration model is stored in the `.npz` files.

Generated testbed artifacts include:

```text
meas_folder/camera_enabled_angles.csv
meas_folder/camera_enabled_pre_mask.png
meas_folder/defl_torque.csv
```

The following older artifacts are stale if present and should not be treated as active calibration outputs:

```text
cal_folder/inv_deflection.csv
cal_folder/inv_deflection_directional.npz
cal_folder/inv_blue_timeshift_candidate.csv
cal_folder/inv_red_timeshift_candidate.csv
cal_folder/inv_deflection_timeshift_candidate.csv
cal_folder/inv_blue_timeshift_candidate_directional.npz
cal_folder/inv_red_timeshift_candidate_directional.npz
cal_folder/inv_deflection_timeshift_candidate_directional.npz
```

To force a fresh camera calibration run, remove the generated camera/inverse files and rerun:

```bash
rm -f cal_folder/camera_enabled_angles.csv cal_folder/inv_blue.csv cal_folder/inv_red.csv cal_folder/inv_blue_directional.npz cal_folder/inv_red_directional.npz cal_folder/red_camera_diagnostics.csv cal_folder/blue_camera_diagnostics.csv
MPLBACKEND=Agg pixi run python -s test_calibration_plotter.py
```

`test_calibration_plotter.py` runs the video reader with `display=False` during regeneration so it can run in headless environments. Set the video-reader display path back to interactive only when OpenCV windows are available.

## Raspberry Pi camera environment

This project uses the Raspberry Pi OS `picamera2`/`libcamera` bindings, which are Linux-only and are provided by the system Python packages rather than PyPI. `picamera2` is not managed by `uv` for this project.

Recreate the uv environment on the Raspberry Pi with system site packages enabled:

```bash
uv venv --python /usr/bin/python --system-site-packages --prompt osl-spring-testbed --clear .venv
uv sync
```

The resulting `.venv/pyvenv.cfg` should include:

```ini
include-system-site-packages = true
prompt = osl-spring-testbed
```

Camera scripts should be run on the Raspberry Pi system Python environment where `picamera2` is already installed.
