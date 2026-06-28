from operator import neg
import numpy as np
import matplotlib.pyplot as plt
# from model_params import *
import scipy.io
from Vision.plot_cal import circle_find_3_points
from Vision.video_reading_test import test, CircleAnnotator, CircleTracker
import os.path
from scipy.signal import argrelextrema
from scipy.signal import find_peaks
from scipy.interpolate import PchipInterpolator
from test_calibration_plotter import (
    fit_camera_time_alignment,
    _apply_inverse_lookup,
    _apply_directional_inverse_lookup,
    _apply_svd_inverse_model,
    _load_directional_inverse_model,
    _load_svd_inverse_model,
    _motion_direction,
)
# from scipy.integrate import cumtrapz

# Model Parameters (tuned here)
rad_2_enc = 2607.59458762
J_m = 0.12e-3 # Kg m^2 [LeePanRouse2019IROS]
b_m = 0.16e-3 # Nm s / rad [LeePanRouse2019IROS]
c_m = 9.1e-3 # Nm [LeePanRouse2019IROS]
J_t = 8e-5 # Kg m^2 empirical--tuned to fit torque sensor data
b_t = 2e-5*50 # Nm s/rad (old_plotme.py)
c_t = 2.5/50 # Nm (old_plotme.py)
K_tau = 0.14 # Nm/A [LeePanRouse2019IROS]
K_emf = 0.14 # Vs/rad [LeePanRouse2019IROS]
R_phase = 3./2.*186e-3 # Ohms [LeePanRouse2019IROS] + empirical multiplication by 5 (presumbably losses in the circuit or voltage/current sensor?)
K_spring = 120# 150 # Nm/rad estimated
spring_degree_limit = 15 # degrees
# Position Control Parameters
# kp=200 # was 50
# ki=0 # was 3
dephy_kp_gain_conversion = .025# 1e0 # Converts dephy units to q-axis volts per radian
sign = np.sign

def simple_velocity(X, T, n=20, s=.001):
    return (X[n:]-X[:-n]) / (T[n:]-T[:-n]) *s

def simple_midpoint(T, n=20):
    return T[n:]*.5+.5*T[:-n]

def cumulative_sum(array):
    array = np.array(array)
    x = 0
    for i in range(len(array)):
        x+=array[i]
        array[i]=x
    return array

conversion_to_spring_frame_degrees= lambda x: x/(45.5111*50.)
ticks_to_motor_radians = lambda x: x*(1)
# torque_sensor_callibrated_volts_2_Nm = np.vectorize(lambda x: 1.0014*x+0.0044 if x>=0 else .9998*x + 0.0018)
torque_sensor_callibrated_volts_2_Nm = np.vectorize(lambda x: 20*x)

def _find_active_window(time_values, signals, threshold_ratio=0.15, pad_ratio=0.05):
    n_samples = len(time_values)
    if n_samples < 2:
        return 0, n_samples

    score = np.zeros(n_samples, dtype=float)
    for signal in signals:
        signal = np.asarray(signal, dtype=float)
        baseline = np.median(signal[:max(5, min(n_samples, 50))])
        centered = signal - baseline
        amplitude = np.max(np.abs(centered))
        if amplitude > 0:
            score = np.maximum(score, np.abs(centered) / (amplitude + 1e-12))

    active = np.flatnonzero(score >= threshold_ratio)
    if active.size == 0:
        return 0, n_samples

    pad = max(10, int(pad_ratio * n_samples))
    start_ind = max(0, active[0] - pad)
    end_ind = min(n_samples, active[-1] + pad + 1)
    if end_ind - start_ind < 20:
        return 0, n_samples
    return start_ind, end_ind

def _select_alignment_signal_indices(enc_signals, relative_threshold=0.2, absolute_threshold=1e-4):
    amplitudes = []
    for signal in enc_signals:
        signal = np.asarray(signal, dtype=float)
        amplitudes.append(np.max(signal) - np.min(signal))

    max_amplitude = max(amplitudes, default=0.0)
    if max_amplitude <= 0:
        return tuple(range(len(enc_signals)))

    selected = tuple(
        i
        for i, amplitude in enumerate(amplitudes)
        if amplitude >= absolute_threshold and amplitude >= relative_threshold * max_amplitude
    )
    return selected if selected else tuple(range(len(enc_signals)))

def _signal_excursion(signal):
    signal = np.asarray(signal, dtype=float)
    return float(np.max(signal) - np.min(signal))

def _apply_affine_inverse_model(camera_values, inverse_model):
    camera_values = np.asarray(camera_values, dtype=float)
    model = inverse_model["combined"]
    return model["slope"] * camera_values + model["intercept"]

def _apply_blended_inverse_model(camera_values, inverse_model):
    return _apply_inverse_lookup(camera_values, inverse_model["combined"])

def _continuity_filter(time_values, measurement, alpha=0.72, beta=0.08,
        max_residual_deg=0.16):
    time_values = np.asarray(time_values, dtype=float)
    measurement = np.asarray(measurement, dtype=float)
    if measurement.size < 2:
        return measurement.copy()

    filtered = np.empty_like(measurement)
    filtered[0] = measurement[0]
    velocity = (measurement[1] - measurement[0]) / (time_values[1] - time_values[0] + 1e-12)
    max_residual = np.deg2rad(max_residual_deg)

    for idx in range(1, measurement.size):
        dt = time_values[idx] - time_values[idx - 1]
        if dt <= 0:
            filtered[idx] = filtered[idx - 1]
            continue

        prediction = filtered[idx - 1] + velocity * dt
        residual = np.clip(measurement[idx] - prediction, -max_residual, max_residual)
        filtered[idx] = prediction + alpha * residual
        velocity = velocity + beta * residual / dt

    return filtered

def _apply_continuous_branch_inverse_lookup(camera_values, time_values, inverse_model):
    positive_model = inverse_model.get("positive")
    negative_model = inverse_model.get("negative")
    if positive_model is None or negative_model is None:
        return _apply_directional_inverse_lookup(camera_values, time_values, inverse_model)

    camera_values = np.asarray(camera_values, dtype=float)
    time_values = np.asarray(time_values, dtype=float)
    direction = _motion_direction(camera_values, time_values)
    positive_values = _apply_inverse_lookup(camera_values, positive_model)
    negative_values = _apply_inverse_lookup(camera_values, negative_model)
    branch_values = np.where(direction >= 0, positive_values, negative_values)

    corrected = np.empty_like(branch_values)
    corrected[0] = branch_values[0]
    active_offset = 0.0
    previous_direction = direction[0]

    for idx in range(1, branch_values.size):
        if direction[idx] != previous_direction:
            active_offset = corrected[idx - 1] - branch_values[idx]
            previous_direction = direction[idx]
        corrected[idx] = branch_values[idx] + active_offset

    return corrected

def _moving_average_edge(values, window_size):
    values = np.asarray(values, dtype=float)
    if window_size <= 1 or values.size < 2:
        return values.copy()
    window_size = int(window_size)
    if window_size % 2 == 0:
        window_size += 1
    pad = window_size // 2
    padded = np.pad(values, pad, mode="edge")
    kernel = np.ones(window_size, dtype=float) / window_size
    return np.convolve(padded, kernel, mode="valid")

def _find_zero_crossing_indices(signal, min_spacing):
    signal = np.asarray(signal, dtype=float)
    signs = np.sign(signal)
    crossing_candidates = np.flatnonzero(signs[:-1] * signs[1:] < 0) + 1
    if crossing_candidates.size == 0:
        return crossing_candidates

    selected = [int(crossing_candidates[0])]
    for idx in crossing_candidates[1:]:
        if idx - selected[-1] >= min_spacing:
            selected.append(int(idx))
    return np.array(selected, dtype=int)

def _scale_raw_optical_deflection(raw_optical_deflection_deg, encoder_deflection_deg,
        baseline_ratio=0.03):
    raw_optical_deflection_deg = np.asarray(raw_optical_deflection_deg, dtype=float)
    encoder_deflection_deg = np.asarray(encoder_deflection_deg, dtype=float)
    if raw_optical_deflection_deg.size < 2:
        return raw_optical_deflection_deg.copy(), {"scale": 1.0, "baseline_count": raw_optical_deflection_deg.size}

    baseline_count = max(5, int(baseline_ratio * raw_optical_deflection_deg.size))
    baseline_count = min(baseline_count, raw_optical_deflection_deg.size)
    raw_zero = np.median(raw_optical_deflection_deg[:baseline_count])
    enc_zero = np.median(encoder_deflection_deg[:baseline_count])
    raw_centered = raw_optical_deflection_deg - raw_zero
    enc_centered = encoder_deflection_deg - enc_zero
    denom = np.dot(raw_centered, raw_centered)
    scale = np.dot(raw_centered, enc_centered) / (denom + 1e-12)
    scaled = raw_centered * scale + enc_zero
    residual = scaled - encoder_deflection_deg
    metrics = {
        "scale": float(scale),
        "baseline_count": int(baseline_count),
        "rmse_deg": float(np.sqrt(np.mean(residual ** 2))),
        "p95_abs_deg": float(np.percentile(np.abs(residual), 95)),
    }
    return scaled, metrics

def _register_optical_deflection_to_encoder(
        time_values,
        optical_deflection_deg,
        encoder_deflection_deg,
        prominence_ratio=0.08,
        min_distance_ratio=0.04,
        offset_smooth_ratio=0.015,
        low_signal_gate_ratio=0.12):
    time_values = np.asarray(time_values, dtype=float)
    optical_deflection_deg = np.asarray(optical_deflection_deg, dtype=float)
    encoder_deflection_deg = np.asarray(encoder_deflection_deg, dtype=float)
    if time_values.size < 4:
        return optical_deflection_deg.copy(), np.zeros_like(optical_deflection_deg), np.array([], dtype=int), {}

    n_samples = time_values.size
    excursion = _signal_excursion(encoder_deflection_deg)
    prominence = max(excursion * prominence_ratio, 1e-6)
    min_distance = max(1, int(min_distance_ratio * n_samples))

    peaks, _ = find_peaks(encoder_deflection_deg, prominence=prominence, distance=min_distance)
    troughs, _ = find_peaks(-encoder_deflection_deg, prominence=prominence, distance=min_distance)
    anchors = np.unique(np.r_[peaks, troughs]).astype(int)
    anchors.sort()

    anchor_offsets = encoder_deflection_deg[anchors] - optical_deflection_deg[anchors]
    if anchors.size >= 4:
        offset_model = PchipInterpolator(time_values[anchors], anchor_offsets, extrapolate=False)
        offset = offset_model(np.clip(time_values, time_values[anchors[0]], time_values[anchors[-1]]))
    elif anchors.size >= 2:
        offset = np.interp(time_values, time_values[anchors], anchor_offsets)
    else:
        offset = np.full_like(optical_deflection_deg, np.median(anchor_offsets) if anchor_offsets.size else 0.0)

    smooth_window = max(1, int(offset_smooth_ratio * n_samples))
    offset = _moving_average_edge(offset, smooth_window)
    gate_threshold = max(excursion * low_signal_gate_ratio, 0.5)
    correction_weight = np.clip(np.abs(optical_deflection_deg) / (gate_threshold + 1e-12), 0.0, 1.0)
    offset = offset * correction_weight
    registered = optical_deflection_deg + offset
    residual = registered - encoder_deflection_deg
    metrics = {
        "anchor_count": int(anchors.size),
        "peak_count": int(peaks.size),
        "trough_count": int(troughs.size),
        "zero_count": 0,
        "rmse_deg": float(np.sqrt(np.mean(residual ** 2))),
        "p95_abs_deg": float(np.percentile(np.abs(residual), 95)),
        "max_abs_deg": float(np.max(np.abs(residual))),
        "offset_span_deg": float(np.max(offset) - np.min(offset)),
        "gate_threshold_deg": float(gate_threshold),
    }
    return registered, offset, anchors, metrics

def _detect_extrema_indices(signal, prominence_ratio=0.08, min_distance_ratio=0.04):
    signal = np.asarray(signal, dtype=float)
    n_samples = signal.size
    if n_samples < 3:
        return np.array([], dtype=int), np.array([], dtype=int)

    prominence = max(_signal_excursion(signal) * prominence_ratio, 1e-6)
    min_distance = max(1, int(min_distance_ratio * n_samples))
    peaks, _ = find_peaks(signal, prominence=prominence, distance=min_distance)
    troughs, _ = find_peaks(-signal, prominence=prominence, distance=min_distance)
    return peaks.astype(int), troughs.astype(int)

def _peak_time_align_optical_to_encoder(time_values, optical_deflection_deg,
        encoder_deflection_deg, window_s=0.55, max_shift_s=0.18):
    time_values = np.asarray(time_values, dtype=float)
    optical_deflection_deg = np.asarray(optical_deflection_deg, dtype=float)
    encoder_deflection_deg = np.asarray(encoder_deflection_deg, dtype=float)
    if time_values.size < 4:
        return optical_deflection_deg.copy(), np.zeros_like(optical_deflection_deg), np.array([], dtype=int), {}

    enc_peaks, enc_troughs = _detect_extrema_indices(encoder_deflection_deg)
    opt_peaks, opt_troughs = _detect_extrema_indices(optical_deflection_deg)

    matched = []
    used_opt = set()
    for enc_indices, opt_indices, sign_label in (
            (enc_peaks, opt_peaks, 1),
            (enc_troughs, opt_troughs, -1)):
        for enc_idx in enc_indices:
            if opt_indices.size == 0:
                continue
            time_delta = np.abs(time_values[opt_indices] - time_values[enc_idx])
            nearest_order = np.argsort(time_delta)
            for candidate_rank in nearest_order:
                opt_idx = int(opt_indices[candidate_rank])
                if opt_idx in used_opt:
                    continue
                shift = time_values[opt_idx] - time_values[enc_idx]
                if abs(shift) <= max_shift_s:
                    matched.append((int(enc_idx), opt_idx, float(shift), sign_label))
                    used_opt.add(opt_idx)
                break

    if not matched:
        metrics = {
            "matched_count": 0,
            "max_abs_shift_ms": 0.0,
            "p95_abs_shift_ms": 0.0,
        }
        return optical_deflection_deg.copy(), np.zeros_like(optical_deflection_deg), np.array([], dtype=int), metrics

    correction_sum = np.zeros_like(time_values)
    weight_sum = np.zeros_like(time_values)
    for enc_idx, _, shift, _ in matched:
        distance = np.abs(time_values - time_values[enc_idx])
        in_window = distance < window_s
        if not np.any(in_window):
            continue
        weight = 0.5 * (1.0 + np.cos(np.pi * distance[in_window] / window_s))
        correction_sum[in_window] += shift * weight
        weight_sum[in_window] += weight

    correction = np.zeros_like(time_values)
    active = weight_sum > 1e-12
    correction[active] = correction_sum[active] / weight_sum[active]

    applied_scale = 1.0
    warped_time = time_values + correction
    nonmonotonic_applied = False
    if np.any(np.diff(warped_time) <= 0):
        for scale in (0.5, 0.25, 0.125, 0.0625):
            candidate_correction = correction * scale
            candidate_warped_time = time_values + candidate_correction
            if np.all(np.diff(candidate_warped_time) > 0):
                correction = candidate_correction
                warped_time = candidate_warped_time
                applied_scale = scale
                break

    if np.any(np.diff(warped_time) <= 0):
        nonmonotonic_applied = True

    aligned = np.interp(warped_time, time_values, optical_deflection_deg)
    shifts = np.asarray([item[2] for item in matched], dtype=float)
    metrics = {
        "matched_count": len(matched),
        "max_abs_shift_ms": float(np.max(np.abs(shifts)) * 1e3),
        "p95_abs_shift_ms": float(np.percentile(np.abs(shifts), 95) * 1e3),
        "disabled_nonmonotonic": nonmonotonic_applied,
        "applied_scale": float(applied_scale),
    }
    return aligned, correction, np.array([item[0] for item in matched], dtype=int), metrics

def _plot_inverse_model_construction(inv_blue, inv_red, blue_camera_values, red_camera_values):
    def _plot_model(ax, inverse_model, camera_values, title):
        colors = {
            "combined": "0.35",
            "positive": "tab:orange",
            "negative": "tab:purple",
        }
        combined = inverse_model["combined"]
        combined_lookup = combined["lookup"]
        combined_fit = combined["slope"] * combined_lookup[:, 0] + combined["intercept"]
        combined_residual_deg = (combined_fit - combined_lookup[:, 1]) * 180 / np.pi
        affine_rmse = float(np.sqrt(np.mean(combined_residual_deg ** 2)))
        affine_p95 = float(np.percentile(np.abs(combined_residual_deg), 95))
        affine_max = float(np.max(np.abs(combined_residual_deg)))
        blended_vs_affine_p95 = float(np.percentile(np.abs(combined_residual_deg), 95))
        test_min = float(np.min(camera_values))
        test_max = float(np.max(camera_values))
        lookup_min = float(combined_lookup[0, 0])
        lookup_max = float(combined_lookup[-1, 0])
        test_coverage = (
            "inside"
            if test_min >= lookup_min and test_max <= lookup_max
            else "outside"
        )
        affine_branch_metrics = []
        for key in ("positive", "negative"):
            branch = inverse_model.get(key)
            if branch is None:
                continue
            lookup = branch["lookup"]
            branch_fit = combined["slope"] * lookup[:, 0] + combined["intercept"]
            branch_residual_deg = (lookup[:, 1] - branch_fit) * 180 / np.pi
            affine_branch_metrics.append(np.percentile(np.abs(branch_residual_deg), 95))

        branch_gap_p95 = np.nan
        positive = inverse_model.get("positive")
        negative = inverse_model.get("negative")
        if positive is not None and negative is not None:
            common_min = max(positive["lookup"][0, 0], negative["lookup"][0, 0])
            common_max = min(positive["lookup"][-1, 0], negative["lookup"][-1, 0])
            if common_max > common_min:
                common_x = np.linspace(common_min, common_max, 300)
                positive_y = np.interp(common_x, positive["lookup"][:, 0], positive["lookup"][:, 1])
                negative_y = np.interp(common_x, negative["lookup"][:, 0], negative["lookup"][:, 1])
                branch_gap_p95 = float(np.percentile(np.abs(positive_y - negative_y) * 180 / np.pi, 95))

        for key in ("combined", "positive", "negative"):
            branch = inverse_model.get(key)
            if branch is None:
                continue
            lookup = branch["lookup"]
            ax.plot(
                lookup[:, 0] * 180 / np.pi,
                lookup[:, 1] * 180 / np.pi,
                ".",
                color=colors[key],
                markersize=2,
                alpha=0.45,
                label="%s lookup" % key,
            )
            if key == "combined":
                x_model = np.linspace(lookup[0, 0], lookup[-1, 0], 300)
                y_blended = np.interp(x_model, lookup[:, 0], lookup[:, 1])
                y_model = branch["slope"] * x_model + branch["intercept"]
                ax.plot(
                    x_model * 180 / np.pi,
                    y_blended * 180 / np.pi,
                    color="tab:green",
                    linewidth=1.5,
                    label="blended centerline",
                )
                ax.plot(
                    x_model * 180 / np.pi,
                    y_model * 180 / np.pi,
                    "k-",
                    linewidth=1.5,
                    label="affine model used",
                )

        cam_min = np.min(camera_values) * 180 / np.pi
        cam_max = np.max(camera_values) * 180 / np.pi
        ax.axvspan(cam_min, cam_max, color="tab:blue", alpha=0.08, label="test range")
        ax.set_title(title)
        ax.set_xlabel("camera angle (deg)")
        ax.set_ylabel("encoder-equivalent angle (deg)")
        ax.legend(fontsize=8)
        notes = [
            "affine residual on combined lookup:",
            "  RMSE %.4f deg" % affine_rmse,
            "  p95 %.4f deg" % affine_p95,
            "  max %.4f deg" % affine_max,
            "blended-affine p95 %.4f deg" % blended_vs_affine_p95,
            "branch-vs-affine p95 %.4f deg" % (
                max(affine_branch_metrics) if affine_branch_metrics else np.nan
            ),
            "pos-neg branch gap p95 %.4f deg" % branch_gap_p95,
            "test range: %s" % test_coverage,
            "  %.2f..%.2f deg" % (cam_min, cam_max),
            "lookup range:",
            "  %.2f..%.2f deg" % (lookup_min * 180 / np.pi, lookup_max * 180 / np.pi),
        ]
        ax.text(
            1.02,
            0.98,
            "\n".join(notes),
            transform=ax.transAxes,
            va="top",
            ha="left",
            fontsize=8,
            bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.85, "edgecolor": "0.75"},
        )

    fig, axes = plt.subplots(1, 2, num=7, figsize=(15, 5), clear=True)
    _plot_model(axes[0], inv_blue, blue_camera_values, "Blue inverse map construction")
    _plot_model(axes[1], inv_red, red_camera_values, "Red inverse map construction")
    fig.suptitle("Inverse Mapping: Discrete Calibration Points to Continuous Test Model")
    fig.tight_layout(rect=(0, 0, 0.86, 0.95))

def _print_peak_side_diagnostics(time_values, center_idx, label, signal_deg,
        half_window_s=0.45):
    time_values = np.asarray(time_values, dtype=float)
    signal_deg = np.asarray(signal_deg, dtype=float)
    window = np.abs(time_values - time_values[center_idx]) <= half_window_s
    if np.count_nonzero(window) < 3:
        print("%s peak-side diagnostic: not enough samples" % label)
        return

    local_time = time_values[window]
    local_signal = signal_deg[window]
    step = np.diff(local_signal)
    dt = np.diff(local_time)
    slope = step / (dt + 1e-12)
    print(
        "%s peak-side diagnostic: range=%.6f deg max_step=%.6f deg max_slope=%.6f deg/s median_step=%.6f deg"
        % (
            label,
            np.max(local_signal) - np.min(local_signal),
            np.max(np.abs(step)),
            np.max(np.abs(slope)),
            np.median(np.abs(step)),
        )
    )

def _detect_signal_landmarks(signal, prominence_ratio=0.08, min_distance_ratio=0.08):
    signal = np.asarray(signal, dtype=float)
    n_samples = len(signal)
    if n_samples < 3:
        return []

    prominence = max(_signal_excursion(signal) * prominence_ratio, 1e-6)
    min_distance = max(1, int(min_distance_ratio * n_samples))
    peaks, _ = find_peaks(signal, prominence=prominence, distance=min_distance)
    troughs, _ = find_peaks(-signal, prominence=prominence, distance=min_distance)

    landmarks = [(idx, 1) for idx in peaks] + [(idx, -1) for idx in troughs]
    landmarks.sort(key=lambda item: item[0])
    if not landmarks:
        return []

    cleaned = []
    for idx, sign in landmarks:
        strength = abs(signal[idx])
        if not cleaned or sign != cleaned[-1][1]:
            cleaned.append([idx, sign, strength])
            continue
        if strength > cleaned[-1][2]:
            cleaned[-1] = [idx, sign, strength]
    return [(idx, sign) for idx, sign, _ in cleaned]

def _match_landmark_sequences(cam_time, cam_signal, enc_time, enc_signal):
    cam_landmarks = _detect_signal_landmarks(cam_signal)
    enc_landmarks = _detect_signal_landmarks(enc_signal)

    best = None
    cam_signs = np.array([sign for _, sign in cam_landmarks], dtype=int)
    enc_signs = np.array([sign for _, sign in enc_landmarks], dtype=int)

    for cam_start in range(len(cam_landmarks)):
        for enc_start in range(len(enc_landmarks)):
            n_match = min(len(cam_landmarks) - cam_start, len(enc_landmarks) - enc_start)
            if n_match < 3:
                continue
            cam_sign_slice = cam_signs[cam_start:cam_start + n_match]
            enc_sign_slice = enc_signs[enc_start:enc_start + n_match]
            if not np.array_equal(cam_sign_slice, enc_sign_slice):
                continue

            cam_idx = np.array([cam_landmarks[cam_start + i][0] for i in range(n_match)], dtype=int)
            enc_idx = np.array([enc_landmarks[enc_start + i][0] for i in range(n_match)], dtype=int)
            cam_landmark_time = cam_time[cam_idx]
            enc_landmark_time = enc_time[enc_idx]

            cam_dt = np.diff(cam_landmark_time)
            enc_dt = np.diff(enc_landmark_time)
            cam_dt_norm = cam_dt / (np.mean(cam_dt) + 1e-12)
            enc_dt_norm = enc_dt / (np.mean(enc_dt) + 1e-12)
            spacing_error = np.mean((cam_dt_norm - enc_dt_norm) ** 2)
            score = (n_match, -spacing_error)
            if best is None or score > best["score"]:
                best = {
                    "score": score,
                    "cam_idx": cam_idx,
                    "enc_idx": enc_idx,
                    "cam_signs": cam_sign_slice,
                    "spacing_error": spacing_error,
                    "cam_count": len(cam_landmarks),
                    "enc_count": len(enc_landmarks),
                }
    return best

def _piecewise_map_camera_time(cam_time, cam_anchor_time, enc_anchor_time):
    cam_time = np.asarray(cam_time, dtype=float)
    cam_anchor_time = np.asarray(cam_anchor_time, dtype=float)
    enc_anchor_time = np.asarray(enc_anchor_time, dtype=float)
    if len(cam_anchor_time) < 2:
        raise ValueError("Need at least two matched landmarks for piecewise time mapping.")

    mapped = np.interp(cam_time, cam_anchor_time, enc_anchor_time)

    left_slope = (enc_anchor_time[1] - enc_anchor_time[0]) / (cam_anchor_time[1] - cam_anchor_time[0] + 1e-12)
    right_slope = (enc_anchor_time[-1] - enc_anchor_time[-2]) / (cam_anchor_time[-1] - cam_anchor_time[-2] + 1e-12)

    left_mask = cam_time < cam_anchor_time[0]
    right_mask = cam_time > cam_anchor_time[-1]
    mapped[left_mask] = enc_anchor_time[0] + left_slope * (cam_time[left_mask] - cam_anchor_time[0])
    mapped[right_mask] = enc_anchor_time[-1] + right_slope * (cam_time[right_mask] - cam_anchor_time[-1])
    return mapped

class SEATestbedPlotter(object):
    def __init__(self, act0_file, act1_file, adc0_file=None):

        act0 = self._load_log(act0_file)
        act1 = self._load_log(act1_file)
        self.has_adc = self.has_traj = self.has_ctrl = False
        self.has_futek = False
        self.camera_start_elapsed = self._estimate_camera_start_elapsed(act0)

        if adc0_file: 
            adc0 = np.loadtxt(adc0_file, delimiter=",", skiprows=1)
            self.has_adc = True

        self._data=dict(headers=[],data=[])
        self._line_index=-1

        self.initial_angle_offset = 0

        if self._is_single_motor_log(act0) and self._same_file(act0_file, act1_file):
            act0_pi_time = self._get_col(act0, "elapsed_time", 3, "timestamp")
            init_pi_time = min([act0_pi_time[0], adc0[0,0]]) if self.has_adc else act0_pi_time[0]

            a0_x = self._get_col(act0, "knee[DephyActuator].output_position", 11, "output_position")
            a0_xd = self._get_col(act0, "knee[DephyActuator].output_velocity", 12, "output_velocity")
            zeros = np.zeros_like(a0_x)

            self.add_line("a0_t", "pi_time", act0_pi_time-init_pi_time)
            self.add_line("a0_ts", "State time", self._get_col(act0, "monotonic_time_ns", 2, "elapsed_time"))
            self.add_line("a0_x", "Motor enc angle", -a0_x + a0_x[0])
            self.add_line("a0_xd", "Motor enc velocity", a0_xd)
            self.add_line("a0_xdd", "Motor enc acceleration", self._differentiate(a0_xd, act0_pi_time))
            self.add_line("a0_vm", "Motor deph voltage", self._get_col(act0, "knee[DephyActuator].motor_voltage", 14, "motor_voltage"))
            self.add_line("a0_im", "Motor deph current", self._get_col(act0, "knee[DephyActuator].motor_current", 15, "motor_current"))
            self.add_line("a0_vb", "Battery deph voltage", self._get_col(act0, "knee[DephyActuator].battery_voltage", 16, "battery_voltage"))
            self.add_line("a0_ib", "Battery deph current", self._get_col(act0, "knee[DephyActuator].battery_current", 17, "battery_current"))

            self.add_line("a1_t", "pi_time", act0_pi_time-init_pi_time)
            self.add_line("a1_ts", "State time", self._get_col(act0, "monotonic_time_ns", 2, "elapsed_time"))
            self.add_line("a1_x", "Motor enc angle", zeros)
            self.add_line("a1_xd", "Motor enc velocity", zeros)
            self.add_line("a1_xdd", "Motor enc acceleration", zeros)
            self.add_line("a1_vm", "Motor deph voltage", zeros)
            self.add_line("a1_im", "Motor deph current", zeros)
            self.add_line("a1_vb", "Battery deph voltage", zeros)
            self.add_line("a1_ib", "Battery deph current", zeros)
        else:
            act0_pi_time = self._get_col(act0, "state_time", 0)
            act1_pi_time = self._get_col(act1, "state_time", 0)
            init_pi_time = min([act0_pi_time[0], act1_pi_time[0], adc0[0,0]]) if self.has_adc else min([act0_pi_time[0], act1_pi_time[0]])

            self.add_line("a0_t", "pi_time", act0_pi_time-init_pi_time)
            self.add_line("a0_ts", "State time", self._get_col(act0, "sys_time", 30, "state_time"))
            self.add_line("a0_x", "Motor enc angle", -self._get_col(act0, "mot_ang", 7)+self._get_col(act0, "mot_ang", 7)[0])
            self.add_line("a0_xd", "Motor enc velocity", self._get_col(act0, "mot_vel", 8))
            self.add_line("a0_xdd", "Motor enc acceleration", self._get_col(act0, "mot_acc", 9))
            self.add_line("a0_vm", "Motor deph voltage", self._get_col(act0, "mot_volt", 11))
            self.add_line("a0_im", "Motor deph current", self._get_col(act0, "mot_cur", 10))
            self.add_line("a0_vb", "Battery deph voltage", self._get_col(act0, "batt_volt", 12))
            self.add_line("a0_ib", "Battery deph current", self._get_col(act0, "batt_curr", 13))

            self.add_line("a1_t", "pi_time", act1_pi_time-init_pi_time)
            self.add_line("a1_ts", "State time", self._get_col(act1, "sys_time", 30, "state_time"))
            self.add_line("a1_x", "Motor enc angle", -self._get_col(act1, "mot_ang", 7)+self._get_col(act1, "mot_ang", 7)[0])
            self.add_line("a1_xd", "Motor enc velocity", self._get_col(act1, "mot_vel", 8))
            self.add_line("a1_xdd", "Motor enc acceleration", self._get_col(act1, "mot_acc", 9))
            self.add_line("a1_vm", "Motor deph voltage", self._get_col(act1, "mot_volt", 11))
            self.add_line("a1_im", "Motor deph current", self._get_col(act1, "mot_cur", 10))
            self.add_line("a1_vb", "Battery deph voltage", self._get_col(act1, "batt_volt", 12))
            self.add_line("a1_ib", "Battery deph current", self._get_col(act1, "batt_curr", 13))

        futek_tau = self._get_futek_torque(act0)
        if futek_tau is None and not self._same_file(act0_file, act1_file):
            futek_tau = self._get_futek_torque(act1)
        if futek_tau is not None:
            self.add_line("tau_futek", "Futek Torque Reading", futek_tau)
            self.add_line("tau", "torque sensor torque signal", futek_tau)
            self.has_futek = True

        if self.has_adc: self.add_line("adc_t", "ADC pi Time", adc0[:,0]-init_pi_time)
        if self.has_adc: self.add_line("adc_v", "ADC Voltage", adc0[:,1])
        if self.has_adc: self.add_line("adc_d", "ADC sampling duration (seconds)", adc0[:,2])

        self.generate_easy_derived_signals()

    @staticmethod
    def _load_log(filename):
        with open(filename, "r", encoding="utf-8") as f:
            headers = [header.strip() for header in f.readline().strip().split(",")]
        data = np.loadtxt(filename, delimiter=",", skiprows=1)
        return {
            "headers": headers,
            "data": np.atleast_2d(data),
        }

    @staticmethod
    def _get_col(data, name, fallback_idx, alt_name=None):
        headers = data.get("headers", [])
        if name in headers:
            return np.asarray(data["data"][:, headers.index(name)], dtype=float)
        if alt_name is not None and alt_name in headers:
            return np.asarray(data["data"][:, headers.index(alt_name)], dtype=float)
        return np.asarray(data["data"][:, fallback_idx], dtype=float)

    @staticmethod
    def _get_optional_col(data, *names, suffix=None):
        headers = data.get("headers", [])
        for name in names:
            if name in headers:
                return np.asarray(data["data"][:, headers.index(name)], dtype=float)
        if suffix is not None:
            for header in headers:
                if header.endswith(suffix):
                    return np.asarray(data["data"][:, headers.index(header)], dtype=float)
        return None

    @staticmethod
    def _get_futek_torque(data):
        return SEATestbedPlotter._get_optional_col(
            data,
            "tau_futek",
            "torque",
            suffix=".torque",
        )

    @staticmethod
    def _estimate_camera_start_elapsed(data):
        elapsed_time = SEATestbedPlotter._get_optional_col(data, "elapsed_time")
        if elapsed_time is None:
            return None

        candidates = []

        monotonic_time_ns = SEATestbedPlotter._get_optional_col(data, "monotonic_time_ns")
        camera_start_monotonic_time_ns = SEATestbedPlotter._get_optional_col(data, "camera_start_monotonic_time_ns")
        if monotonic_time_ns is not None and camera_start_monotonic_time_ns is not None:
            candidates.append(
                elapsed_time - (monotonic_time_ns - camera_start_monotonic_time_ns) * 1e-9
            )

        timestamp = SEATestbedPlotter._get_optional_col(data, "timestamp")
        camera_start_timestamp = SEATestbedPlotter._get_optional_col(data, "camera_start_timestamp")
        if timestamp is not None and camera_start_timestamp is not None:
            candidates.append(elapsed_time - (timestamp - camera_start_timestamp))

        wall_time_ns = SEATestbedPlotter._get_optional_col(data, "wall_time_ns")
        camera_start_wall_time_ns = SEATestbedPlotter._get_optional_col(data, "camera_start_wall_time_ns")
        if wall_time_ns is not None and camera_start_wall_time_ns is not None:
            candidates.append(
                elapsed_time - (wall_time_ns - camera_start_wall_time_ns) * 1e-9
            )

        if not candidates:
            return None

        finite_candidates = [
            candidate[np.isfinite(candidate)]
            for candidate in candidates
            if np.any(np.isfinite(candidate))
        ]
        if not finite_candidates:
            return None
        stacked = np.hstack(finite_candidates)
        return float(np.median(stacked))

    @staticmethod
    def _is_single_motor_log(data):
        headers = set(data.get("headers", []))
        return "knee[DephyActuator].output_position" in headers

    @staticmethod
    def _same_file(path_a, path_b):
        return os.path.abspath(path_a) == os.path.abspath(path_b)

    @staticmethod
    def _differentiate(values, time_values):
        if len(values) < 2:
            return np.zeros_like(values)
        return np.gradient(values, time_values)

    def add_line(self, shortname, name, value):
        self._line_index+=1
        self._data["headers"].append(name)
        self._data["data"].append(value)
        setattr(self, shortname, np.array(self._data["data"][self._line_index]))
        return self._line_index

    def generate_easy_derived_signals(self):
        if self.has_adc: self.add_line("futek_v", "torque sensor voltage signal", 2*(self.adc_v-self.adc_v[0]))
        if self.has_adc and not self.has_futek:
            adc_tau = torque_sensor_callibrated_volts_2_Nm(self.futek_v)
            self.add_line("tau_futek", "Futek Torque Reading", adc_tau)
            self.add_line("tau", "torque sensor torque signal", adc_tau)
            self.has_futek = True
        self.add_line("v0", "q-axis voltage act0", self.a0_vm*1e-3 * np.sqrt(3./2.))
        self.add_line("i0", "q-axis current act0", self.a0_im*1e-3 * .537/np.sqrt(2.))
        self.add_line("v1", "q-axis voltage act1", -self.a1_vm*1e-3 * np.sqrt(3./2.))
        self.add_line("i1", "q-axis current act1", -self.a1_im*1e-3 * .537/np.sqrt(2.))
        self.add_line("phi_0", "motor-side motor angle for act0 in radians", ticks_to_motor_radians(self.a0_x) + self.initial_angle_offset)
        self.add_line("phid_0", "motor-side motor vel for act0 in radians/s", ticks_to_motor_radians(self.a0_xd)*1e3)
        self.add_line("phidd_0", "motor-side motor acc for act0 in radians/s/s", self.a0_xdd)

        self.add_line("i0_prime", "q-axis current act0 inferred from model", self.v0/R_phase - self.phid_0*K_emf/R_phase)
        self.add_line("v0_prime", "q-axis voltage act0 inferred from model", self.i0*R_phase + self.phid_0*K_emf)

        self.add_line("phi_1", "motor-side motor angle for act1 in radians", ticks_to_motor_radians(-self.a1_x) + self.initial_angle_offset)
        self.add_line("phid_1", "motor-side motor velocity for act1 in radians per second", ticks_to_motor_radians(-self.a1_xd)*1e3)
        self.add_line("phidd_1", "motor-side motor acceleration for act1 in radians per second squared", -self.a1_xdd)

        self.add_line("i1_prime", "q-axis current act1 inferred from model", self.v1/R_phase - self.phid_1*K_emf/R_phase)
        self.add_line("v1_prime", "q-axis voltage act1 inferred from model", self.i1*R_phase + self.phid_1*K_emf)

        self.add_line("theta_0", "spring-side angle (dev0), radians", self.phi_0)
        self.add_line("theta_1", "spring-side angle (dev1), radians", self.phi_1)
        self.add_line("delta_s", "spring-side deflection, radians", self.theta_0-self.theta_1)


        self.add_line("tau_m_1", "spring-side angle (dev1), radians", self.i1*K_tau)


def main(cal_folder,inner_mask,outer_mask,test_folder,defl_trq_file='/defl_torque.csv'):
    # Calculate camera_angs 
    if not os.path.exists(test_folder + '/camera_enabled_angles.csv'):

        blue_cam_angs, red_cam_angs, cam_time = test(file = test_folder + '/Stiffness_Measure_260627_195643.h264',
                                                    inner_mask_loc = inner_mask,
                                                    outer_mask_loc = outer_mask,
                                                    pre_mask_save_loc = test_folder + '/camera_enabled_pre_mask.png',
                                                    red_cal_save_loc = None,
                                                    blue_cal_save_loc = None)
        blue_cam_angs = blue_cam_angs - blue_cam_angs[0]
        red_cam_angs = red_cam_angs - red_cam_angs[0]
        cam_enabled_angs = np.vstack((cam_time,blue_cam_angs,red_cam_angs)).T

        with open(test_folder + '/camera_enabled_angles.csv', 'w') as f:
            np.savetxt(f, cam_enabled_angs, fmt='%.7f', delimiter=", ")

    else:
        cam_enabled_angs = np.loadtxt(test_folder + '/camera_enabled_angles.csv', delimiter=',')
        cam_time = cam_enabled_angs[:,0]
        blue_cam_angs = cam_enabled_angs[:,1]
        red_cam_angs = cam_enabled_angs[:,2]

    if False: # for viewing tests without camera data
        stp = SEATestbedPlotter(test_folder + 'camera_enabled_dev0.csv',
                            test_folder + 'camera_enabled_dev1.csv',
                            test_folder + 'camera_enabled_volts.csv')
        red_cam_angs = stp.theta_0
        blue_cam_angs = stp.theta_1
        cam_time = stp.a0_t

    if False:
        stp = SEATestbedPlotter(test_folder + 'camera_enabled_dev0.csv',
                        test_folder + 'camera_enabled_dev1.csv')
        red_enc_angs = stp.theta_0
        blue_enc_angs = stp.theta_1
        enc_time = stp.a0_t
        torque = red_enc_angs*0.0
        trq_time = stp.a0_t


    # Read encoder_angs from SEA_Testbed_Plotter
    stp = SEATestbedPlotter(test_folder + '/Stiffness_Measure_260627_195643.csv',
                            test_folder + '/Stiffness_Measure_260627_195643.csv',
                            # test_folder + '/camera_enabled_volts.csv'
                            )
    red_enc_angs = -(stp.theta_1 - stp.theta_1[0])
    blue_enc_angs = -(stp.theta_0 - stp.theta_0[0])
    enc_time = stp.a0_t
    torque = stp.tau
    trq_time = stp.a0_t

    if stp.camera_start_elapsed is not None:
        cam_time = stp.camera_start_elapsed + cam_time
        print("camera start elapsed anchor: %.6f s" % stp.camera_start_elapsed)

    cam_signal_list = (red_cam_angs, blue_cam_angs)
    enc_signal_list = (red_enc_angs, blue_enc_angs)
    align_signal_indices = _select_alignment_signal_indices(enc_signal_list)
    align_cam_signals = tuple(cam_signal_list[i] for i in align_signal_indices)
    align_enc_signals = tuple(enc_signal_list[i] for i in align_signal_indices)
    primary_align_index = max(
        align_signal_indices,
        key=lambda idx: _signal_excursion(enc_signal_list[idx]),
    )

    cam_start_ind, cam_end_ind = _find_active_window(cam_time, align_cam_signals)
    enc_start_ind, enc_end_ind = _find_active_window(enc_time, align_enc_signals)

    new_cam_time = cam_time[cam_start_ind:cam_end_ind]
    new_blue_cam_angs = blue_cam_angs[cam_start_ind:cam_end_ind]
    new_red_cam_angs = red_cam_angs[cam_start_ind:cam_end_ind]

    new_enc_time = enc_time[enc_start_ind:enc_end_ind]
    new_blue_enc_angs = blue_enc_angs[enc_start_ind:enc_end_ind]
    new_red_enc_angs = red_enc_angs[enc_start_ind:enc_end_ind]
    new_torque = torque[enc_start_ind:enc_end_ind]

    cam_primary_signal = cam_signal_list[primary_align_index][cam_start_ind:cam_end_ind]
    enc_primary_signal = enc_signal_list[primary_align_index][enc_start_ind:enc_end_ind]
    landmark_match = _match_landmark_sequences(
        new_cam_time,
        cam_primary_signal,
        new_enc_time,
        enc_primary_signal,
    )

    if landmark_match is not None:
        cam_anchor_time = new_cam_time[landmark_match["cam_idx"]]
        enc_anchor_time = new_enc_time[landmark_match["enc_idx"]]
        aligned_cam_time = _piecewise_map_camera_time(new_cam_time, cam_anchor_time, enc_anchor_time)
        print(
            "landmark alignment: channel=%d matched=%d camera_landmarks=%d encoder_landmarks=%d spacing_error=%.6g"
            % (
                primary_align_index,
                len(cam_anchor_time),
                landmark_match["cam_count"],
                landmark_match["enc_count"],
                landmark_match["spacing_error"],
            )
        )
    else:
        aligned_cam_time, alignment_scale, alignment_offset, align_error_before, align_error_after = fit_camera_time_alignment(
            new_cam_time,
            tuple(signal[cam_start_ind:cam_end_ind] for signal in align_cam_signals),
            new_enc_time,
            tuple(signal[enc_start_ind:enc_end_ind] for signal in align_enc_signals),
        )
        print(
            "fallback affine alignment: scale=%.8f, offset=%.6f s, error %.6g -> %.6g"
            % (alignment_scale, alignment_offset, align_error_before, align_error_after)
        )
    print("alignment channels:", align_signal_indices, "primary:", primary_align_index)

    plot_origin = min(aligned_cam_time[0], new_enc_time[0])
    aligned_cam_time_plot = aligned_cam_time - plot_origin
    new_enc_time_plot = new_enc_time - plot_origin

    plt.figure(2)
    plt.plot(aligned_cam_time_plot, new_red_cam_angs, 'r')
    plt.plot(aligned_cam_time_plot, new_blue_cam_angs, 'b')
    plt.plot(new_enc_time_plot, new_blue_enc_angs, 'c')
    plt.plot(new_enc_time_plot, new_red_enc_angs, 'm')
    plt.legend([
        'red_cam_angs', 'blue_cam_angs',
        'blue_enc_angs', 'red_enc_angs',
    ])
    plt.show()


    # Second calibration, apply the time-shift-corrected direction-aware camera-to-encoder inverse map.
    inv_blue = _load_directional_inverse_model(cal_folder + '/inv_blue_directional.npz')
    inv_red = _load_directional_inverse_model(cal_folder + '/inv_red_directional.npz')
    svd_candidate_available = (
        os.path.exists(cal_folder + '/svd_blue_cal.npz')
        and os.path.exists(cal_folder + '/svd_red_cal.npz')
    )
    if svd_candidate_available:
        svd_blue_cal = _load_svd_inverse_model(cal_folder + '/svd_blue_cal.npz')
        svd_red_cal = _load_svd_inverse_model(cal_folder + '/svd_red_cal.npz')
    else:
        svd_blue_cal = None
        svd_red_cal = None
    unwrapped_blue_cam = np.unwrap(new_blue_cam_angs)
    unwrapped_red_cam = np.unwrap(new_red_cam_angs)

    blue_cam_ang_cal = _apply_directional_inverse_lookup(
        unwrapped_blue_cam,
        aligned_cam_time,
        inv_blue,
    )
    red_cam_ang_cal = _apply_directional_inverse_lookup(
        unwrapped_red_cam,
        aligned_cam_time,
        inv_red,
    )
    if svd_candidate_available:
        blue_cam_ang_cal_svd = _apply_svd_inverse_model(
            unwrapped_blue_cam,
            aligned_cam_time,
            svd_blue_cal,
        )
        red_cam_ang_cal_svd = _apply_svd_inverse_model(
            unwrapped_red_cam,
            aligned_cam_time,
            svd_red_cal,
        )
    else:
        blue_cam_ang_cal_svd = np.full_like(unwrapped_blue_cam, np.nan)
        red_cam_ang_cal_svd = np.full_like(unwrapped_red_cam, np.nan)

    valid_enc = (new_enc_time >= aligned_cam_time[0]) & (new_enc_time <= aligned_cam_time[-1])
    if np.count_nonzero(valid_enc) < 2:
        raise ValueError("Camera and encoder windows do not overlap after alignment.")

    plot_time = new_enc_time[valid_enc]
    blue_enc_plot = new_blue_enc_angs[valid_enc]
    red_enc_plot = new_red_enc_angs[valid_enc]
    torque_plot = new_torque[valid_enc]
    plot_time_rel = plot_time - plot_origin

    red_cam_ang_cal_res = np.interp(plot_time, aligned_cam_time, red_cam_ang_cal)
    blue_cam_ang_cal_res = np.interp(plot_time, aligned_cam_time, blue_cam_ang_cal)
    red_cam_ang_svd_res = np.interp(plot_time, aligned_cam_time, red_cam_ang_cal_svd)
    blue_cam_ang_svd_res = np.interp(plot_time, aligned_cam_time, blue_cam_ang_cal_svd)
    red_cam_raw_res = np.interp(plot_time, aligned_cam_time, unwrapped_red_cam)
    blue_cam_raw_res = np.interp(plot_time, aligned_cam_time, unwrapped_blue_cam)
    optical_defl_directional = (red_cam_ang_cal_res - blue_cam_ang_cal_res)*180/np.pi
    optical_defl_svd = (red_cam_ang_svd_res - blue_cam_ang_svd_res)*180/np.pi
    enc_defl = (red_enc_plot - blue_enc_plot)*180/np.pi
    optical_defl_raw = (red_cam_raw_res - blue_cam_raw_res)*180/np.pi
    red_cam_drift_res = red_cam_ang_cal_res - np.median(red_cam_ang_cal_res)
    optical_residual = optical_defl_directional - enc_defl
    svd_residual = optical_defl_svd - enc_defl
    print(
        "testbed time-shift corrected directional inverse residual: p95_abs %.6f deg, max_abs %.6f deg"
        % (
            np.percentile(np.abs(optical_residual), 95),
            np.max(np.abs(optical_residual)),
        )
    )
    if svd_candidate_available:
        print(
            "testbed SVD candidate residual: p95_abs %.6f deg, max_abs %.6f deg"
            % (
                np.nanpercentile(np.abs(svd_residual), 95),
                np.nanmax(np.abs(svd_residual)),
            )
        )
    else:
        print("SVD candidate calibration files unavailable; skipping SVD comparison curve")
    print(
        "red optical motion relative to grounded encoder: p95_abs=%.6f deg, max_abs=%.6f deg"
        % (
            np.percentile(np.abs(red_cam_drift_res), 95) * 180 / np.pi,
            np.max(np.abs(red_cam_drift_res)) * 180 / np.pi,
        )
    )
    peak_idx = int(np.argmax(np.abs(enc_defl)))
    _print_peak_side_diagnostics(
        plot_time_rel,
        peak_idx,
        "red directional side",
        red_cam_ang_cal_res * 180 / np.pi,
    )
    _print_peak_side_diagnostics(
        plot_time_rel,
        peak_idx,
        "blue directional side",
        blue_cam_ang_cal_res * 180 / np.pi,
    )
    _print_peak_side_diagnostics(
        plot_time_rel,
        peak_idx,
        "time-shift corrected red-minus-blue",
        optical_defl_directional,
    )
    if svd_candidate_available:
        _print_peak_side_diagnostics(
            plot_time_rel,
            peak_idx,
            "SVD candidate red-minus-blue",
            optical_defl_svd,
        )
    _print_peak_side_diagnostics(
        plot_time_rel,
        peak_idx,
        "raw red-minus-blue",
        optical_defl_raw,
    )
    
    plt.figure(3)
    plt.plot(plot_time_rel, enc_defl, color='tab:blue')
    plt.plot(plot_time_rel, optical_defl_directional, color='0.25')
    figure3_legend = [
        'Encoder deflection',
        'Time-shift corrected directional optical deflection',
    ]
    if svd_candidate_available:
        plt.plot(plot_time_rel, optical_defl_svd, color='tab:green')
        figure3_legend.append('SVD-regularized optical candidate')
    plt.legend(figure3_legend)
    plt.title('Deflection Measurement Comparison')
    plt.xlabel('Time (s)')
    plt.ylabel('Deflection (deg)')
    plt.show()

    side_zoom = np.abs(plot_time_rel - plot_time_rel[peak_idx]) <= 0.7
    plt.figure(9)
    plt.subplot(2, 1, 1)
    plt.plot(plot_time_rel[side_zoom], red_cam_ang_cal_res[side_zoom] * 180 / np.pi, 'r')
    plt.plot(plot_time_rel[side_zoom], blue_cam_ang_cal_res[side_zoom] * 180 / np.pi, 'b')
    plt.plot(plot_time_rel[side_zoom], red_cam_raw_res[side_zoom] * 180 / np.pi, 'r--', alpha=0.45)
    plt.plot(plot_time_rel[side_zoom], blue_cam_raw_res[side_zoom] * 180 / np.pi, 'b--', alpha=0.45)
    plt.axvline(plot_time_rel[peak_idx], color='0.5', linewidth=1)
    side_angle_legend = [
        'Red time-shift corrected inverse',
        'Blue time-shift corrected inverse',
        'Red raw camera',
        'Blue raw camera',
    ]
    plt.legend(side_angle_legend)
    plt.ylabel('Side angle (deg)')
    plt.title('Peak-Side Time-Shift Corrected Inverse Diagnostics')
    plt.subplot(2, 1, 2)
    plt.plot(plot_time_rel[side_zoom], enc_defl[side_zoom], color='tab:blue')
    plt.plot(plot_time_rel[side_zoom], optical_defl_directional[side_zoom], color='0.25')
    if svd_candidate_available:
        plt.plot(plot_time_rel[side_zoom], optical_defl_svd[side_zoom], color='tab:green')
    plt.plot(plot_time_rel[side_zoom], optical_defl_raw[side_zoom], color='tab:olive')
    plt.axvline(plot_time_rel[peak_idx], color='0.5', linewidth=1)
    side_zoom_legend = [
        'Encoder deflection',
        'Time-shift corrected red-minus-blue',
    ]
    if svd_candidate_available:
        side_zoom_legend.append('SVD candidate red-minus-blue')
    side_zoom_legend.append(
        'Raw red-minus-blue',
    )
    plt.legend(side_zoom_legend)
    plt.xlabel('Time (s)')
    plt.ylabel('Deflection (deg)')
    plt.show()

    plt.figure(4)
    plt.plot(plot_time_rel, optical_residual, color='0.25')
    residual_legend = ['Time-shift corrected optical - encoder']
    if svd_candidate_available:
        plt.plot(plot_time_rel, svd_residual, color='tab:green')
        residual_legend.append('SVD candidate optical - encoder')
    plt.legend(residual_legend)
    plt.title('Directional Optical Residual')
    plt.xlabel('Time (s)')
    plt.ylabel('Deflection error (deg)')
    plt.show()
    
    # torque_cam_reg = torque[red_enc_pks[0]:red_enc_pks[-1]]
    
    # red_enc_ang_res = np.interp(cam_time[red_cam_pks[0]:red_cam_pks[-1]], 
    #                             enc_time, 
    #                             red_enc_angs[red_enc_pks[0]:red_enc_pks[-1]])
    # blue_enc_ang_res = np.interp(cam_time[red_cam_pks[0]:red_cam_pks[-1]], 
    #                              enc_time, 
    #                              blue_enc_angs[red_enc_pks[0]:red_enc_pks[-1]])
    # Validation thing
    # red_cam_rng_res = np.interp(enc_time,cam_time,red_cam_angs)
    # plt.figure(5)
    # plt.plot(enc_time[red_enc_pks[0]:red_enc_pks[-1]], 
    #          red_cam_ang_cal_res - red_enc_angs[red_enc_pks[0]:red_enc_pks[-1]], 'r')
    # plt.plot(enc_time[red_enc_pks[0]:red_enc_pks[-1]], 
    #          blue_cam_ang_cal_res - blue_enc_angs[red_enc_pks[0]:red_enc_pks[-1]], 'b')
    # p = np.polyfit(red_cam_rng_res,red_enc_angs,1)
    # print("Fit: ",p)
    
    # plt.figure(6)
    # plt.plot(cam_time[red_cam_pks[0]:red_cam_pks[-1]], 
    #          red_cam_ang_cal[red_cam_pks[0]:red_cam_pks[-1]] - red_enc_ang_res, 'r')
    # plt.plot(cam_time[red_cam_pks[0]:red_cam_pks[-1]],
    #          blue_cam_ang_cal[red_cam_pks[0]:red_cam_pks[-1]] - blue_enc_ang_res, 'b')
    
    plt.figure(1)
    plt.plot(aligned_cam_time_plot, new_red_cam_angs*180/np.pi, 'r')
    plt.plot(aligned_cam_time_plot, new_blue_cam_angs*180/np.pi, 'b')
    plt.plot(new_enc_time_plot, new_blue_enc_angs*180/np.pi, 'c')
    plt.plot(new_enc_time_plot, new_red_enc_angs*180/np.pi, 'm')
    plt.plot(new_enc_time_plot, -new_torque)
    plt.legend(['Red Cam','Blue Cam','Blue Enc','Red Enc',
                'Torque Sensor'])
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (deg)')
    plt.title('Raw Camera and Encoder Signals')

    # defl = (red_cam_ang_cal - blue_cam_ang_cal)*180/np.pi
    # torque_res = np.interp(cam_time, trq_time, torque)
    
    # defl_torque = np.vstack((defl,torque_res)).T
    # defl_torque = np.vstack((enc_defl,torque)).T

    # plt.figure(2)
    # plt.plot(cam_time,defl)
    # plt.plot(enc_time,enc_defl)

    plt.figure(2)
    plt.plot(enc_defl, torque_plot)
    plt.plot(optical_defl_directional, torque_plot)
    torque_legend = [
        'Motor Encoder Measurement',
        'Optical Time-Shift Corrected Directional Inverse',
    ]
    if svd_candidate_available:
        plt.plot(optical_defl_svd, torque_plot)
        torque_legend.append('Optical SVD-Regularized Candidate')
    plt.legend(torque_legend)
    plt.xlabel('Deflection (deg)')
    plt.ylabel('Torque (Nm)')

    with open(test_folder + defl_trq_file, 'w') as f:
        np.savetxt(f, np.vstack(
            (optical_defl_directional, torque_plot)
            ).T, fmt='%.7f', delimiter=", ")



if __name__ == '__main__':

    # main(cal_folder='data/08_30_22_T13/',
    #         inner_mask = 'inner_mask0830.png',
    #         outer_mask = 'outer_mask0830.png',
    #         test_folder='data/09_02_22_T1/',
    #         defl_trq_file='S4_mencoder.csv')

    # main(cal_folder='data/08_30_22_T13/',
    #         inner_mask = 'inner_mask0830.png',
    #         outer_mask = 'outer_mask0830.png',
    #         test_folder='data/09_02_22_T3/',
    #         defl_trq_file='S4_mencoder.csv')

    # main(cal_folder='data/08_30_22_T13/',
    #         inner_mask = 'inner_mask0830.png',
    #         outer_mask = 'outer_mask0830.png',
    #         test_folder='data/09_02_22_T4/',
    #         defl_trq_file='S4_mencoder.csv')
    # plt.legend(['No Lubricant','Graphite Lubricant','Graphite SLOW'])


    # main(cal_folder='data/08_16_22_T1/',
    #         inner_mask = 'inner_mask5.png',
    #         outer_mask = 'outer_mask3.png',
    #         test_folder='data/08_16_22_T4/',
    #         defl_trq_file='S1_14_19.csv')    

    main(cal_folder='./cal_folder',
            inner_mask = 'mask_inner_0627.png',
            outer_mask = 'mask_outer_0627.png',
            test_folder='./meas_folder',
        )
    plt.show()
"""
Bibliography
walking_knee_78kg.mat
    [LeePanRouse2019IROS] Ung Hee Lee, Chen-Wen Pan, and Elliott J. Rouse "Empirical Characterization
        of a High-performance Exterior-rotor Type Brushless DC Motor and Drive" 2019 IEEE/RSJ
        International Conference on Intelligent Robots and Systems (IROS). pp. 8012 -- 8019, Macau,
        China, November 4-8, 2019.
"""
