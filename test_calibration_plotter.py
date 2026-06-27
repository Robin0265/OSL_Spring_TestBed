import numpy as np
import matplotlib.pyplot as plt
# from model_params import *
import scipy.io
from Vision.plot_cal import circle_find_3_points
from Vision.video_reading_test import test, CircleAnnotator, CircleTracker
import os.path
from scipy.signal import find_peaks
from scipy.stats import linregress
from scipy.optimize import minimize


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

def _normalize_for_alignment(x):
    x = np.asarray(x)
    return (x - np.mean(x)) / (np.std(x) + 1e-12)

def _alignment_error(cam_time, cam_signals, enc_time, enc_signals, n_grid=2000):
    start_time = max(cam_time[0], enc_time[0])
    end_time = min(cam_time[-1], enc_time[-1])
    if end_time <= start_time:
        return np.inf

    common_time = np.linspace(start_time, end_time, n_grid)
    error = 0.0
    for cam_signal, enc_signal in zip(cam_signals, enc_signals):
        cam_res = np.interp(common_time, cam_time, _normalize_for_alignment(cam_signal))
        enc_res = np.interp(common_time, enc_time, _normalize_for_alignment(enc_signal))
        error += np.mean((cam_res - enc_res) ** 2)
    return error

def fit_camera_time_alignment(cam_time, cam_signals, enc_time, enc_signals):
    initial_scale = (enc_time[-1] - enc_time[0]) / (cam_time[-1] - cam_time[0])
    initial_params = np.array([initial_scale, enc_time[0] - initial_scale * cam_time[0]])

    def objective(params):
        scale, offset = params
        aligned_cam_time = scale * cam_time + offset
        return _alignment_error(aligned_cam_time, cam_signals, enc_time, enc_signals)

    before_error = objective(initial_params)
    result = minimize(
        objective,
        initial_params,
        method="L-BFGS-B",
        bounds=((0.95, 1.05), (-0.5, 0.5)),
    )
    if result.success and result.fun <= before_error:
        scale, offset = result.x
    else:
        scale, offset = initial_params
    after_error = objective((scale, offset))
    return scale * cam_time + offset, scale, offset, before_error, after_error

def refine_camera_time_warp(aligned_cam_time, cam_signals, enc_time, enc_signals,
        n_anchors=10, max_correction_s=0.02, n_grid=1800,
        roughness_weight=0.15, magnitude_weight=0.02):
    aligned_cam_time = np.asarray(aligned_cam_time, dtype=float)
    enc_time = np.asarray(enc_time, dtype=float)
    cam_signals = tuple(np.asarray(signal, dtype=float) for signal in cam_signals)
    enc_signals = tuple(np.asarray(signal, dtype=float) for signal in enc_signals)

    start_time = max(aligned_cam_time[0], enc_time[0])
    end_time = min(aligned_cam_time[-1], enc_time[-1])
    if end_time <= start_time or aligned_cam_time.size < 4:
        return aligned_cam_time, np.zeros_like(aligned_cam_time), np.inf, np.inf

    n_anchors = max(4, min(n_anchors, aligned_cam_time.size))
    anchor_time = np.linspace(start_time, end_time, n_anchors)
    common_time = np.linspace(start_time, end_time, n_grid)
    min_dt = np.min(np.diff(aligned_cam_time))
    before_error = _alignment_error(
        aligned_cam_time,
        cam_signals,
        enc_time,
        enc_signals,
        n_grid=n_grid,
    )

    shift_candidates = np.linspace(-max_correction_s, max_correction_s, 81)
    half_window = max(2.5, 0.75 * (end_time - start_time) / max(1, n_anchors - 1))
    anchor_correction = np.zeros(n_anchors, dtype=float)

    for anchor_idx, center_time in enumerate(anchor_time):
        local_start = max(start_time, center_time - half_window)
        local_end = min(end_time, center_time + half_window)
        local_time = common_time[(common_time >= local_start) & (common_time <= local_end)]
        if local_time.size < 20:
            continue

        best_score = None
        best_shift = 0.0
        for shift in shift_candidates:
            shifted_cam_time = aligned_cam_time + shift
            if local_time[0] < shifted_cam_time[0] or local_time[-1] > shifted_cam_time[-1]:
                continue

            score = 0.0
            n_used = 0
            for cam_signal, enc_signal in zip(cam_signals, enc_signals):
                cam_resampled = np.interp(local_time, shifted_cam_time, cam_signal)
                enc_resampled = np.interp(local_time, enc_time, enc_signal)
                cam_resampled = _normalize_for_alignment(cam_resampled)
                enc_resampled = _normalize_for_alignment(enc_resampled)
                if np.std(cam_resampled) < 1e-6 or np.std(enc_resampled) < 1e-6:
                    continue
                score += np.mean((cam_resampled - enc_resampled) ** 2)
                n_used += 1

            if n_used == 0:
                continue
            score /= n_used
            score += magnitude_weight * (shift / max_correction_s) ** 2
            if best_score is None or score < best_score:
                best_score = score
                best_shift = shift
        anchor_correction[anchor_idx] = best_shift

    if anchor_correction.size >= 5:
        anchor_correction = _smooth_signal(anchor_correction, window=5)
    elif anchor_correction.size >= 3:
        anchor_correction = _smooth_signal(anchor_correction, window=3)

    if anchor_correction.size >= 3 and roughness_weight > 0:
        initial_correction = anchor_correction.copy()

        def objective(candidate):
            candidate = np.asarray(candidate, dtype=float)
            correction = np.interp(
                aligned_cam_time,
                anchor_time,
                candidate,
                left=candidate[0],
                right=candidate[-1],
            )
            warped_time = aligned_cam_time + correction
            if np.any(np.diff(warped_time) <= 0.2 * min_dt):
                return 1e6
            data_fit = np.mean((candidate - initial_correction) ** 2) / (max_correction_s ** 2)
            roughness = np.mean(np.diff(candidate / max_correction_s, n=2) ** 2)
            magnitude = np.mean((candidate / max_correction_s) ** 2)
            return data_fit + roughness_weight * roughness + magnitude_weight * magnitude

        result = minimize(
            objective,
            initial_correction,
            method="L-BFGS-B",
            bounds=[(-max_correction_s, max_correction_s)] * n_anchors,
            options={"maxiter": 200},
        )
        if result.success:
            anchor_correction = result.x

    correction = np.interp(
        aligned_cam_time,
        anchor_time,
        anchor_correction,
        left=anchor_correction[0],
        right=anchor_correction[-1],
    )
    warped_time = aligned_cam_time + correction
    if np.any(np.diff(warped_time) <= 0.2 * min_dt):
        return aligned_cam_time, np.zeros_like(aligned_cam_time), before_error, before_error

    after_error = _alignment_error(
        warped_time,
        cam_signals,
        enc_time,
        enc_signals,
        n_grid=n_grid,
    )
    return warped_time, correction, before_error, after_error

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

def _signal_excursion(signal):
    signal = np.asarray(signal, dtype=float)
    return float(np.max(signal) - np.min(signal))

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

def _select_best_landmark_match(cam_time, cam_signals, enc_time, enc_signals):
    best = None
    for idx, (cam_signal, enc_signal) in enumerate(zip(cam_signals, enc_signals)):
        match = _match_landmark_sequences(cam_time, cam_signal, enc_time, enc_signal)
        if match is None:
            continue
        score = match["score"]
        if best is None or score > best["score"]:
            best = {
                "score": score,
                "channel_index": idx,
                "match": match,
            }
    return best

def _interval_alignment_error(aligned_cam_time, cam_signals, enc_time, enc_signals,
        start_time, end_time, n_grid=1200):
    if end_time <= start_time:
        return np.inf

    common_time = np.linspace(start_time, end_time, n_grid)
    error = 0.0
    n_used = 0
    for cam_signal, enc_signal in zip(cam_signals, enc_signals):
        cam_mask = (aligned_cam_time >= start_time) & (aligned_cam_time <= end_time)
        enc_mask = (enc_time >= start_time) & (enc_time <= end_time)
        if np.count_nonzero(cam_mask) < 3 or np.count_nonzero(enc_mask) < 3:
            continue

        cam_res = np.interp(common_time, aligned_cam_time[cam_mask], _normalize_for_alignment(cam_signal[cam_mask]))
        enc_res = np.interp(common_time, enc_time[enc_mask], _normalize_for_alignment(enc_signal[enc_mask]))
        error += np.mean((cam_res - enc_res) ** 2)
        n_used += 1

    if n_used == 0:
        return np.inf
    return error / n_used

def _select_best_event_pair_alignment(cam_time, cam_signals, enc_time, enc_signals):
    best = None
    cam_duration_total = cam_time[-1] - cam_time[0]
    enc_duration_total = enc_time[-1] - enc_time[0]
    min_duration = 0.20 * min(cam_duration_total, enc_duration_total)

    for channel_index, (cam_signal, enc_signal) in enumerate(zip(cam_signals, enc_signals)):
        cam_landmarks = _detect_signal_landmarks(
            cam_signal,
            prominence_ratio=0.03,
            min_distance_ratio=0.03,
        )
        enc_landmarks = _detect_signal_landmarks(
            enc_signal,
            prominence_ratio=0.03,
            min_distance_ratio=0.03,
        )
        if len(cam_landmarks) < 2 or len(enc_landmarks) < 2:
            continue

        for cam_pair_start in range(len(cam_landmarks) - 1):
            cam_start_idx, cam_start_sign = cam_landmarks[cam_pair_start]
            for cam_pair_end in range(cam_pair_start + 1, len(cam_landmarks)):
                cam_end_idx, cam_end_sign = cam_landmarks[cam_pair_end]
                if cam_start_sign == cam_end_sign:
                    continue
                cam_dt = cam_time[cam_end_idx] - cam_time[cam_start_idx]
                if cam_dt < min_duration:
                    continue

                for enc_pair_start in range(len(enc_landmarks) - 1):
                    enc_start_idx, enc_start_sign = enc_landmarks[enc_pair_start]
                    if enc_start_sign != cam_start_sign:
                        continue
                    for enc_pair_end in range(enc_pair_start + 1, len(enc_landmarks)):
                        enc_end_idx, enc_end_sign = enc_landmarks[enc_pair_end]
                        if enc_end_sign != cam_end_sign:
                            continue
                        enc_dt = enc_time[enc_end_idx] - enc_time[enc_start_idx]
                        if enc_dt < min_duration:
                            continue

                        scale = enc_dt / (cam_dt + 1e-12)
                        if scale < 0.85 or scale > 1.15:
                            continue
                        offset = enc_time[enc_start_idx] - scale * cam_time[cam_start_idx]
                        aligned_cam_time = scale * cam_time + offset
                        start_time = enc_time[enc_start_idx]
                        end_time = enc_time[enc_end_idx]
                        error = _interval_alignment_error(
                            aligned_cam_time,
                            cam_signals,
                            enc_time,
                            enc_signals,
                            start_time,
                            end_time,
                        )
                        if not np.isfinite(error):
                            continue

                        excursion = abs(enc_signal[enc_end_idx] - enc_signal[enc_start_idx])
                        duration = end_time - start_time
                        score = (error, -excursion, -duration)
                        if best is None or score < best["score"]:
                            best = {
                                "score": score,
                                "channel_index": channel_index,
                                "scale": scale,
                                "offset": offset,
                                "error": error,
                                "excursion": excursion,
                                "cam_start_idx": cam_start_idx,
                                "cam_end_idx": cam_end_idx,
                                "enc_start_idx": enc_start_idx,
                                "enc_end_idx": enc_end_idx,
                                "cam_landmark_count": len(cam_landmarks),
                                "enc_landmark_count": len(enc_landmarks),
                            }
    return best

def _select_best_matched_interval(match, enc_signal):
    cam_idx = np.asarray(match["cam_idx"], dtype=int)
    enc_idx = np.asarray(match["enc_idx"], dtype=int)
    if cam_idx.size < 2 or enc_idx.size < 2:
        raise ValueError("Need at least two matched landmarks to select a monotonic interval.")

    best = None
    for i in range(len(enc_idx) - 1):
        start_enc = enc_idx[i]
        end_enc = enc_idx[i + 1]
        start_cam = cam_idx[i]
        end_cam = cam_idx[i + 1]
        excursion = abs(enc_signal[end_enc] - enc_signal[start_enc])
        duration = end_enc - start_enc
        score = (excursion, duration)
        if best is None or score > best["score"]:
            best = {
                "score": score,
                "cam_start_idx": start_cam,
                "cam_end_idx": end_cam,
                "enc_start_idx": start_enc,
                "enc_end_idx": end_enc,
                "landmark_pair_index": i,
            }
    return best

def _wrap_to_pi(values):
    return (np.asarray(values, dtype=float) + np.pi) % (2 * np.pi) - np.pi

def _prepare_inverse_lookup(camera_values, encoder_values, max_bins=720,
        min_bin_count=3):
    camera_values = np.asarray(camera_values, dtype=float)
    encoder_values = np.asarray(encoder_values, dtype=float)

    valid = np.isfinite(camera_values) & np.isfinite(encoder_values)
    camera_values = camera_values[valid]
    encoder_values = encoder_values[valid]
    if camera_values.size < 2:
        raise ValueError("Not enough valid samples to build inverse lookup.")

    slope, intercept, _, _, _ = linregress(camera_values, encoder_values)
    n_bins = min(max_bins, max(20, camera_values.size // min_bin_count))
    edges = np.linspace(np.min(camera_values), np.max(camera_values), n_bins + 1)
    if np.allclose(edges[0], edges[-1]):
        raise ValueError("Inverse lookup needs nonzero camera angle range.")
    bin_index = np.clip(np.searchsorted(edges, camera_values, side="right") - 1, 0, n_bins - 1)

    binned_camera = []
    binned_encoder = []
    for idx in range(n_bins):
        in_bin = bin_index == idx
        if np.count_nonzero(in_bin) < min_bin_count:
            continue
        binned_camera.append(float(np.median(camera_values[in_bin])))
        binned_encoder.append(float(np.median(encoder_values[in_bin])))

    if len(binned_camera) < 2:
        order = np.argsort(camera_values)
        binned_camera = camera_values[order]
        binned_encoder = encoder_values[order]
    else:
        binned_camera = np.asarray(binned_camera, dtype=float)
        binned_encoder = np.asarray(binned_encoder, dtype=float)

    order = np.argsort(binned_camera)
    binned_camera = binned_camera[order]
    binned_encoder = binned_encoder[order]

    unique_camera = [binned_camera[0]]
    unique_encoder = [binned_encoder[0]]
    for camera_value, encoder_value in zip(binned_camera[1:], binned_encoder[1:]):
        if np.isclose(camera_value, unique_camera[-1]):
            unique_encoder[-1] = 0.5 * (unique_encoder[-1] + encoder_value)
            continue
        unique_camera.append(camera_value)
        unique_encoder.append(encoder_value)

    lookup = np.column_stack((unique_camera, unique_encoder))
    return {
        "lookup": lookup,
        "slope": slope,
        "intercept": intercept,
    }

def _apply_inverse_lookup(camera_values, inverse_model):
    camera_values = np.asarray(camera_values, dtype=float)
    lookup = inverse_model["lookup"]
    calibrated = np.interp(camera_values, lookup[:, 0], lookup[:, 1])

    left = camera_values < lookup[0, 0]
    right = camera_values > lookup[-1, 0]
    if lookup.shape[0] >= 2:
        left_slope = (lookup[1, 1] - lookup[0, 1]) / (lookup[1, 0] - lookup[0, 0] + 1e-12)
        right_slope = (lookup[-1, 1] - lookup[-2, 1]) / (lookup[-1, 0] - lookup[-2, 0] + 1e-12)
        calibrated[left] = lookup[0, 1] + left_slope * (camera_values[left] - lookup[0, 0])
        calibrated[right] = lookup[-1, 1] + right_slope * (camera_values[right] - lookup[-1, 0])
    return calibrated

def _smooth_signal(values, window=11):
    values = np.asarray(values, dtype=float)
    if values.size < 3:
        return values.copy()

    window = min(window, values.size)
    if window % 2 == 0:
        window -= 1
    if window < 3:
        return values.copy()

    kernel = np.ones(window, dtype=float) / window
    pad = window // 2
    padded = np.pad(values, (pad, pad), mode="edge")
    return np.convolve(padded, kernel, mode="valid")

def _fill_zero_direction(direction):
    direction = np.asarray(direction, dtype=float).copy()
    nonzero = np.flatnonzero(direction != 0)
    if nonzero.size == 0:
        direction[:] = 1.0
        return direction

    first = nonzero[0]
    direction[:first] = direction[first]
    for idx in range(first + 1, direction.size):
        if direction[idx] == 0:
            direction[idx] = direction[idx - 1]

    last = nonzero[-1]
    direction[last + 1:] = direction[last]
    return direction

def _motion_direction(values, time_values, deadband_ratio=0.04):
    values = np.asarray(values, dtype=float)
    time_values = np.asarray(time_values, dtype=float)
    if values.size < 2:
        return np.ones_like(values, dtype=float)

    velocity = np.gradient(values, time_values)
    velocity = _smooth_signal(velocity, window=15)
    finite_speed = np.abs(velocity[np.isfinite(velocity)])
    if finite_speed.size == 0:
        return np.ones_like(values, dtype=float)

    threshold = max(np.percentile(finite_speed, 70) * deadband_ratio, 1e-8)
    direction = np.zeros_like(values, dtype=float)
    direction[velocity > threshold] = 1.0
    direction[velocity < -threshold] = -1.0
    return _fill_zero_direction(direction)

def _prepare_directional_inverse_lookup(camera_values, encoder_values, time_values,
        direction_values=None, min_branch_samples=80):
    camera_values = np.asarray(camera_values, dtype=float)
    encoder_values = np.asarray(encoder_values, dtype=float)
    time_values = np.asarray(time_values, dtype=float)
    if direction_values is None:
        direction_values = _motion_direction(encoder_values, time_values)
    else:
        direction_values = np.asarray(direction_values, dtype=float)

    model = {
        "combined": _prepare_inverse_lookup(camera_values, encoder_values),
        "positive": None,
        "negative": None,
        "min_branch_samples": min_branch_samples,
    }

    for label, key in ((1.0, "positive"), (-1.0, "negative")):
        branch = direction_values == label
        if np.count_nonzero(branch) < min_branch_samples:
            continue
        model[key] = _prepare_inverse_lookup(camera_values[branch], encoder_values[branch])
    return model

def _apply_directional_inverse_lookup(camera_values, time_values, inverse_model,
        direction_values=None):
    camera_values = np.asarray(camera_values, dtype=float)
    time_values = np.asarray(time_values, dtype=float)
    if direction_values is None:
        direction_values = _motion_direction(camera_values, time_values)
    else:
        direction_values = _fill_zero_direction(np.sign(np.asarray(direction_values, dtype=float)))

    calibrated = _apply_inverse_lookup(camera_values, inverse_model["combined"])
    for label, key in ((1.0, "positive"), (-1.0, "negative")):
        branch_model = inverse_model.get(key)
        if branch_model is None:
            continue
        branch = direction_values == label
        if np.any(branch):
            calibrated[branch] = _apply_inverse_lookup(camera_values[branch], branch_model)
    return calibrated

def _directional_inverse_rows(inverse_model):
    rows = []
    for label, key in ((0.0, "combined"), (1.0, "positive"), (-1.0, "negative")):
        branch_model = inverse_model.get(key)
        if branch_model is None:
            continue
        lookup = branch_model["lookup"]
        rows.append(np.column_stack((
            np.full(len(lookup), label),
            lookup,
            np.full(len(lookup), branch_model["slope"]),
            np.full(len(lookup), branch_model["intercept"]),
        )))
    if not rows:
        return np.empty((0, 5), dtype=float)
    return np.vstack(rows)

def _save_directional_inverse_model(path, inverse_model):
    arrays = {}
    for key in ("combined", "positive", "negative"):
        branch_model = inverse_model.get(key)
        if branch_model is None:
            arrays[key + "_lookup"] = np.empty((0, 2), dtype=float)
            arrays[key + "_slope"] = np.asarray(np.nan)
            arrays[key + "_intercept"] = np.asarray(np.nan)
            continue
        arrays[key + "_lookup"] = branch_model["lookup"]
        arrays[key + "_slope"] = np.asarray(branch_model["slope"])
        arrays[key + "_intercept"] = np.asarray(branch_model["intercept"])
    np.savez(path, **arrays)

def _load_directional_inverse_model(path):
    data = np.load(path)
    model = {}
    for key in ("combined", "positive", "negative"):
        lookup = np.asarray(data[key + "_lookup"], dtype=float)
        slope = float(data[key + "_slope"])
        intercept = float(data[key + "_intercept"])
        if lookup.size == 0 or not np.isfinite(slope) or not np.isfinite(intercept):
            model[key] = None
            continue
        model[key] = {
            "lookup": lookup,
            "slope": slope,
            "intercept": intercept,
        }
    return model

def _print_residual_stats(label, residual, mask=None):
    residual = np.asarray(residual, dtype=float)
    if mask is not None:
        residual = residual[np.asarray(mask, dtype=bool)]
    residual = residual[np.isfinite(residual)]
    if residual.size == 0:
        print("%s residual: no samples" % label)
        return

    print(
        "%s residual: n=%d mean=%.6g med_abs=%.6g p95_abs=%.6g max_abs=%.6g rmse=%.6g"
        % (
            label,
            residual.size,
            np.mean(residual),
            np.median(np.abs(residual)),
            np.percentile(np.abs(residual), 95),
            np.max(np.abs(residual)),
            np.sqrt(np.mean(residual ** 2)),
        )
    )

def _build_directional_inverse_calibration(aligned_cam_time, blue_cam_angs, red_cam_angs,
        enc_time, blue_enc_angs, red_enc_angs):
    valid_calibration_time = (
        (enc_time >= aligned_cam_time[0]) &
        (enc_time <= aligned_cam_time[-1])
    )
    inverse_time = enc_time[valid_calibration_time]
    blue_enc_rng = blue_enc_angs[valid_calibration_time]
    red_enc_rng = red_enc_angs[valid_calibration_time]
    if inverse_time.size < 2:
        raise ValueError("No overlapping camera/encoder samples for inverse calibration.")

    blue_cam_rng_res = np.interp(inverse_time, aligned_cam_time, blue_cam_angs)
    red_cam_rng_res = np.interp(inverse_time, aligned_cam_time, red_cam_angs)
    blue_direction = _motion_direction(blue_enc_rng, inverse_time)
    red_direction = _motion_direction(red_enc_rng, inverse_time)

    inv_blue = _prepare_directional_inverse_lookup(
        blue_cam_rng_res,
        blue_enc_rng,
        inverse_time,
        direction_values=blue_direction,
    )
    inv_red = _prepare_directional_inverse_lookup(
        red_cam_rng_res,
        red_enc_rng,
        inverse_time,
        direction_values=red_direction,
    )
    return {
        "inverse_time": inverse_time,
        "blue_enc_rng": blue_enc_rng,
        "red_enc_rng": red_enc_rng,
        "blue_cam_rng_res": blue_cam_rng_res,
        "red_cam_rng_res": red_cam_rng_res,
        "inv_blue": inv_blue,
        "inv_red": inv_red,
    }

def _estimate_time_correction_from_residuals(aligned_cam_time, blue_cam_cal,
        red_cam_cal, enc_time, blue_enc_angs, red_enc_angs,
        max_correction_s=0.02, n_anchors=10):
    valid = (aligned_cam_time >= enc_time[0]) & (aligned_cam_time <= enc_time[-1])
    plot_time = aligned_cam_time[valid]
    if plot_time.size < 10:
        return np.zeros_like(aligned_cam_time)

    blue_enc_res = np.interp(plot_time, enc_time, blue_enc_angs)
    red_enc_res = np.interp(plot_time, enc_time, red_enc_angs)
    blue_residual = blue_cam_cal[valid] - blue_enc_res
    red_residual = red_cam_cal[valid] - red_enc_res

    blue_enc_velocity = np.gradient(blue_enc_angs, enc_time)
    red_enc_velocity = np.gradient(red_enc_angs, enc_time)
    blue_velocity_res = np.interp(plot_time, enc_time, blue_enc_velocity)
    red_velocity_res = np.interp(plot_time, enc_time, red_enc_velocity)

    velocity_scale = np.nanpercentile(
        np.abs(np.hstack((blue_velocity_res, red_velocity_res))),
        70,
    )
    min_velocity = max(0.05 * velocity_scale, 1e-5)
    correction_samples = []
    for residual, velocity in (
            (blue_residual, blue_velocity_res),
            (red_residual, red_velocity_res)):
        valid_channel = (
            np.isfinite(residual) &
            np.isfinite(velocity) &
            (np.abs(velocity) >= min_velocity)
        )
        correction = np.full_like(residual, np.nan)
        correction[valid_channel] = residual[valid_channel] / velocity[valid_channel]
        correction_samples.append(correction)

    correction_stack = np.vstack(correction_samples)
    finite_column = np.any(np.isfinite(correction_stack), axis=0)
    correction_samples = np.full(correction_stack.shape[1], np.nan)
    correction_samples[finite_column] = np.nanmedian(
        correction_stack[:, finite_column],
        axis=0,
    )
    correction_samples = np.clip(correction_samples, -max_correction_s, max_correction_s)

    finite = np.isfinite(correction_samples)
    if np.count_nonzero(finite) < 5:
        return np.zeros_like(aligned_cam_time)

    anchor_time = np.linspace(plot_time[0], plot_time[-1], n_anchors)
    edges = np.linspace(plot_time[0], plot_time[-1], n_anchors + 1)
    anchor_correction = np.zeros(n_anchors, dtype=float)
    finite_time = plot_time[finite]
    finite_correction = correction_samples[finite]
    for idx in range(n_anchors):
        in_bin = (
            (plot_time >= edges[idx]) &
            (plot_time <= edges[idx + 1]) &
            finite
        )
        if np.count_nonzero(in_bin) < 5:
            nearest = np.argmin(np.abs(finite_time - anchor_time[idx]))
            anchor_correction[idx] = finite_correction[nearest]
            continue
        anchor_correction[idx] = np.median(correction_samples[in_bin])

    if n_anchors >= 5:
        anchor_correction = _smooth_signal(anchor_correction, window=5)
    elif n_anchors >= 3:
        anchor_correction = _smooth_signal(anchor_correction, window=3)
    anchor_correction = np.clip(anchor_correction, -max_correction_s, max_correction_s)

    return np.interp(
        aligned_cam_time,
        anchor_time,
        anchor_correction,
        left=anchor_correction[0],
        right=anchor_correction[-1],
    )

def _rolling_median(values, window):
    values = np.asarray(values, dtype=float)
    if values.size == 0:
        return values.copy()
    window = max(1, int(window))
    if window % 2 == 0:
        window += 1
    half_window = window // 2
    result = np.empty_like(values)
    for idx in range(values.size):
        start_idx = max(0, idx - half_window)
        end_idx = min(values.size, idx + half_window + 1)
        result[idx] = np.median(values[start_idx:end_idx])
    return result

def _short_runs(mask, max_run_length):
    mask = np.asarray(mask, dtype=bool)
    if mask.size == 0:
        return []
    changes = np.flatnonzero(np.diff(mask.astype(int)) != 0) + 1
    starts = np.r_[0, changes]
    ends = np.r_[changes, mask.size]
    runs = []
    for start_idx, end_idx in zip(starts, ends):
        if not mask[start_idx]:
            continue
        if end_idx - start_idx <= max_run_length:
            runs.append((start_idx, end_idx))
    return runs

def _candidate_angle_outlier_mask(angle, time_values, window=9,
        threshold_multiplier=8.0, max_bad_velocity_run=2):
    angle = np.asarray(angle, dtype=float)
    time_values = np.asarray(time_values, dtype=float)
    sample_mask = np.zeros(angle.size, dtype=bool)
    if angle.size < window + 2:
        return sample_mask

    velocity = np.diff(angle) / (np.diff(time_values) + 1e-12)
    local_velocity = _rolling_median(velocity, window)
    velocity_error = velocity - local_velocity
    median_error = np.median(velocity_error)
    mad = np.median(np.abs(velocity_error - median_error))
    velocity_scale = np.percentile(np.abs(velocity), 90)
    threshold = max(threshold_multiplier * 1.4826 * mad, 1.75 * velocity_scale, 1e-6)
    bad_velocity = np.abs(velocity_error) > threshold

    for start_idx, end_idx in _short_runs(bad_velocity, max_bad_velocity_run):
        sample_start = max(1, start_idx + 1)
        sample_end = min(angle.size - 1, end_idx + 1)
        sample_mask[sample_start:sample_end + 1] = True
    return sample_mask

def _repair_camera_angle_outliers(time_values, blue_angle, red_angle):
    blue_candidate = _candidate_angle_outlier_mask(blue_angle, time_values)
    red_candidate = _candidate_angle_outlier_mask(red_angle, time_values)

    # If both channels are suspicious at the same instant, leave it alone; that is
    # more likely alignment or real motion than a single-channel tracker glitch.
    both_candidate = blue_candidate & red_candidate
    blue_repair_mask = blue_candidate & ~both_candidate
    red_repair_mask = red_candidate & ~both_candidate

    def repair(angle, repair_mask):
        repaired = np.asarray(angle, dtype=float).copy()
        if not np.any(repair_mask):
            return repaired
        good = ~repair_mask
        if np.count_nonzero(good) < 2:
            return repaired
        repaired[repair_mask] = np.interp(
            time_values[repair_mask],
            time_values[good],
            repaired[good],
        )
        return repaired

    return (
        repair(blue_angle, blue_repair_mask),
        repair(red_angle, red_repair_mask),
        blue_repair_mask,
        red_repair_mask,
        both_candidate,
    )

conversion_to_spring_frame_degrees= lambda x: x/(45.5111*50.)
ticks_to_motor_radians = lambda x: x*(1)
# torque_sensor_callibrated_volts_2_Nm = np.vectorize(lambda x: 1.0014*x+0.0044 if x>=0 else .9998*x + 0.0018)
torque_sensor_callibrated_volts_2_Nm = np.vectorize(lambda x: 20*x)

class SEATestbedPlotter(object):
    def __init__(self, act0_file, act1_file, adc0_file=None):

        act0 = self._load_log(act0_file)
        act1 = self._load_log(act1_file)
        self.has_adc = self.has_traj = self.has_ctrl = False
        self.camera_start_elapsed = self._estimate_camera_start_elapsed(act0)

        if adc0_file: 
            adc0 = np.loadtxt(adc0_file, delimiter=",", skiprows=1)
            self.has_adc = True

        self._data=dict(headers=[],data=[])
        self._line_index=-1

        self.initial_angle_offset = 0

        if self._is_combined_motor_log(act0) and self._same_file(act0_file, act1_file):
            act0_pi_time = self._get_col(act0, "elapsed_time", 3, "timestamp")
            act1_pi_time = act0_pi_time.copy()
            init_pi_time = min([act0_pi_time[0], act1_pi_time[0], adc0[0,0]]) if self.has_adc else act0_pi_time[0]

            a0_x = self._get_col(act0, "knee[DephyActuator].output_position", 10)
            a0_xd = self._get_col(act0, "knee[DephyActuator].output_velocity", 11)
            a1_x = self._get_col(act1, "ankle[DephyActuator].output_position", 17)
            a1_xd = self._get_col(act1, "ankle[DephyActuator].output_velocity", 18)

            self.add_line("a0_t", "pi_time", act0_pi_time-init_pi_time)
            self.add_line("a0_ts", "State time", self._get_col(act0, "monotonic_time_ns", 2, "elapsed_time"))
            self.add_line("a0_x", "Motor enc angle", -a0_x+a0_x[0])
            self.add_line("a0_xd", "Motor enc velocity", a0_xd)
            self.add_line("a0_xdd", "Motor enc acceleration", self._differentiate(a0_xd, act0_pi_time))
            self.add_line("a0_vm", "Motor deph voltage", self._get_col(act0, "knee[DephyActuator].motor_voltage", 13))
            self.add_line("a0_im", "Motor deph current", self._get_col(act0, "knee[DephyActuator].motor_current", 14))
            self.add_line("a0_vb", "Battery deph voltage", self._get_col(act0, "knee[DephyActuator].battery_voltage", 15))
            self.add_line("a0_ib", "Battery deph current", self._get_col(act0, "knee[DephyActuator].battery_current", 16))

            self.add_line("a1_t", "pi_time", act1_pi_time-init_pi_time)
            self.add_line("a1_ts", "State time", self._get_col(act1, "monotonic_time_ns", 2, "elapsed_time"))
            self.add_line("a1_x", "Motor enc angle", -a1_x+a1_x[0])
            self.add_line("a1_xd", "Motor enc velocity", a1_xd)
            self.add_line("a1_xdd", "Motor enc acceleration", self._differentiate(a1_xd, act1_pi_time))
            self.add_line("a1_vm", "Motor deph voltage", self._get_col(act1, "ankle[DephyActuator].motor_voltage", 20))
            self.add_line("a1_im", "Motor deph current", self._get_col(act1, "ankle[DephyActuator].motor_current", 21))
            self.add_line("a1_vb", "Battery deph voltage", self._get_col(act1, "ankle[DephyActuator].battery_voltage", 22))
            self.add_line("a1_ib", "Battery deph current", self._get_col(act1, "ankle[DephyActuator].battery_current", 23))
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
        return float(np.median(np.hstack(finite_candidates)))

    @staticmethod
    def _is_combined_motor_log(data):
        headers = set(data.get("headers", []))
        return {
            "knee[DephyActuator].output_position",
            "ankle[DephyActuator].output_position",
        }.issubset(headers)

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
        if self.has_adc: self.add_line("tau", "torque sensor torque signal", torque_sensor_callibrated_volts_2_Nm(self.futek_v))
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


def main(cal_folder,inner_mask,outer_mask):
    # Create red_cal and blue_cal if they aren't in folder
    if not os.path.exists('blue_cal.csv') or not os.path.exists('red_cal.csv'):
        print('Do the plot_cal thing')
        test(file = cal_folder + '/Calibration260627_191953.h264',
            inner_mask_loc = inner_mask,
            outer_mask_loc = outer_mask,
            pre_mask_save_loc = cal_folder + '/camera_calibration_pre_mask.png',
            red_cal_save_loc = 'red_cal.csv',
            blue_cal_save_loc = 'blue_cal.csv')

    # Create circle_cal if not in folder
    if not os.path.exists('circle_cal.csv'):
        datb = np.loadtxt('blue_cal.csv', delimiter=',')
        x0, y0, SinvUT =circle_find_3_points(datb, color='blue')
        thetas = np.linspace(0,np.pi*2,1000)
        circ = np.linalg.solve(SinvUT, np.block([[np.cos(thetas)],[np.sin(thetas)]]))+np.array([[x0],[y0]])
        plt.plot(circ[0,:], circ[1,:],'b')
        circle_cal1 = np.hstack((np.array([x0, y0]),SinvUT.flatten()))

        datr = np.loadtxt('red_cal.csv', delimiter=',')
        x0, y0, SinvUT =circle_find_3_points(datr, color='red')
        thetas = np.linspace(0,np.pi*2,1000)
        circ = np.linalg.solve(SinvUT, np.block([[np.cos(thetas)],[np.sin(thetas)]]))+np.array([[x0],[y0]])
        plt.plot(circ[0,:], circ[1,:],'r')
        circle_cal2 = np.hstack((np.array([x0, y0]),SinvUT.flatten()))
        circle_cal = np.vstack((circle_cal1,circle_cal2))

        with open('circle_cal.csv', 'w') as f:
            np.savetxt(f, circle_cal, fmt='%.7f', delimiter=", ")
        plt.show()

    # Calculate camera_angs 
    if not os.path.exists(cal_folder + '/camera_enabled_angles.csv'):

        blue_cam_angs, red_cam_angs, cam_time = test(file = cal_folder + '/Calibration260627_191953.h264',
                                                    inner_mask_loc = inner_mask,
                                                    outer_mask_loc = outer_mask,
                                                    pre_mask_save_loc = cal_folder + '/camera_calibration_pre_mask.png',
                                                    red_cal_save_loc = None,
                                                    blue_cal_save_loc = None,
                                                    red_diagnostic_save_loc = cal_folder + '/red_camera_diagnostics.csv',
                                                    blue_diagnostic_save_loc = cal_folder + '/blue_camera_diagnostics.csv',
                                                    display=True)

        blue_cam_angs = blue_cam_angs - blue_cam_angs[0]
        red_cam_angs = red_cam_angs - red_cam_angs[0]
        cam_enabled_angs = np.vstack((cam_time,blue_cam_angs,red_cam_angs)).T

        with open(cal_folder + '/camera_enabled_angles.csv', 'w') as f:
            np.savetxt(f, cam_enabled_angs, fmt='%.7f', delimiter=", ")
    else:
        cam_enabled_angs = np.loadtxt(cal_folder + '/camera_enabled_angles.csv', delimiter=',')
        cam_time = cam_enabled_angs[:,0]
        blue_cam_angs = cam_enabled_angs[:,1]
        red_cam_angs = cam_enabled_angs[:,2]

    # Read encoder_angs from SEA_Testbed_Plotter
    stp = SEATestbedPlotter(cal_folder + '/Calibration260627_191953.csv',cal_folder + '/Calibration260627_191953.csv')
    red_enc_angs = - stp.theta_1
    blue_enc_angs = - stp.theta_0
    enc_time = stp.a0_t

    if stp.camera_start_elapsed is not None:
        cam_time = stp.camera_start_elapsed + cam_time
        print("camera start elapsed anchor: %.6f s" % stp.camera_start_elapsed)

    plt.figure(1)
    plt.plot(cam_time,red_cam_angs,'r')
    plt.plot(cam_time,blue_cam_angs,'b')
    plt.plot(enc_time,blue_enc_angs,'c')
    plt.plot(enc_time,red_enc_angs,'m')
    
    plt.legend([
        'red_cam_angs', 'blue_cam_angs',
        'blue_enc_angs', 'red_enc_angs',
        # 'red_cam_det', 'red_enc_det',
        ])

    cam_start_ind, cam_end_ind = _find_active_window(
        cam_time,
        (red_cam_angs, blue_cam_angs),
    )
    enc_start_ind, enc_end_ind = _find_active_window(
        enc_time,
        (red_enc_angs, blue_enc_angs),
    )

    new_cam_time = cam_time[cam_start_ind:cam_end_ind]
    new_blue_cam_angs = blue_cam_angs[cam_start_ind:cam_end_ind]
    new_red_cam_angs = red_cam_angs[cam_start_ind:cam_end_ind]
    new_enc_time = enc_time[enc_start_ind:enc_end_ind]
    new_blue_enc_angs = blue_enc_angs[enc_start_ind:enc_end_ind]
    new_red_enc_angs = red_enc_angs[enc_start_ind:enc_end_ind]
    best_landmark = _select_best_landmark_match(
        new_cam_time,
        (new_red_cam_angs, new_blue_cam_angs),
        new_enc_time,
        (new_red_enc_angs, new_blue_enc_angs),
    )
    if best_landmark is not None:
        landmark_match = best_landmark["match"]
        cam_anchor_time = new_cam_time[landmark_match["cam_idx"]]
        enc_anchor_time = new_enc_time[landmark_match["enc_idx"]]
        aligned_cam_time = _piecewise_map_camera_time(new_cam_time, cam_anchor_time, enc_anchor_time)
        print(
            "landmark alignment: channel=%d matched=%d camera_landmarks=%d encoder_landmarks=%d spacing_error=%.6g"
            % (
                best_landmark["channel_index"],
                len(cam_anchor_time),
                landmark_match["cam_count"],
                landmark_match["enc_count"],
                landmark_match["spacing_error"],
            )
        )
        selected_interval = _select_best_matched_interval(
            landmark_match,
            (new_red_enc_angs, new_blue_enc_angs)[best_landmark["channel_index"]],
        )
        print(
            "selected interval: channel=%d pair=%d cam[%d:%d] enc[%d:%d]"
            % (
                best_landmark["channel_index"],
                selected_interval["landmark_pair_index"],
                selected_interval["cam_start_idx"],
                selected_interval["cam_end_idx"],
                selected_interval["enc_start_idx"],
                selected_interval["enc_end_idx"],
            )
        )
    else:
        event_pair = _select_best_event_pair_alignment(
            new_cam_time,
            (new_red_cam_angs, new_blue_cam_angs),
            new_enc_time,
            (new_red_enc_angs, new_blue_enc_angs),
        )
        if event_pair is not None:
            aligned_cam_time = event_pair["scale"] * new_cam_time + event_pair["offset"]
            selected_interval = {
                "cam_start_idx": event_pair["cam_start_idx"],
                "cam_end_idx": event_pair["cam_end_idx"],
                "enc_start_idx": event_pair["enc_start_idx"],
                "enc_end_idx": event_pair["enc_end_idx"],
                "landmark_pair_index": -1,
            }
            print(
                "event-pair alignment: channel=%d scale=%.8f offset=%.6f error=%.6g excursion=%.6g cam_landmarks=%d enc_landmarks=%d cam[%d:%d] enc[%d:%d]"
                % (
                    event_pair["channel_index"],
                    event_pair["scale"],
                    event_pair["offset"],
                    event_pair["error"],
                    event_pair["excursion"],
                    event_pair["cam_landmark_count"],
                    event_pair["enc_landmark_count"],
                    event_pair["cam_start_idx"],
                    event_pair["cam_end_idx"],
                    event_pair["enc_start_idx"],
                    event_pair["enc_end_idx"],
                )
            )
        else:
            aligned_cam_time, alignment_scale, alignment_offset, align_error_before, align_error_after = fit_camera_time_alignment(
                new_cam_time,
                (new_red_cam_angs, new_blue_cam_angs),
                new_enc_time,
                (new_red_enc_angs, new_blue_enc_angs),
            )
            print(
                "fallback affine alignment: scale=%.8f, offset=%.6f s, error %.6g -> %.6g"
                % (alignment_scale, alignment_offset, align_error_before, align_error_after)
            )
            cam_start_idx = int(np.argmax(new_red_cam_angs))
            cam_end_idx = int(np.argmin(new_red_cam_angs))
            enc_start_idx = int(np.argmax(new_red_enc_angs))
            enc_end_idx = int(np.argmin(new_red_enc_angs))
            if cam_end_idx < cam_start_idx:
                cam_start_idx, cam_end_idx = cam_end_idx, cam_start_idx
            if enc_end_idx < enc_start_idx:
                enc_start_idx, enc_end_idx = enc_end_idx, enc_start_idx
            selected_interval = {
                "cam_start_idx": cam_start_idx,
                "cam_end_idx": cam_end_idx,
                "enc_start_idx": enc_start_idx,
                "enc_end_idx": enc_end_idx,
                "landmark_pair_index": -1,
            }

    coarse_aligned_cam_time = aligned_cam_time.copy()
    aligned_cam_time, time_warp_correction, time_warp_error_before, time_warp_error_after = refine_camera_time_warp(
        aligned_cam_time,
        (np.unwrap(new_red_cam_angs), np.unwrap(new_blue_cam_angs)),
        new_enc_time,
        (np.unwrap(new_red_enc_angs), np.unwrap(new_blue_enc_angs)),
    )
    print(
        "time-warp refinement: correction %.3f..%.3f ms, error %.6g -> %.6g"
        % (
            np.min(time_warp_correction) * 1e3,
            np.max(time_warp_correction) * 1e3,
            time_warp_error_before,
            time_warp_error_after,
        )
    )

    plot_origin = min(aligned_cam_time[0], new_enc_time[0])
    aligned_cam_time_plot = aligned_cam_time - plot_origin
    new_enc_time_plot = new_enc_time - plot_origin
    
    plt.figure(2)
    plt.plot(aligned_cam_time_plot,new_red_cam_angs,'r')
    plt.plot(aligned_cam_time_plot,new_blue_cam_angs,'b')

    plt.plot(new_enc_time_plot,new_blue_enc_angs,'c')
    plt.plot(new_enc_time_plot,new_red_enc_angs,'m')

    plt.legend([
        'red_cam_angs', 'blue_cam_angs',
        'blue_enc_angs', 'red_enc_angs',
        # 'red_cam_det', 'red_enc_det',
        ])
    plt.show()
    # plt.plot(cam_time,red_cam_det)
    # plt.plot(enc_time,red_enc_det)


    # Compare camera_angs and encoder_angs to get inverse_calibration
    cam_slice = slice(selected_interval["cam_start_idx"], selected_interval["cam_end_idx"] + 1)
    enc_slice = slice(selected_interval["enc_start_idx"], selected_interval["enc_end_idx"] + 1)
    cam_time_rng = aligned_cam_time[cam_slice]
    enc_time_rng = new_enc_time[enc_slice]
    blue_cam_rng = new_blue_cam_angs[cam_slice]
    red_cam_rng = new_red_cam_angs[cam_slice]
    blue_enc_rng = new_blue_enc_angs[enc_slice]
    red_enc_rng = new_red_enc_angs[enc_slice]

    unwrapped_blue_cam_angs = np.unwrap(new_blue_cam_angs)
    unwrapped_red_cam_angs = np.unwrap(new_red_cam_angs)
    unwrapped_blue_enc_angs = np.unwrap(new_blue_enc_angs)
    unwrapped_red_enc_angs = np.unwrap(new_red_enc_angs)
    (
        unwrapped_blue_cam_angs,
        unwrapped_red_cam_angs,
        blue_repair_mask,
        red_repair_mask,
        shared_repair_candidate_mask,
    ) = _repair_camera_angle_outliers(
        aligned_cam_time,
        unwrapped_blue_cam_angs,
        unwrapped_red_cam_angs,
    )
    print(
        "camera angle outlier repair: blue %d/%d, red %d/%d, shared skipped %d"
        % (
            np.count_nonzero(blue_repair_mask),
            blue_repair_mask.size,
            np.count_nonzero(red_repair_mask),
            red_repair_mask.size,
            np.count_nonzero(shared_repair_candidate_mask),
        )
    )

    total_residual_time_correction = np.zeros_like(aligned_cam_time)
    for residual_refinement_idx in range(2):
        inverse_calibration = _build_directional_inverse_calibration(
            aligned_cam_time,
            unwrapped_blue_cam_angs,
            unwrapped_red_cam_angs,
            new_enc_time,
            unwrapped_blue_enc_angs,
            unwrapped_red_enc_angs,
        )
        inv_blue = inverse_calibration["inv_blue"]
        inv_red = inverse_calibration["inv_red"]
        preliminary_blue_cam_ang_cal = _apply_directional_inverse_lookup(
            unwrapped_blue_cam_angs,
            aligned_cam_time,
            inv_blue,
        )
        preliminary_red_cam_ang_cal = _apply_directional_inverse_lookup(
            unwrapped_red_cam_angs,
            aligned_cam_time,
            inv_red,
        )
        residual_time_correction = _estimate_time_correction_from_residuals(
            aligned_cam_time,
            preliminary_blue_cam_ang_cal,
            preliminary_red_cam_ang_cal,
            new_enc_time,
            unwrapped_blue_enc_angs,
            unwrapped_red_enc_angs,
        )
        bounded_total_correction = np.clip(
            total_residual_time_correction + residual_time_correction,
            -0.02,
            0.02,
        )
        residual_time_correction = bounded_total_correction - total_residual_time_correction
        total_residual_time_correction = bounded_total_correction
        aligned_cam_time = aligned_cam_time + residual_time_correction
        print(
            "residual time correction pass %d: step %.3f..%.3f ms, total %.3f..%.3f ms"
            % (
                residual_refinement_idx + 1,
                np.min(residual_time_correction) * 1e3,
                np.max(residual_time_correction) * 1e3,
                np.min(total_residual_time_correction) * 1e3,
                np.max(total_residual_time_correction) * 1e3,
            )
        )
    plot_origin = min(aligned_cam_time[0], new_enc_time[0])

    inverse_calibration = _build_directional_inverse_calibration(
        aligned_cam_time,
        unwrapped_blue_cam_angs,
        unwrapped_red_cam_angs,
        new_enc_time,
        unwrapped_blue_enc_angs,
        unwrapped_red_enc_angs,
    )
    inverse_time = inverse_calibration["inverse_time"]
    blue_enc_rng = inverse_calibration["blue_enc_rng"]
    red_enc_rng = inverse_calibration["red_enc_rng"]
    blue_cam_rng_res = inverse_calibration["blue_cam_rng_res"]
    red_cam_rng_res = inverse_calibration["red_cam_rng_res"]
    inv_blue = inverse_calibration["inv_blue"]
    inv_red = inverse_calibration["inv_red"]

    def _lookup_size(inverse_model, key):
        branch = inverse_model.get(key)
        if branch is None:
            return 0
        return len(branch["lookup"])

    print(
        "inverse lookup sizes: blue n=%d combined=%d positive=%d negative=%d, red n=%d combined=%d positive=%d negative=%d"
        % (
            len(blue_cam_rng_res),
            _lookup_size(inv_blue, "combined"),
            _lookup_size(inv_blue, "positive"),
            _lookup_size(inv_blue, "negative"),
            len(red_cam_rng_res),
            _lookup_size(inv_red, "combined"),
            _lookup_size(inv_red, "positive"),
            _lookup_size(inv_red, "negative"),
        )
    )

    # plt.legend([
    #     # 'red_cam_angs', 'blue_cam_angs',
    #     'Camera Measurement - Outer Ring', 
    #     'Encoder Measurement - Outer Ring',  
    #     # 'red_cam_det', 'red_enc_det',
    #     ])
    # plt.show()

    plt.figure(3)
    plt.plot(blue_cam_rng_res,blue_enc_rng)
    plt.plot(red_cam_rng_res,red_enc_rng)

    slope, intercept, r_value, p_value, std_err = linregress(blue_cam_rng_res,blue_enc_rng)
    print('blue linregress: ',slope, intercept, r_value)
    slope, intercept, r_value, p_value, std_err = linregress(red_cam_rng_res,red_enc_rng)
    print('red linregress: ',slope, intercept, r_value)

    with open(cal_folder + '/inv_blue.csv', 'w') as f:
        np.savetxt(
            f,
            _directional_inverse_rows(inv_blue),
            fmt='%.7f',
            delimiter=", ",
            header="direction_branch, camera_angle_unwrapped, encoder_angle, affine_slope_diagnostic, affine_intercept_diagnostic",
        )
    with open(cal_folder + '/inv_red.csv', 'w') as f:
        np.savetxt(
            f,
            _directional_inverse_rows(inv_red),
            fmt='%.7f',
            delimiter=", ",
            header="direction_branch, camera_angle_unwrapped, encoder_angle, affine_slope_diagnostic, affine_intercept_diagnostic",
        )
    _save_directional_inverse_model(cal_folder + '/inv_blue_directional.npz', inv_blue)
    _save_directional_inverse_model(cal_folder + '/inv_red_directional.npz', inv_red)

    plt.figure(4)
    plt.plot(blue_cam_rng_res,blue_enc_rng-blue_cam_rng_res,'b')
    plt.plot(red_cam_rng_res,red_enc_rng-red_cam_rng_res,'r')
    
    blue_cam_ang_cal = _apply_directional_inverse_lookup(
        unwrapped_blue_cam_angs,
        aligned_cam_time,
        inv_blue,
    )
    red_cam_ang_cal = _apply_directional_inverse_lookup(
        unwrapped_red_cam_angs,
        aligned_cam_time,
        inv_red,
    )
    
    valid_aligned = (aligned_cam_time >= new_enc_time[0]) & (aligned_cam_time <= new_enc_time[-1])
    plot_time = aligned_cam_time[valid_aligned]
    plot_time_rel = plot_time - plot_origin
    red_cam_ang_cal_plot = red_cam_ang_cal[valid_aligned]
    blue_cam_ang_cal_plot = blue_cam_ang_cal[valid_aligned]
    red_enc_ang_res = np.interp(plot_time, new_enc_time, new_red_enc_angs)
    blue_enc_ang_res = np.interp(plot_time, new_enc_time, new_blue_enc_angs)
    plt.figure(5)
    plt.plot(plot_time_rel, 
             red_cam_ang_cal_plot, 'r')
    plt.plot(plot_time_rel, 
             red_enc_ang_res, 'b')
    plt.plot(plot_time_rel, 
             blue_cam_ang_cal_plot, 'c')
    plt.plot(plot_time_rel,
             blue_enc_ang_res, 'g')
    
    plt.figure(6)
    red_residual = red_cam_ang_cal_plot - red_enc_ang_res
    blue_residual = blue_cam_ang_cal_plot - blue_enc_ang_res
    red_residual_deg = red_residual * 180 / np.pi
    blue_residual_deg = blue_residual * 180 / np.pi
    blue_plot_direction = _motion_direction(unwrapped_blue_cam_angs[valid_aligned], plot_time)
    red_plot_direction = _motion_direction(unwrapped_red_cam_angs[valid_aligned], plot_time)
    _print_residual_stats("blue all deg", blue_residual_deg)
    _print_residual_stats("blue positive deg", blue_residual_deg, blue_plot_direction > 0)
    _print_residual_stats("blue negative deg", blue_residual_deg, blue_plot_direction < 0)
    _print_residual_stats("red all deg", red_residual_deg)
    _print_residual_stats("red positive deg", red_residual_deg, red_plot_direction > 0)
    _print_residual_stats("red negative deg", red_residual_deg, red_plot_direction < 0)
    plt.plot(plot_time_rel, 
             red_residual_deg, 'r')
    plt.plot(plot_time_rel,
             blue_residual_deg, 'b')
    plt.legend(['red residual', 'blue residual'])
    plt.xlabel('Time (s)')
    plt.ylabel('Camera inverse minus encoder (deg)')
    plt.title('Direction-aware inverse mapping residual')
    plt.show()


if __name__ == '__main__':
    # root_folder = "/home/gray/wk/nonlinear_spring_data/"
    # folder = root_folder+"with0springsTake3/"
    folder = "./cal_folder"
    # test(file=folder+'camera_calibration_spring_test.h264',
    main(cal_folder=folder,
        inner_mask = "mask_inner_0627.png", #None, #folder+'inner_mask0826.png',
        outer_mask = "mask_outer_0627.png" #None, #folder+'outer_mask0826.png',
        )
    # main(cal_folder='data/08_30_22_T13',
    #         inner_mask = 'inner_mask0830.png',
    #         outer_mask = 'outer_mask0830.png')

    # main(cal_folder='data/08_16_22_T1',
    #         inner_mask = 'inner_mask5.png',
    #         outer_mask = 'outer_mask3.png')

"""
Bibliography
walking_knee_78kg.mat
    [LeePanRouse2019IROS] Ung Hee Lee, Chen-Wen Pan, and Elliott J. Rouse "Empirical Characterization
        of a High-performance Exterior-rotor Type Brushless DC Motor and Drive" 2019 IEEE/RSJ
        International Conference on Intelligent Robots and Systems (IROS). pp. 8012 -- 8019, Macau,
        China, November 4-8, 2019.
"""
