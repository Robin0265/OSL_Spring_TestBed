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

def _prepare_inverse_lookup(camera_values, encoder_values, min_dx=1e-6):
    camera_values = np.asarray(camera_values, dtype=float)
    encoder_values = np.asarray(encoder_values, dtype=float)

    valid = np.isfinite(camera_values) & np.isfinite(encoder_values)
    camera_values = camera_values[valid]
    encoder_values = encoder_values[valid]
    if camera_values.size < 2:
        raise ValueError("Not enough valid samples to build inverse lookup.")

    order = np.argsort(camera_values)
    camera_sorted = camera_values[order]
    encoder_sorted = encoder_values[order]

    uniq_camera = [camera_sorted[0]]
    uniq_encoder = [encoder_sorted[0]]
    bucket_camera = [camera_sorted[0]]
    bucket_encoder = [encoder_sorted[0]]

    for camera_value, encoder_value in zip(camera_sorted[1:], encoder_sorted[1:]):
        if abs(camera_value - uniq_camera[-1]) <= min_dx:
            bucket_camera.append(camera_value)
            bucket_encoder.append(encoder_value)
            uniq_camera[-1] = float(np.mean(bucket_camera))
            uniq_encoder[-1] = float(np.mean(bucket_encoder))
            continue

        bucket_camera = [camera_value]
        bucket_encoder = [encoder_value]
        uniq_camera.append(camera_value)
        uniq_encoder.append(encoder_value)

    uniq_camera = np.asarray(uniq_camera, dtype=float)
    uniq_encoder = np.asarray(uniq_encoder, dtype=float)

    strictly_increasing = np.diff(uniq_camera) > 0
    if not np.all(strictly_increasing):
        keep = np.hstack(([True], strictly_increasing))
        uniq_camera = uniq_camera[keep]
        uniq_encoder = uniq_encoder[keep]

    if uniq_camera.size < 2:
        raise ValueError("Inverse lookup collapsed after monotonicity cleanup.")

    return np.column_stack((uniq_camera, uniq_encoder))

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
        test(file = cal_folder + '/Calibration260624_232636.h264',
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

        blue_cam_angs, red_cam_angs, cam_time = test(file = cal_folder + '/Calibration260624_232636.h264',
                                                    inner_mask_loc = inner_mask,
                                                    outer_mask_loc = outer_mask,
                                                    pre_mask_save_loc = cal_folder + '/camera_calibration_pre_mask.png',
                                                    red_cal_save_loc = None,
                                                    blue_cal_save_loc = None)

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
    stp = SEATestbedPlotter(cal_folder + '/Calibration260624_232636.csv',cal_folder + '/Calibration260624_232636.csv')
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

    blue_cam_rng_res = np.interp(enc_time_rng, cam_time_rng, blue_cam_rng)
    red_cam_rng_res = np.interp(enc_time_rng, cam_time_rng, red_cam_rng)

    inv_blue = _prepare_inverse_lookup(blue_cam_rng_res, blue_enc_rng)
    inv_red = _prepare_inverse_lookup(red_cam_rng_res, red_enc_rng)
    print(
        "inverse lookup sizes: blue %d -> %d, red %d -> %d"
        % (len(blue_cam_rng_res), len(inv_blue), len(red_cam_rng_res), len(inv_red))
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
        np.savetxt(f, inv_blue, fmt='%.7f', delimiter=", ")
    with open(cal_folder + '/inv_red.csv', 'w') as f:
        np.savetxt(f, inv_red, fmt='%.7f', delimiter=", ")

    plt.figure(4)
    plt.plot(blue_cam_rng_res,blue_enc_rng-blue_cam_rng_res,'b')
    plt.plot(red_cam_rng_res,red_enc_rng-red_cam_rng_res,'r')
    
    blue_cam_ang_cal = np.interp(new_blue_cam_angs,inv_blue[:,0],inv_blue[:,1])
    red_cam_ang_cal = np.interp(new_red_cam_angs,inv_red[:,0],inv_red[:,1])
    
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
    plt.plot(plot_time_rel, 
             red_cam_ang_cal_plot - red_enc_ang_res, 'r')
    plt.plot(plot_time_rel,
             blue_cam_ang_cal_plot - blue_enc_ang_res, 'b')
    plt.show()


if __name__ == '__main__':
    # root_folder = "/home/gray/wk/nonlinear_spring_data/"
    # folder = root_folder+"with0springsTake3/"
    folder = "./cal_folder"
    # test(file=folder+'camera_calibration_spring_test.h264',
    main(cal_folder=folder,
        inner_mask = "mask_inner_0618.png", #None, #folder+'inner_mask0826.png',
        outer_mask = "mask_outer_0618.png" #None, #folder+'outer_mask0826.png',
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
