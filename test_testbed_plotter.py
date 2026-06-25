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
from test_calibration_plotter import fit_camera_time_alignment
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

        blue_cam_angs, red_cam_angs, cam_time = test(file = test_folder + '/Stiffness_Measure_260625_005652.h264',
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
    stp = SEATestbedPlotter(test_folder + '/Stiffness_Measure_260625_005652.csv',
                            test_folder + '/Stiffness_Measure_260625_005652.csv',
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


    # Second calibration, interpolate cam_angs to enc_angs
    inv_blue = np.loadtxt(cal_folder + '/inv_blue.csv', delimiter=',')
    inv_red = np.loadtxt(cal_folder + '/inv_red.csv', delimiter=',')
    inv_blue = inv_blue[np.argsort(inv_blue[:,0])]
    inv_red = inv_red[np.argsort(inv_red[:,0])]

    blue_cam_ang_cal = np.interp(new_blue_cam_angs, inv_blue[:,0], inv_blue[:,1])
    red_cam_ang_cal = np.interp(new_red_cam_angs, inv_red[:,0], inv_red[:,1])

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
    
    plt.figure(3)
    plt.plot(plot_time_rel,blue_cam_ang_cal_res*180/np.pi,'b')
    plt.plot(plot_time_rel,red_cam_ang_cal_res*180/np.pi,'r')
    plt.plot(plot_time_rel,blue_enc_plot*180/np.pi,'c')
    plt.plot(plot_time_rel,red_enc_plot*180/np.pi,'m')
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
    plt.plot(aligned_cam_time_plot, red_cam_ang_cal*180/np.pi, 'g')
    plt.plot(aligned_cam_time_plot, blue_cam_ang_cal*180/np.pi, 'k')
    plt.plot(new_enc_time_plot, -new_torque)
    plt.legend(['Blue Cam','Red Cam','Blue Enc','Red Enc',
                'Blue Cam Cal','Red Cam Cal',
                'Torque Sensor'])
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (deg)')
    plt.title('Angle Calibration')

    # defl = -(blue_cam_ang_cal - red_cam_ang_cal)*180/np.pi
    # torque_res = np.interp(cam_time, trq_time, torque)
    
    enc_defl = -(blue_enc_plot - red_enc_plot)*180/np.pi

    # defl_torque = np.vstack((defl,torque_res)).T
    # defl_torque = np.vstack((enc_defl,torque)).T

    # plt.figure(2)
    # plt.plot(cam_time,defl)
    # plt.plot(enc_time,enc_defl)

    plt.figure(2)
    plt.plot(enc_defl, torque_plot)
    plt.plot(-(blue_cam_ang_cal_res - red_cam_ang_cal_res)*180/np.pi, torque_plot)
    # plt.plot(defl,torque_res)
    plt.legend(['Motor Encoder Measurement','Optical Measurement'])
    plt.xlabel('Deflection (deg)')
    plt.ylabel('Torque (Nm)')

    with open(test_folder + defl_trq_file, 'w') as f:
        np.savetxt(f, np.vstack(
            (-(blue_cam_ang_cal_res - red_cam_ang_cal_res)*180/np.pi, torque_plot)
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
            inner_mask = 'mask_inner_0618.png',
            outer_mask = 'mask_outer_0618.png',
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
