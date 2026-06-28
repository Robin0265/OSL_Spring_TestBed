import numpy as np
from scipy.optimize import minimize


def _as_clean_arrays(theta_deg, tau_nm):
    theta_deg = np.asarray(theta_deg, dtype=float)
    tau_nm = np.asarray(tau_nm, dtype=float)
    valid = np.isfinite(theta_deg) & np.isfinite(tau_nm)
    return theta_deg[valid], tau_nm[valid]


def _moving_average_edge(values, window_size):
    values = np.asarray(values, dtype=float)
    if window_size is None or window_size <= 1 or values.size < 2:
        return values.copy()

    window_size = int(window_size)
    if window_size % 2 == 0:
        window_size += 1
    window_size = min(window_size, values.size)
    if window_size % 2 == 0:
        window_size -= 1
    if window_size <= 1:
        return values.copy()

    pad = window_size // 2
    padded = np.pad(values, pad, mode="edge")
    kernel = np.ones(window_size, dtype=float) / window_size
    return np.convolve(padded, kernel, mode="valid")


def _prepare_fit_data(theta_deg, tau_nm, smooth_window):
    theta_deg, tau_nm = _as_clean_arrays(theta_deg, tau_nm)
    theta_rad = np.deg2rad(theta_deg)
    theta_rad_smooth = _moving_average_edge(theta_rad, smooth_window)
    tau_nm_smooth = _moving_average_edge(tau_nm, smooth_window)
    return theta_rad, tau_nm, theta_rad_smooth, tau_nm_smooth


def _even_subset(*arrays, max_samples=4000):
    n_samples = len(arrays[0])
    if max_samples is None or n_samples <= max_samples:
        return arrays
    indices = np.unique(np.linspace(0, n_samples - 1, int(max_samples)).astype(int))
    return tuple(np.asarray(array)[indices] for array in arrays)


def _rms(values):
    values = np.asarray(values, dtype=float)
    if values.size == 0:
        return np.nan
    return float(np.sqrt(np.mean(values ** 2)))


def _l1_optimal_offset(measured, model):
    return float(np.median(np.asarray(measured, dtype=float) - np.asarray(model, dtype=float)))


def _clip_initial(initial, lower, upper):
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)
    initial = np.asarray(initial, dtype=float)
    return np.minimum(np.maximum(initial, lower), upper)


def _bounded_multistart_minimize(objective, initials, lower, upper):
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)
    starts = [_clip_initial(initial, lower, upper) for initial in initials]

    best_result = None
    best_value = np.inf
    bounds = list(zip(lower, upper))
    for initial in starts:
        result = minimize(
            objective,
            initial,
            bounds=bounds,
            method="Powell",
            options={
                "disp": False,
                "maxiter": 700,
                "xtol": 1e-7,
                "ftol": 1e-7,
            },
        )
        value = objective(result.x)
        if np.isfinite(value) and value < best_value:
            best_value = value
            best_result = result

    if best_result is None:
        raise RuntimeError("Spring model optimization failed from all initial guesses.")
    return best_result


def _softplus(z):
    z = np.asarray(z, dtype=float)
    return np.maximum(z, 0.0) + np.log1p(np.exp(-np.abs(z)))


def calc_simple_backlash_torque(theta_rad, stiffness, half_backlash):
    theta_rad = np.asarray(theta_rad, dtype=float)
    half_backlash = max(float(half_backlash), 0.0)
    deflection = np.where(theta_rad > 0.0, theta_rad - half_backlash, theta_rad + half_backlash)
    deflection = np.where(np.abs(theta_rad) < half_backlash, 0.0, deflection)
    return float(stiffness) * deflection


def calc_logit_backlash_torque(theta_rad, stiffness, half_backlash, logit_buffer):
    theta_rad = np.asarray(theta_rad, dtype=float)
    half_backlash = max(float(half_backlash), 0.0)
    logit_buffer = max(float(logit_buffer), np.finfo(float).eps)
    deflection = logit_buffer * (
        _softplus((theta_rad - half_backlash) / logit_buffer)
        - _softplus((-theta_rad - half_backlash) / logit_buffer)
    )
    return float(stiffness) * deflection


def fit_linear_spring_model(theta_deg, tau_nm, smooth_window=25):
    theta_raw, tau_raw, theta_fit, tau_fit = _prepare_fit_data(
        theta_deg,
        tau_nm,
        smooth_window,
    )
    if theta_raw.size < 2:
        raise ValueError("At least two finite theta/torque samples are required.")

    stiffness, offset = np.polyfit(theta_raw, tau_raw, 1)
    tau_hat = stiffness * theta_fit + offset
    residual = tau_fit - tau_hat
    return {
        "name": "linear",
        "theta_rad": theta_fit,
        "theta_deg": np.rad2deg(theta_fit),
        "tau_nm": tau_fit,
        "tau_hat_nm": tau_hat,
        "residual_nm": residual,
        "rms_tau_nm": _rms(residual),
        "params": {
            "stiffness_nm_per_rad": float(stiffness),
            "torque_offset_nm": float(offset),
        },
        "success": True,
        "message": "",
    }


def fit_simple_backlash_model(theta_deg, tau_nm, smooth_window=25, linear_fit=None,
        max_fit_samples=4000):
    theta_raw, tau_raw, theta_fit, tau_fit = _prepare_fit_data(
        theta_deg,
        tau_nm,
        smooth_window,
    )
    if theta_fit.size < 2:
        raise ValueError("At least two finite theta/torque samples are required.")

    if linear_fit is None:
        linear_fit = fit_linear_spring_model(theta_deg, tau_nm, smooth_window=smooth_window)
    linear_stiffness = linear_fit["params"]["stiffness_nm_per_rad"]
    theta_obj, tau_obj = _even_subset(theta_fit, tau_fit, max_samples=max_fit_samples)

    eps = np.finfo(float).eps
    max_deflection = float(np.max(np.abs(theta_fit)))
    lower = np.array([0.0, 0.0, np.min(theta_fit)], dtype=float)
    upper = np.array([
        10.0 * max(abs(linear_stiffness), eps),
        max(max_deflection, eps),
        np.max(theta_fit),
    ], dtype=float)
    x0_candidates = np.array([
        0.0,
        np.deg2rad(-1.5),
        np.deg2rad(-1.0),
        np.deg2rad(-0.5),
        np.deg2rad(0.5),
        np.deg2rad(1.0),
        np.deg2rad(1.5),
    ])
    h_candidates = np.array([
        np.deg2rad(0.25),
        np.deg2rad(0.5),
        np.deg2rad(1.0),
    ])
    k_candidates = np.array([linear_stiffness])
    initials = [
        np.array([k, h, x0], dtype=float)
        for k in k_candidates
        for h in h_candidates
        for x0 in x0_candidates
    ]

    def objective(params):
        stiffness, half_backlash, theta_offset = params
        centered_tau_hat = calc_simple_backlash_torque(
            theta_obj - theta_offset,
            stiffness,
            half_backlash,
        )
        torque_offset = _l1_optimal_offset(tau_obj, centered_tau_hat)
        tau_hat = torque_offset + centered_tau_hat
        return float(np.sum(np.abs(tau_obj - tau_hat)))

    result = _bounded_multistart_minimize(
        objective,
        initials,
        lower,
        upper,
    )

    stiffness, half_backlash, theta_offset = result.x
    centered_theta = theta_fit - theta_offset
    centered_tau_hat = calc_simple_backlash_torque(centered_theta, stiffness, half_backlash)
    torque_offset = _l1_optimal_offset(tau_fit, centered_tau_hat)
    tau_hat = torque_offset + centered_tau_hat
    residual = tau_fit - tau_hat
    return {
        "name": "simple",
        "theta_rad": theta_fit,
        "theta_deg": np.rad2deg(theta_fit),
        "centered_theta_rad": centered_theta,
        "centered_theta_deg": np.rad2deg(centered_theta),
        "tau_nm": tau_fit,
        "centered_tau_nm": tau_fit - torque_offset,
        "tau_hat_nm": tau_hat,
        "centered_tau_hat_nm": centered_tau_hat,
        "residual_nm": residual,
        "rms_tau_nm": _rms(residual),
        "params": {
            "stiffness_nm_per_rad": float(stiffness),
            "half_backlash_rad": float(half_backlash),
            "backlash_rad": float(2.0 * half_backlash),
            "theta_offset_rad": float(theta_offset),
            "torque_offset_nm": float(torque_offset),
        },
        "success": bool(result.success),
        "message": result.message,
    }


def fit_logit_backlash_model(theta_deg, tau_nm, smooth_window=25, simple_fit=None,
        max_fit_samples=4000):
    theta_raw, tau_raw, theta_fit, tau_fit = _prepare_fit_data(
        theta_deg,
        tau_nm,
        smooth_window,
    )
    if theta_fit.size < 2:
        raise ValueError("At least two finite theta/torque samples are required.")

    if simple_fit is None:
        simple_fit = fit_simple_backlash_model(
            theta_deg,
            tau_nm,
            smooth_window=smooth_window,
            max_fit_samples=max_fit_samples,
        )
    theta_obj, tau_obj = _even_subset(theta_fit, tau_fit, max_samples=max_fit_samples)

    eps = np.finfo(float).eps
    simple_params = simple_fit["params"]
    simple_stiffness = simple_params["stiffness_nm_per_rad"]
    simple_half_backlash = simple_params["half_backlash_rad"]
    max_deflection = float(np.max(np.abs(theta_fit)))

    lower = np.array([0.0, 0.0, eps, np.min(theta_fit)], dtype=float)
    upper = np.array([
        10.0 * max(abs(simple_stiffness), eps),
        max(max_deflection, eps),
        max(max_deflection, eps),
        np.max(theta_fit),
    ], dtype=float)
    base_initial = np.array([
            simple_stiffness,
            max(abs(simple_half_backlash), eps),
            max(abs(simple_half_backlash) / 2.0, eps),
            simple_params["theta_offset_rad"],
    ], dtype=float)
    buffer_candidates = np.array([
        max(abs(simple_half_backlash) / 4.0, eps),
        max(abs(simple_half_backlash) / 2.0, eps),
        max(abs(simple_half_backlash), eps),
        np.deg2rad(0.1),
        np.deg2rad(0.25),
    ])
    x0_candidates = np.array([
        simple_params["theta_offset_rad"],
        simple_params["theta_offset_rad"] - np.deg2rad(0.5),
        simple_params["theta_offset_rad"] + np.deg2rad(0.5),
        0.0,
    ])
    initials = []
    for logit_buffer in buffer_candidates:
        for theta_offset in x0_candidates:
            initial = base_initial.copy()
            initial[2] = logit_buffer
            initial[3] = theta_offset
            initials.append(initial)

    def objective(params):
        stiffness, half_backlash, logit_buffer, theta_offset = params
        centered_tau_hat = calc_logit_backlash_torque(
            theta_obj - theta_offset,
            stiffness,
            half_backlash,
            logit_buffer,
        )
        torque_offset = _l1_optimal_offset(tau_obj, centered_tau_hat)
        tau_hat = torque_offset + centered_tau_hat
        return float(np.sum(np.abs(tau_obj - tau_hat)))

    result = _bounded_multistart_minimize(
        objective,
        initials,
        lower,
        upper,
    )

    stiffness, half_backlash, logit_buffer, theta_offset = result.x
    centered_theta = theta_fit - theta_offset
    centered_tau_hat = calc_logit_backlash_torque(
        centered_theta,
        stiffness,
        half_backlash,
        logit_buffer,
    )
    torque_offset = _l1_optimal_offset(tau_fit, centered_tau_hat)
    tau_hat = torque_offset + centered_tau_hat
    residual = tau_fit - tau_hat
    return {
        "name": "logit",
        "theta_rad": theta_fit,
        "theta_deg": np.rad2deg(theta_fit),
        "centered_theta_rad": centered_theta,
        "centered_theta_deg": np.rad2deg(centered_theta),
        "tau_nm": tau_fit,
        "centered_tau_nm": tau_fit - torque_offset,
        "tau_hat_nm": tau_hat,
        "centered_tau_hat_nm": centered_tau_hat,
        "residual_nm": residual,
        "rms_tau_nm": _rms(residual),
        "params": {
            "stiffness_nm_per_rad": float(stiffness),
            "half_backlash_rad": float(half_backlash),
            "backlash_rad": float(2.0 * half_backlash),
            "logit_buffer_rad": float(logit_buffer),
            "logit_param": float(1.0 / max(logit_buffer, eps)),
            "theta_offset_rad": float(theta_offset),
            "torque_offset_nm": float(torque_offset),
        },
        "success": bool(result.success),
        "message": result.message,
    }


def fit_all_spring_models(theta_deg, tau_nm, smooth_window=25, max_fit_samples=4000):
    linear_fit = fit_linear_spring_model(theta_deg, tau_nm, smooth_window=smooth_window)
    simple_fit = fit_simple_backlash_model(
        theta_deg,
        tau_nm,
        smooth_window=smooth_window,
        linear_fit=linear_fit,
        max_fit_samples=max_fit_samples,
    )
    logit_fit = fit_logit_backlash_model(
        theta_deg,
        tau_nm,
        smooth_window=smooth_window,
        simple_fit=simple_fit,
        max_fit_samples=max_fit_samples,
    )
    return {
        "linear": linear_fit,
        "simple": simple_fit,
        "logit": logit_fit,
    }
