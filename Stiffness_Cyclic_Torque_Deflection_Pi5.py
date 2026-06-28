import os
import sys
import time
from time import strftime

import numpy as np
from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.logging.logger import Logger
from opensourceleg.robots.osl import OpenSourceLeg
from opensourceleg.utilities import SoftRealtimeLoop
from picamera2 import Picamera2, Preview
from picamera2.encoders import H264Encoder

sys.path.append("./")

from hardware.encoder import AS5048A_Encoder  # noqa: E402
from hardware.encoder_correction import (  # noqa: E402
    Basic_LPF,
    Median_Filter,
    Spring_Model,
    TorqueSensorThread,
    backlash_centering_by_hand,
    calc_velocity_timescale,
    nonlinear_compensation,
)
from hardware.filtered_dephy import FilteredDephyActuator  # noqa: E402
from hardware.futek import Big100NmFutek  # noqa: E402


GR_ACTPACK = 1
GR_TRANS = 75 / 11
GR_BOSTONGEAR = 50

FREQUENCY = 500
DT = 1 / FREQUENCY
WAIT = 5
KNEE_PORT = "/dev/ttyACM0"

# User-defined cyclic stiffness test criteria.
NUM_CYCLES = 4
MAX_DEFLECTION_RAD = np.deg2rad(15)  # Maximum deflection in radians
DEFLECTION_TOL_RAD = np.deg2rad(0.5)
COMMAND_DEFLECTION_MARGIN_RAD = np.deg2rad(5)
MAX_TORQUE_NM = 150 * np.deg2rad(15)  # Maximum torque in Nm
TORQUE_TARGET_MARGIN_NM = 2.0
CYCLE_TIME = 24.0
HOLD_TIMEOUT_SEC = 5.0

# Hard safety limits. These are independent from the requested test criteria.
TORQUE_SAFETY_LIMIT_NM = 45.0
MOTOR_CURRENT_SAFETY_LIMIT_A = 8.5
WINDING_TEMPERATURE_SAFETY_LIMIT_C = 90.0
BATTERY_VOLTAGE_MAX_V = 43.0
BATTERY_VOLTAGE_MIN_V = 25.0


def port_is_accessible(port):
    if port is None:
        return False
    if not os.path.exists(port):
        return False

    try:
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError:
        return False

    os.close(fd)
    return True


def validate_test_config():
    if NUM_CYCLES <= 0:
        raise ValueError("NUM_CYCLES must be positive.")
    if MAX_DEFLECTION_RAD <= 0:
        raise ValueError("MAX_DEFLECTION_RAD must be positive.")
    if DEFLECTION_TOL_RAD < 0:
        raise ValueError("DEFLECTION_TOL_RAD must be non-negative.")
    if DEFLECTION_TOL_RAD >= MAX_DEFLECTION_RAD:
        raise ValueError("DEFLECTION_TOL_RAD must be below MAX_DEFLECTION_RAD.")
    if COMMAND_DEFLECTION_MARGIN_RAD < 0:
        raise ValueError("COMMAND_DEFLECTION_MARGIN_RAD must be non-negative.")
    if MAX_TORQUE_NM <= 0:
        raise ValueError("MAX_TORQUE_NM must be positive.")
    if TORQUE_TARGET_MARGIN_NM < 0:
        raise ValueError("TORQUE_TARGET_MARGIN_NM must be non-negative.")
    if CYCLE_TIME <= 0:
        raise ValueError("CYCLE_TIME must be positive.")
    if HOLD_TIMEOUT_SEC <= 0:
        raise ValueError("HOLD_TIMEOUT_SEC must be positive.")
    if MAX_TORQUE_NM >= TORQUE_SAFETY_LIMIT_NM:
        raise ValueError("MAX_TORQUE_NM must be below TORQUE_SAFETY_LIMIT_NM.")
    if MAX_TORQUE_NM + TORQUE_TARGET_MARGIN_NM >= TORQUE_SAFETY_LIMIT_NM:
        raise ValueError(
            "MAX_TORQUE_NM + TORQUE_TARGET_MARGIN_NM must be below "
            "TORQUE_SAFETY_LIMIT_NM."
        )


def check_safety(knee, tau_futek):
    if knee.winding_temperature > WINDING_TEMPERATURE_SAFETY_LIMIT_C:
        raise ValueError("Motor above thermal limit. Quitting!")

    battery_voltage_v = knee.battery_voltage / 1000
    if battery_voltage_v > BATTERY_VOLTAGE_MAX_V:
        print("Knee voltage {}".format(battery_voltage_v))
        raise ValueError("Battery voltage above {} V".format(BATTERY_VOLTAGE_MAX_V))
    if battery_voltage_v < BATTERY_VOLTAGE_MIN_V:
        print("Knee voltage {}".format(battery_voltage_v))
        raise ValueError("Battery voltage below {} V".format(BATTERY_VOLTAGE_MIN_V))

    if np.abs(tau_futek) > TORQUE_SAFETY_LIMIT_NM:
        raise ValueError("Torque too high: {} Nm".format(tau_futek))
    if np.abs(knee.motor_current) / 1000 > MOTOR_CURRENT_SAFETY_LIMIT_A:
        raise ValueError("Motor current too high: {} A".format(knee.motor_current * 1 / 1000))


if __name__ == "__main__":
    validate_test_config()

    file_name = "Stiffness_CyclicMax_" + strftime("%y%m%d_%H%M%S")
    video_file_name = file_name + ".h264"

    if not port_is_accessible(KNEE_PORT):
        raise RuntimeError(f"Knee actuator port {KNEE_PORT} is not accessible.")

    knee = FilteredDephyActuator(
        tag="knee",
        port=KNEE_PORT,
        gear_ratio=GR_BOSTONGEAR * GR_ACTPACK,
        frequency=FREQUENCY,
        dephy_log=False,
    )
    osl = OpenSourceLeg(
        tag="stiffness_test_pi4b",
        actuators={"knee": knee},
        sensors={},
    )
    clock = SoftRealtimeLoop(dt=DT, report=True)
    logger = Logger(log_path="./logs", file_name=file_name)

    torqueSensor = Big100NmFutek()
    torque_sensor_thread = TorqueSensorThread(torqueSensor, update_interval=DT)

    camera_available = False
    picam2 = None
    encoder = None
    camera_start_timestamp = 0.0
    camera_start_wall_time_ns = 0
    camera_start_monotonic_time_ns = 0
    camera_is_recording = False
    camera_preview_started = False
    camera_info = Picamera2.global_camera_info()
    if camera_info:
        picam2 = Picamera2()
        picam2.configure(
            picam2.create_video_configuration(
                raw={"size": (1640, 1232)},
                main={"size": (640, 480)},
            )
        )
        picam2.set_controls({"FrameRate": 30})
        encoder = H264Encoder()
        camera_available = True
    else:
        print("WARNING: No Picamera2 camera detected; continuing without video recording.")

    # Initialize joint encoder:
    # knee_enc = AS5048A_Encoder(
    #         name="output",
    #         basepath="/",
    #         bus="/dev/i2c-1",
    #         A1_adr_pin=False,
    #         A2_adr_pin=True,
    #         zero_position=0,
    #         )
    # knee_enc._start()
    # knee_enc._update()

    tau_futek = 0.0
    pos_command = 0.0
    measured_deflection = 0.0
    command_deflection_limit_rad = MAX_DEFLECTION_RAD + COMMAND_DEFLECTION_MARGIN_RAD
    torque_abort_limit_nm = MAX_TORQUE_NM + TORQUE_TARGET_MARGIN_NM
    target_direction = 1
    cycle_count = 0
    half_cycle_count = 0
    deflection_reached = False
    torque_reached = False
    torque_abort_reached = False
    switch_ready = False
    hold_elapsed = 0.0
    experiment_start_monotonic_time_ns = time.monotonic_ns()
    torque_thread_started = False

    logger.track_function(
        [
            lambda: time.time(),
            lambda: time.time_ns(),
            lambda: time.monotonic_ns(),
            lambda: (time.monotonic_ns() - experiment_start_monotonic_time_ns) * 1e-9,
            lambda: FREQUENCY,
            lambda: NUM_CYCLES,
            lambda: MAX_DEFLECTION_RAD,
            lambda: DEFLECTION_TOL_RAD,
            lambda: COMMAND_DEFLECTION_MARGIN_RAD,
            lambda: command_deflection_limit_rad,
            lambda: MAX_TORQUE_NM,
            lambda: TORQUE_TARGET_MARGIN_NM,
            lambda: torque_abort_limit_nm,
            lambda: CYCLE_TIME,
            lambda: HOLD_TIMEOUT_SEC,
            lambda: pos_command,
            lambda: measured_deflection,
            lambda: tau_futek,
            lambda: target_direction,
            lambda: cycle_count,
            lambda: half_cycle_count,
            lambda: int(deflection_reached),
            lambda: int(torque_reached),
            lambda: int(torque_abort_reached),
            lambda: int(switch_ready),
            lambda: hold_elapsed,
            lambda: int(camera_available),
            lambda: camera_start_timestamp,
            lambda: camera_start_wall_time_ns,
            lambda: camera_start_monotonic_time_ns,
        ],
        [
            "timestamp",
            "wall_time_ns",
            "monotonic_time_ns",
            "elapsed_time",
            "frequency",
            "num_cycles",
            "max_deflection_rad",
            "deflection_tol_rad",
            "command_deflection_margin_rad",
            "command_deflection_limit_rad",
            "max_torque_nm",
            "torque_target_margin_nm",
            "torque_abort_limit_nm",
            "cycle_time",
            "hold_timeout_sec",
            "pos_command",
            "measured_deflection",
            "tau_futek",
            "target_direction",
            "cycle_count",
            "half_cycle_count",
            "deflection_reached",
            "torque_reached",
            "torque_abort_reached",
            "switch_ready",
            "hold_elapsed",
            "camera_available",
            "camera_start_timestamp",
            "camera_start_wall_time_ns",
            "camera_start_monotonic_time_ns",
        ],
    )

    log_info = [
        "output_position",
        "output_velocity",
        "accelx",
        "motor_voltage",
        "motor_current",
        "battery_voltage",
        "battery_current",
        "packet_rejected",
        "packet_rejection_count",
        "packet_resync_count",
    ]
    logger.track_attributes(knee, log_info)
    logger.track_attributes(torqueSensor, ["torque"])

    try:
        if camera_available and picam2 is not None and encoder is not None:
            picam2.start_preview(Preview.DRM)
            camera_preview_started = True
            picam2.start_recording(encoder, video_file_name)
            camera_start_timestamp = time.time()
            camera_start_wall_time_ns = time.time_ns()
            camera_start_monotonic_time_ns = time.monotonic_ns()
            camera_is_recording = True

        # nonlinear_compensation(osl,knee_enc,Calibrate=False)
        with osl:
            torqueSensor.calibrate_loadcell()
            # offset,joint_ang_init = backlash_centering_by_hand(osl,knee_enc,Calculate=True)
            input("Hit enter to continue...")

            knee.set_control_mode(CONTROL_MODES.POSITION)
            knee.set_position_gains(
                kp=450,
                ki=500,
                kd=0,
                ff=-50,
            )

            osl.update()
            init_pos = knee.output_position

            # Initialize filters
            f_c = 50
            window = 3

            # ang_err_med_filter = Median_Filter(0,window)
            # ang_err_lp_filter = Basic_LPF(0,osl._frequency,f_c)

            # Output SEA Config
            spring_type = "Output SEA"

            # # Intermediate SEA Config
            # spring_type = "Intermediate SEA"

            # Load spring parameters
            spring = Spring_Model(spring_type)

            # Velocity compensation
            velocity_time_scalar = calc_velocity_timescale(FREQUENCY)

            # Set up test
            # ank_output_0 = osl.ankle.output_position

            deflection_rate_rad_per_sec = 4 * MAX_DEFLECTION_RAD / CYCLE_TIME

            print("Test in Progress:")
            torque_sensor_thread.start()
            torque_thread_started = True

            for t in clock:
                tau_futek = torque_sensor_thread.get_latest_torque()
                osl.update()
                check_safety(knee, tau_futek)
                logger.update()
                if t > WAIT:
                    break

            hold_started_at = None
            last_t = None
            test_complete = False
            for t in clock:
                if last_t is None:
                    loop_dt = DT
                else:
                    loop_dt = max(t - last_t, 0.0)
                last_t = t

                tau_futek = torque_sensor_thread.get_latest_torque()
                # knee_enc._update()
                osl.update()

                measured_deflection = knee.output_position - init_pos
                check_safety(knee, tau_futek)

                # joint_ang_pre = knee_enc.abs_comp_ang
                # output_ang_pre = osl.knee.output_position + offset
                # ang_err_pre = output_ang_pre - joint_ang_pre

                # output_ang_adjusted = output_ang_pre + velocity_time_scalar*osl.knee.output_velocity
                # ang_err = output_ang_adjusted - joint_ang_pre
                # ang_err_filt = ang_err_lp_filter.update(ang_err_med_filter.update(ang_err))
                # deflection = spring.backlash_comp_smooth(ang_err_filt)
                # tau_meas = deflection*spring.K*GR_ACTPACK*GR_TRANS/osl.knee.gear_ratio

                if target_direction > 0:
                    pos_command = min(
                        pos_command + deflection_rate_rad_per_sec * loop_dt,
                        command_deflection_limit_rad,
                    )
                else:
                    pos_command = max(
                        pos_command - deflection_rate_rad_per_sec * loop_dt,
                        -command_deflection_limit_rad,
                    )

                commanded_limit_reached = (
                    target_direction * pos_command >= command_deflection_limit_rad
                )
                deflection_reached = (
                    target_direction * measured_deflection
                    >= MAX_DEFLECTION_RAD - DEFLECTION_TOL_RAD
                )
                # FUTEK polarity can vary by setup, so the test criterion uses magnitude.
                torque_reached = np.abs(tau_futek) >= MAX_TORQUE_NM
                torque_abort_reached = np.abs(tau_futek) >= torque_abort_limit_nm
                switch_ready = deflection_reached and torque_reached
                print(
                    "motor_current_mA={:.0f}, motor_current_A={:.3f}, "
                    "tau_futek_Nm={:.3f}, cmd_deg={:.2f}, deflection_deg={:.2f}, "
                    "deflection_reached={}, torque_reached={}, torque_abort={}, "
                    "hold_elapsed_s={:.2f}".format(
                        knee.motor_current,
                        knee.motor_current / 1000,
                        tau_futek,
                        np.rad2deg(pos_command),
                        np.rad2deg(measured_deflection),
                        deflection_reached,
                        torque_reached,
                        torque_abort_reached,
                        hold_elapsed,
                    )
                )

                if torque_abort_reached:
                    raise RuntimeError(
                        "Torque exceeded target margin before both switch criteria were met. "
                        "deflection_reached={}, torque_reached={}, "
                        "measured_deflection={:.4f} rad, tau_futek={:.2f} Nm, "
                        "torque_abort_limit={:.2f} Nm".format(
                            deflection_reached,
                            torque_reached,
                            measured_deflection,
                            tau_futek,
                            torque_abort_limit_nm,
                        )
                    )
                elif switch_ready:
                    hold_started_at = None
                    hold_elapsed = 0.0
                    half_cycle_count += 1
                    if target_direction < 0:
                        cycle_count += 1
                        print("Completed cycle {} of {}".format(cycle_count, NUM_CYCLES))
                        if cycle_count >= NUM_CYCLES:
                            test_complete = True

                    if not test_complete:
                        target_direction *= -1
                        print(
                            "Switching direction after reaching {:.4f} rad and {:.2f} Nm".format(
                                measured_deflection,
                                tau_futek,
                            )
                        )
                elif commanded_limit_reached:
                    if hold_started_at is None:
                        hold_started_at = t
                    hold_elapsed = t - hold_started_at
                    if hold_elapsed > HOLD_TIMEOUT_SEC:
                        raise TimeoutError(
                            "Timed out at deflection command allowance limit. "
                            "deflection_reached={}, torque_reached={}, "
                            "measured_deflection={:.4f} rad, tau_futek={:.2f} Nm, "
                            "command_deflection_limit={:.4f} rad".format(
                                deflection_reached,
                                torque_reached,
                                measured_deflection,
                                tau_futek,
                                command_deflection_limit_rad,
                            )
                        )
                else:
                    hold_started_at = None
                    hold_elapsed = 0.0

                knee.set_output_position(init_pos + pos_command)
                logger.update()
                if test_complete:
                    break

            print("Returning to zero deflection:")
            last_t = None
            for t in clock:
                if last_t is None:
                    loop_dt = DT
                else:
                    loop_dt = max(t - last_t, 0.0)
                last_t = t

                tau_futek = torque_sensor_thread.get_latest_torque()
                osl.update()
                measured_deflection = knee.output_position - init_pos
                check_safety(knee, tau_futek)

                if pos_command > 0:
                    pos_command = max(
                        pos_command - deflection_rate_rad_per_sec * loop_dt,
                        0.0,
                    )
                else:
                    pos_command = min(
                        pos_command + deflection_rate_rad_per_sec * loop_dt,
                        0.0,
                    )

                deflection_reached = False
                torque_reached = False
                torque_abort_reached = False
                switch_ready = False
                hold_elapsed = 0.0

                knee.set_output_position(init_pos + pos_command)
                logger.update()

                if pos_command == 0.0:
                    break

            for t in clock:
                tau_futek = torque_sensor_thread.get_latest_torque()
                osl.update()
                measured_deflection = knee.output_position - init_pos
                check_safety(knee, tau_futek)
                logger.update()
                if t > WAIT:
                    break

            time.sleep(3)
            print("Test complete :)")
    finally:
        if torque_thread_started:
            torque_sensor_thread.stop()
            torque_sensor_thread.join()

        if camera_is_recording and picam2 is not None:
            picam2.stop_recording()
        if camera_preview_started and picam2 is not None:
            picam2.stop_preview()
        logger.close()
