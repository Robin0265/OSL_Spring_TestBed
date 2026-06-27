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


if __name__ == "__main__":
    file_name = "Stiffness_Measure_" + strftime("%y%m%d_%H%M%S")
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
    experiment_start_monotonic_time_ns = time.monotonic_ns()
    torque_thread_started = False

    logger.track_function(
        [
            lambda: time.time(),
            lambda: time.time_ns(),
            lambda: time.monotonic_ns(),
            lambda: (time.monotonic_ns() - experiment_start_monotonic_time_ns) * 1e-9,
            lambda: FREQUENCY,
            lambda: pos_command,
            lambda: tau_futek,
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
            "pos_command",
            "tau_futek",
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
                kp=300,
                ki=150,
                kd=100,
                ff=100,
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

            # i_des = np.linspace(1000, 3000, 4)
            pos_des = np.array(
                [
                    4 / 180 * np.pi,
                    8 / 180 * np.pi,
                    12 / 180 * np.pi,
                    15 / 180 * np.pi,
                ]
            )
            t_test = 7

            print("Test in Progress:")
            torque_sensor_thread.start()
            torque_thread_started = True

            for t in clock:
                tau_futek = torque_sensor_thread.get_latest_torque()
                osl.update()
                logger.update()
                if t > WAIT:
                    break

            # for i in i_des:
            for p in pos_des:
                for t in clock:
                    if t < t_test:
                        # i_command = i/t_test*t
                        pos_command = p / t_test * t
                    elif t < 3 * t_test:
                        # i_command = -i/t_test*t + 2*i
                        pos_command = -p / t_test * t + 2 * p
                    elif t < 4 * t_test:
                        # i_command = i/t_test*t - 4*i
                        pos_command = p / t_test * t - 4 * p
                    else:
                        break

                    tau_futek = torque_sensor_thread.get_latest_torque()
                    # knee_enc._update()
                    osl.update()

                    # joint_ang_pre = knee_enc.abs_comp_ang
                    # output_ang_pre = osl.knee.output_position + offset
                    # ang_err_pre = output_ang_pre - joint_ang_pre

                    # output_ang_adjusted = output_ang_pre + velocity_time_scalar*osl.knee.output_velocity
                    # ang_err = output_ang_adjusted - joint_ang_pre
                    # ang_err_filt = ang_err_lp_filter.update(ang_err_med_filter.update(ang_err))
                    # deflection = spring.backlash_comp_smooth(ang_err_filt)
                    # tau_meas = deflection*spring.K*GR_ACTPACK*GR_TRANS/osl.knee.gear_ratio

                    print(knee.motor_current)
                    # SAFETY CHECKS
                    if knee.winding_temperature > 90:
                        raise ValueError("Motor above thermal limit. Quitting!")
                    # Check battery voltages
                    if knee.battery_voltage / 1000 > 43:
                        print("Knee voltage {}".format(1 / 1000 * knee.battery_voltage))
                        raise ValueError("Battery voltage above 43 V")
                    if knee.battery_voltage / 1000 < 25:
                        print("Knee voltage {}".format(1 / 1000 * knee.battery_voltage))
                        raise ValueError("Battery voltage below 25 V")
                    if np.abs(tau_futek) > 45:
                        print("Torque too high: {} Nm".format(tau_futek))
                        break
                    if np.abs(knee.motor_current)/1000 > 8.5:
                        print("Motor current too high: {} A".format(knee.motor_current * 1 / 1000))
                        break

                    knee.set_output_position(init_pos + pos_command)
                    logger.update()

            for t in clock:
                tau_futek = torque_sensor_thread.get_latest_torque()
                osl.update()
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
