# Series Spring Calibration Process
# Compatible with opensourceleg 3.5.x
import os
import sys
import time
from time import strftime

import numpy as np
from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.logging.logger import Logger
from opensourceleg.robots.osl import OpenSourceLeg
from opensourceleg.utilities import SoftRealtimeLoop
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder

# import RPi.GPIO as GPIO

# Redirect to current folder
sys.path.append("./")

from hardware.encoder_correction import TorqueSensorThread  # noqa: E402
from hardware.filtered_dephy import FilteredDephyActuator  # noqa: E402
from hardware.futek import Big100NmFutek  # noqa: E402

# Mechanical Constants
GR_ACTPACK = 1  # Actuator Gearbox
GR_TRANS = 75 / 11
GR_BOSTONGEAR = 50  # Gear Ratio from Boston Gear

T = 10  # Period Time
TIME_TO_STEP = 1.0
FREQUENCY = 200
DT = 1 / FREQUENCY
WAIT = 3

PIN_FALLING = 6
PIN_END = 26

VOLT = 2000  # in mV
DIRECTION = 1
FLAG = 1
cnt = 0

# GPIO.setmode(GPIO.BCM)
# GPIO.setup(PIN_FALLING, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
# GPIO.setup(PIN_END, GPIO.OUT)

file_name = "Calibration" + strftime("%y%m%d_%H%M%S")
video_file_name = file_name + ".h264"

ACTUATOR_PORTS = {
    "knee": "/dev/ttyACM0",
    "ankle": "/dev/ttyACM1",
}


def port_is_accessible(port):
    if not os.path.exists(port):
        return False

    try:
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError:
        return False

    os.close(fd)
    return True


actuators = {}
for tag, port in ACTUATOR_PORTS.items():
    if not port_is_accessible(port):
        print(f"WARNING: {tag} actuator port {port} is not accessible; skipping {tag}.")
        continue

    actuators[tag] = FilteredDephyActuator(
        tag=tag,
        port=port,
        gear_ratio=GR_ACTPACK * GR_BOSTONGEAR,
        frequency=FREQUENCY,
        dephy_log=False,
    )

if not actuators:
    raise RuntimeError("No actuators found. Check /dev/ttyACM0 and /dev/ttyACM1.")

print(f"Active actuators: {', '.join(actuators)}")

osl = OpenSourceLeg(
    tag="series_spring_calibration",
    actuators=actuators,
    sensors={},
)

has_knee = "knee" in osl.actuators
has_ankle = "ankle" in osl.actuators
knee = osl.actuators.get("knee")
ankle = osl.actuators.get("ankle")

clock = SoftRealtimeLoop(dt=DT, report=True)
logger = Logger(log_path="./logs", file_name=file_name)
camera_available = False
camera_info = Picamera2.global_camera_info()
picam2 = None
encoder = None
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

if not camera_available:
    print("WARNING: No Picamera2 camera detected; continuing without video recording.")

# torqueSensor = Big100NmFutek()
# torque_sensor_thread = TorqueSensorThread(torqueSensor, update_interval=DT)

position_command = 0.0
camera_start_timestamp = 0.0
camera_start_wall_time_ns = 0
camera_start_monotonic_time_ns = 0
camera_is_recording = False
experiment_start_monotonic_time_ns = time.monotonic_ns()
# tau_futek = 0.0


def raw_data(actuator, field, default=np.nan):
    if actuator is None:
        return default
    data = getattr(actuator, "_data", None)
    if data is None:
        return default
    return data.get(field, default)


def track_raw_actuator_data(tag, actuator):
    logger.track_function(
        [
            lambda: raw_data(actuator, "state_time"),
            lambda: raw_data(actuator, "sys_time"),
            lambda: raw_data(actuator, "mot_ang"),
            lambda: raw_data(actuator, "mot_vel"),
            lambda: raw_data(actuator, "mot_acc"),
            lambda: raw_data(actuator, "mot_cur"),
            lambda: raw_data(actuator, "mot_volt"),
            lambda: raw_data(actuator, "batt_volt"),
            lambda: raw_data(actuator, "batt_curr"),
            lambda: raw_data(actuator, "temperature"),
            lambda: raw_data(actuator, "status_mn"),
            lambda: raw_data(actuator, "status_ex"),
            lambda: raw_data(actuator, "status_re"),
        ],
        [
            f"{tag}_state_time",
            f"{tag}_sys_time",
            f"{tag}_mot_ang",
            f"{tag}_mot_vel",
            f"{tag}_mot_acc",
            f"{tag}_mot_cur",
            f"{tag}_mot_volt",
            f"{tag}_batt_volt",
            f"{tag}_batt_curr",
            f"{tag}_temperature",
            f"{tag}_status_mn",
            f"{tag}_status_ex",
            f"{tag}_status_re",
        ],
    )


logger.track_function(
    [
        lambda: time.time(),
        lambda: time.time_ns(),
        lambda: time.monotonic_ns(),
        lambda: (time.monotonic_ns() - experiment_start_monotonic_time_ns) * 1e-9,
        lambda: FREQUENCY,
        lambda: position_command,
        lambda: int(camera_available),
        lambda: camera_start_timestamp,
        lambda: camera_start_wall_time_ns,
        lambda: camera_start_monotonic_time_ns,
        # lambda: tau_futek,
    ],
    [
        "timestamp",
        "wall_time_ns",
        "monotonic_time_ns",
        "elapsed_time",
        "frequency",
        "position_command",
        "camera_available",
        "camera_start_timestamp",
        "camera_start_wall_time_ns",
        "camera_start_monotonic_time_ns",
        # "tau_futek",
    ],
)

if has_knee:
    track_raw_actuator_data("knee", knee)
if has_ankle:
    track_raw_actuator_data("ankle", ankle)

log_info = [
    "output_position",
    "output_velocity",
    "accelx",
    "motor_voltage",
    "motor_current",
    "battery_voltage",
    "battery_current",
    "case_temperature",
    "winding_temperature",
    "thermal_scaling_factor",
    "raw_thermal_current",
    "filtered_thermal_current",
    "raw_thermal_case_temperature",
    "filtered_thermal_case_temperature",
    "thermal_filter_rejections",
]
if has_knee:
    logger.track_attributes(knee, log_info)
if has_ankle:
    logger.track_attributes(ankle, log_info)

try:
    # GPIO.output(PIN_END, GPIO.HIGH)
    # torque_sensor_thread.start()
    # while GPIO.input(PIN_FALLING):
    #     time.sleep(DT)

    if camera_available:
        picam2.start_recording(encoder, video_file_name)
        camera_start_timestamp = time.time()
        camera_start_wall_time_ns = time.time_ns()
        camera_start_monotonic_time_ns = time.monotonic_ns()
        camera_is_recording = True

    with osl:
        for actuator in osl.actuators.values():
            actuator.set_control_mode(CONTROL_MODES.POSITION)
            actuator.set_position_gains(
                kp=300,
                ki=150,
                kd=100,
                ff=0,
            )

        osl.update()
        init_knee_position = knee.output_position if has_knee else 0.0
        init_ankle_position = ankle.output_position if has_ankle else 0.0

        if has_knee:
            knee.set_output_position(value=init_knee_position)
        if has_ankle:
            ankle.set_output_position(value=init_ankle_position)

        t_0 = 25
        A = 2 * np.pi

        for t in clock:
            osl.update()
            # tau_futek = torque_sensor_thread.get_latest_torque()

            if t < WAIT:
                position_command = 0.0
            elif t < t_0 + WAIT:
                position_command = A / t_0 * (t - WAIT)
            elif t < 3 * t_0 + WAIT:
                position_command = -A / t_0 * (t - WAIT) + 2 * A
            elif t < 4 * t_0 + WAIT:
                position_command = A / t_0 * (t - WAIT) - 4 * A
            else:
                position_command = 0.0
                if t >= 4 * t_0 + 2 * WAIT:
                    break

            if has_knee:
                knee.set_output_position(value=position_command + init_knee_position)
            if has_ankle:
                ankle.set_output_position(value=-position_command + init_ankle_position)
            logger.update()

        for t in clock:
            osl.update()
            # tau_futek = torque_sensor_thread.get_latest_torque()
            logger.update()

            if t > WAIT:
                break

    # GPIO.output(PIN_END, GPIO.LOW)
    time.sleep(5)
except KeyboardInterrupt:
    pass
finally:
    if camera_is_recording and picam2 is not None:
        picam2.stop_recording()
    # torque_sensor_thread.stop()
    # torque_sensor_thread.join()
    # GPIO.cleanup()
    logger.close()
