# Series Spring Calibration Process
# Compatible with opensourceleg 3.5.x
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

osl = OpenSourceLeg(
    tag="series_spring_calibration",
    actuators={
        "knee": FilteredDephyActuator(
            tag="knee",
            port="/dev/ttyACM0",
            gear_ratio=GR_ACTPACK * GR_BOSTONGEAR,
            frequency=FREQUENCY,
            dephy_log=True,
        ),
        "ankle": FilteredDephyActuator(
            tag="ankle",
            port="/dev/ttyACM1",
            gear_ratio=GR_ACTPACK * GR_BOSTONGEAR,
            frequency=FREQUENCY,
            dephy_log=True,
        ),
    },
    sensors={},
)

clock = SoftRealtimeLoop(dt=DT, report=True)
logger = Logger(log_path="./logs", file_name=file_name)
picam2 = None
encoder = None
camera_available = False
camera_info = Picamera2.global_camera_info()
if camera_info:
    picam2 = Picamera2()
    picam2.configure(
        picam2.create_video_configuration(
            main={"size": (640, 480)},
        )
    )
    picam2.set_controls({"FrameRate": 30})
    encoder = H264Encoder()
    camera_available = True
else:
    print("WARNING: No Picamera2 camera detected; continuing without video recording.")

# torqueSensor = Big100NmFutek()
# torque_sensor_thread = TorqueSensorThread(torqueSensor, update_interval=DT)

position_command = 0.0
camera_start_timestamp = 0.0
camera_is_recording = False
# tau_futek = 0.0

logger.track_function(
    [
        lambda: time.time(),
        lambda: FREQUENCY,
        lambda: position_command,
        lambda: int(camera_available),
        lambda: camera_start_timestamp,
        # lambda: tau_futek,
    ],
    [
        "timestamp",
        "frequency",
        "position_command",
        "camera_available",
        "camera_start_timestamp",
        # "tau_futek",
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
    "case_temperature",
    "winding_temperature",
    "thermal_scaling_factor",
    "raw_thermal_current",
    "filtered_thermal_current",
    "raw_thermal_case_temperature",
    "filtered_thermal_case_temperature",
    "thermal_filter_rejections",
]
logger.track_attributes(osl.knee, log_info)
logger.track_attributes(osl.ankle, log_info)

try:
    # GPIO.output(PIN_END, GPIO.HIGH)
    # torque_sensor_thread.start()
    # while GPIO.input(PIN_FALLING):
    #     time.sleep(DT)

    if camera_available:
        picam2.start_recording(encoder, video_file_name)
        camera_start_timestamp = time.time()
        camera_is_recording = True

    with osl:
        osl.knee.set_control_mode(CONTROL_MODES.POSITION)
        osl.ankle.set_control_mode(CONTROL_MODES.POSITION)

        osl.knee.set_position_gains(
            kp=300,
            ki=150,
            kd=100,
            ff=0,
        )
        osl.ankle.set_position_gains(
            kp=300,
            ki=150,
            kd=100,
            ff=0,
        )

        osl.update()
        init_pos_left = osl.ankle.output_position
        init_pos_right = osl.knee.output_position

        osl.ankle.set_output_position(value=init_pos_left)
        osl.knee.set_output_position(value=init_pos_right)

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

            osl.knee.set_output_position(value=position_command + init_pos_right)
            osl.ankle.set_output_position(value=-position_command + init_pos_left)
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
