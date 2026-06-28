# Series Spring Calibration Process
# Compatible with Pi4B + PiCamera2
import sys
from opensourceleg.osl import OpenSourceLeg
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
import numpy as np
# import RPi.GPIO as GPIO
import time
from time import strftime, time

# Redirect to current folder
sys.path.append("./")

from hardware.futek import Big100NmFutek
from hardware.encoder_correction import TorqueSensorThread

# Mechanical Constants
GR_ACTPACK = 1                      # Actuator Geabox
GR_TRANS = 75/11
GR_BOSTONGEAR = 50                  # Gear Ratio from Boston Gear

T = 10                              # Period Time
TIME_TO_STEP = 1.0
FREQUENCY = 200
DT = 1 / FREQUENCY
WAIT = 3

# Hard safety limits. These match the Pi5 stiffness tests and are independent
# from the calibration trajectory.
MOTOR_CURRENT_SAFETY_LIMIT_A = 8.5
WINDING_TEMPERATURE_SAFETY_LIMIT_C = 90.0
BATTERY_VOLTAGE_MAX_V = 43.0
BATTERY_VOLTAGE_MIN_V = 25.0

PIN_FALLING = 6
PIN_END = 26

VOLT = 2000 # in mV
DIRECTION = 1
FLAG = 1
cnt = 0


def _get_joint_temperature(joint):
    if hasattr(joint, "winding_temperature"):
        return joint.winding_temperature
    if hasattr(joint, "case_temperature"):
        return joint.case_temperature
    return None


def check_joint_safety(joint, name):
    temperature = _get_joint_temperature(joint)
    if (
        temperature is not None
        and temperature > WINDING_TEMPERATURE_SAFETY_LIMIT_C
    ):
        raise ValueError("{} motor above thermal limit: {} C".format(name, temperature))

    battery_voltage_v = joint.battery_voltage / 1000
    if battery_voltage_v > BATTERY_VOLTAGE_MAX_V:
        print("{} voltage {}".format(name, battery_voltage_v))
        raise ValueError(
            "{} battery voltage above {} V".format(name, BATTERY_VOLTAGE_MAX_V)
        )
    if battery_voltage_v < BATTERY_VOLTAGE_MIN_V:
        print("{} voltage {}".format(name, battery_voltage_v))
        raise ValueError(
            "{} battery voltage below {} V".format(name, BATTERY_VOLTAGE_MIN_V)
        )

    motor_current_a = joint.motor_current / 1000
    if np.abs(motor_current_a) > MOTOR_CURRENT_SAFETY_LIMIT_A:
        raise ValueError(
            "{} motor current too high: {} A".format(name, motor_current_a)
        )


def check_safety(osl):
    check_joint_safety(osl.knee, "knee")
    check_joint_safety(osl.ankle, "ankle")

# GPIO.setmode(GPIO.BCM)
# GPIO.setup(PIN_FALLING, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
# GPIO.setup(PIN_END, GPIO.OUT)

osl = OpenSourceLeg(frequency=500, file_name='Calibration'+strftime("%y%m%d_%H%M%S"))

osl.clock.report = True

osl.add_joint(name="knee", port = "/dev/ttyACM0", gear_ratio=GR_ACTPACK*GR_BOSTONGEAR, dephy_log=True)
osl.add_joint(name="ankle", port = "/dev/ttyACM1", gear_ratio=9.0, dephy_log=True)
    
# torqueSensor = Big100NmFutek()
# torque_sensor_thread = TorqueSensorThread(torqueSensor, update_interval=1.0/osl._frequency)    

osl.log.add_attributes(osl, ["timestamp", "_frequency"])
log_info = ["output_position", "output_velocity", "accelx", 
            "motor_voltage", "motor_current", "battery_voltage", 
            "battery_current"]
osl.log.add_attributes(osl.knee, log_info)
osl.log.add_attributes(osl.ankle, log_info)
# osl.log.add_attributes(locals(), ["tau_futek"])

try:
    # GPIO.output(PIN_END, GPIO.HIGH)
    # torque_sensor_thread.start()
    # while (GPIO.input(PIN_FALLING)): 
        # time.sleep(1/500)
    
    with osl: 
        
        osl.knee.set_mode(osl.knee.control_modes.position)
        osl.ankle.set_mode(osl.ankle.control_modes.position)
        osl.knee.set_position_gains(
            kp = 300, 
            ki = 150, 
            kd = 100, 
            ff = 0,
        )
        osl.ankle.set_position_gains(
            kp = 300, 
            ki = 150, 
            kd = 100, 
            ff = 0,
        )
        osl.update()
        check_safety(osl)
        init_pos_left = osl.ankle.output_position
        init_pos_right = osl.knee.output_position
        osl.ankle.set_output_position(position=init_pos_left)
        osl.knee.set_output_position(position=init_pos_right)
        # init_pos = pos
        # i = 0
        # init_time = osl.clock.time()
        # T = osl.clock.time_since()
        t_0 = 25
        A = 2*np.pi
        
        for t in osl.clock:
            osl.update()
            check_safety(osl)
            
            if t < WAIT:
                position_command = 0
            elif t < t_0 + WAIT:
                position_command = A/t_0*(t-WAIT)
            elif t < 3*t_0 + WAIT:
                position_command = -A/t_0*(t-WAIT) + 2*A
            elif t < 4*t_0 + WAIT:
                position_command = A/t_0*(t-WAIT) - 4*A
            else:
                position_command = 0
                if t >= 4*t_0 + 2*WAIT:
                    break
            
            osl.knee.set_output_position(position_command + init_pos_right)
            osl.ankle.set_output_position(-position_command + init_pos_left)
            
        for t in osl.clock:
            # tau_futek = torque_sensor_thread.get_latest_torque()
            osl.update()
            check_safety(osl)
            if t > WAIT:
                break
        
    # GPIO.output(PIN_END, GPIO.LOW)
    time.sleep(5)
except KeyboardInterrupt: 
    # torque_sensor_thread.stop()
    # torque_sensor_thread.join()  
    # GPIO.cleanup()
    exit()
finally: 
    # torque_sensor_thread.stop()
    # torque_sensor_thread.join()  
    # GPIO.cleanup()
    exit()
