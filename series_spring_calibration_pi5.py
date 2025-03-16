# Series Spring Calibration Process
# Compatible with Pi4B + PiCamera2
import sys
from opensourceleg.osl import OpenSourceLeg
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
import numpy as np
import RPi.GPIO as GPIO
import time

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

PIN_FALLING = 6
PIN_END = 26

VOLT = 2000 # in mV
DIRECTION = 1
FLAG = 1
cnt = 0

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_FALLING, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(PIN_END, GPIO.OUT)

osl = OpenSourceLeg(frequency=500, file_name="osl_calib_0312")

osl.clock.report = True

osl.add_joint(name="knee", port = "/dev/ttyACM0", gear_ratio=GR_ACTPACK*GR_BOSTONGEAR, dephy_log=True)
# osl.add_joint(name="ankle", port = "/dev/ttyACM1", gear_ratio=GR_ACTPACK*GR_BOSTONGEAR, dephy_log=True)
    
torqueSensor = Big100NmFutek()
torque_sensor_thread = TorqueSensorThread(torqueSensor, update_interval=1.0/osl._frequency)    

osl.log.add_attributes(osl, ["timestamp", "_frequency"])
log_info = ["output_position", "output_velocity", "accelx", 
            "motor_voltage", "motor_current", "battery_voltage", 
            "battery_current"]
osl.log.add_attributes(osl.knee, log_info)
# osl.log.add_attributes(osl.ankle, log_info)
osl.log.add_attributes(torqueSensor, ["torque"])

try:
    GPIO.output(PIN_END, GPIO.HIGH)
    
    while (GPIO.input(PIN_FALLING)): 
        time.sleep(1/500)
    
    with osl: 
        osl.knee.start()
        # osl.ankle.start()
        torque_sensor_thread.start() 
        osl.knee.set_mode(osl.knee.control_modes.voltage)
        # osl.knee.set_mode(osl.ankle.control_modes.voltage)
        osl.update()
        # init_pos_left = osl.ankle.output_position
        init_pos = osl.knee.output_position
        t_0 = 25
        A = 2*np.pi
        voltage_command = 0
        for t in osl.clock:
            tau_futek = torque_sensor_thread.get_latest_torque()
            osl.update()
            current_pos = osl.knee.output_position - init_pos; 
            
            if (current_pos > 0 and DIRECTION == 1 and cnt >= 2 and t > WAIT):
                voltage_command = 0
                osl.knee.set_voltage(0)
                # osl.ankle.set_voltage(0)
                t_stop = osl.clock.time()
                break
            elif (current_pos < -2 * np.pi and DIRECTION == -1 and t > WAIT):
                voltage_command = VOLT
                DIRECTION = 1
                cnt += 1
            elif (current_pos > 2 * np.pi and DIRECTION == 1 and t > WAIT):
                voltage_command = - VOLT
                DIRECTION = -1
                cnt += 1
            elif (current_pos > 0 and DIRECTION == 1 and t > WAIT):
                voltage_command = VOLT
            elif (current_pos > 0 and DIRECTION == 1 and t < WAIT):
                voltage_command = 0
            
            osl.knee.set_voltage(voltage_command)
            # osl.ankle.set_voltage(-voltage_command)
            
        for t in osl.clock:
            tau_futek = torque_sensor_thread.get_latest_torque()
            osl.update()
            if t > WAIT:
                break
        
    GPIO.output(PIN_END, GPIO.LOW)
    time.sleep(5)
except KeyboardInterrupt: 
    torque_sensor_thread.stop()
    torque_sensor_thread.join()  
    GPIO.cleanup()
    exit()
finally: 
    torque_sensor_thread.stop()
    torque_sensor_thread.join()  
    GPIO.cleanup()
    exit()
