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
WAIT = 5

PIN_FALLING = 6
PIN_END = 26

VOLT = 2000 # in mV
DIRECTION = 1
FLAG = 1
cnt = 0

osl = OpenSourceLeg(frequency=500, file_name="osl_calib_3")
    
osl.clock.report = True
    
osl.add_joint(name="knee", port = "/dev/ttyACM0", gear_ratio=GR_ACTPACK*GR_BOSTONGEAR, dephy_log=True)
    
torqueSensor = Big100NmFutek()
torque_sensor_thread = TorqueSensorThread(torqueSensor, update_interval=1.0/osl._frequency)    

osl.log.add_attributes(osl, ["timestamp", "_frequency"])
log_info = ["output_position", "output_velocity", "accelx", 
            "motor_voltage", "motor_current", "battery_voltage", 
            "battery_current"]
osl.log.add_attributes(osl.knee, log_info)
osl.log.add_attributes(torqueSensor, ["torque"])
osl.log.add_attributes(locals(), ["freq_command","position_command","torque_command","tau_meas","joint_ang_pre","output_ang_pre","t","i_command","ang_err","ang_err_filt","deflection","deflection_filt","tau_futek","p","intgrl","d"])

try:
    
    with osl: 
        osl.knee.start()
        torque_sensor_thread.start() 
        osl.knee.set_mode(osl.knee.control_modes.voltage)
        osl.knee.update()
        init_pos = osl.knee.output_position
        # init_pos = pos
        init_time = osl.clock.time()
        T = osl.clock.time_since()
        t_0 = 25
        A = 2*np.pi
        voltage_command = VOLT # in Volts
        for t in osl.clock:
            tau_futek = torque_sensor_thread.get_latest_torque()
            osl.update()
            current_pos = osl.knee.output_position - init_pos; 
            if (current_pos > 2 * np.pi and DIRECTION == 1):
                voltage_command = - VOLT
                DIRECTION = -1
                cnt += 1
            elif (current_pos < -2 * np.pi and DIRECTION == -1):
                voltage_command = VOLT
                DIRECTION = 1
                cnt += 1
            elif (current_pos > 0 and DIRECTION == 1 and cnt == 2):
                break
                 
            # position_command = -position_command
            # position_command_right = position_command + init_pos
            osl.knee.set_voltage(voltage_command)
    
except KeyboardInterrupt: 
    torque_sensor_thread.stop()
    torque_sensor_thread.join()  
    exit()
finally: 
    torque_sensor_thread.stop()
    torque_sensor_thread.join()  
    exit()
