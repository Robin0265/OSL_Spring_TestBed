# Series Spring Calibration Process
# Compatible with Pi4B + PiCamera2
import sys
from opensourceleg.osl import OpenSourceLeg
from picamera2 import Picamera2, Preview
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


osl = OpenSourceLeg(frequency=500, file_name="osl_calib_0314")

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
    picam2 = Picamera2()
    picam2.configure(
        picam2.create_video_configuration(
            raw={"size":(1640,1232)}, # raw size 
            main={"size": (640, 480)} # scaled size
            )
        )
    picam2.set_controls({"FrameRate": 30})
    encoder = H264Encoder()
    picam2.start_preview(Preview.DRM)
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
        
        picam2.start_recording(encoder, 'Calib_0314.h264')
        
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

except KeyboardInterrupt: 
    
    picam2.stop_preview()
    picam2.stop_recording()
    torque_sensor_thread.stop()
    torque_sensor_thread.join()  
    exit()
finally: 
    picam2.stop_preview()
    picam2.stop_recording()
    torque_sensor_thread.stop()
    torque_sensor_thread.join()  
    exit()
