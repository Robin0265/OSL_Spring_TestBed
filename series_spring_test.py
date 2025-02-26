# Series Spring Calibration Process
# Compatible with Pi4B + PiCamera2
import sys
from opensourceleg.osl import OpenSourceLeg
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
import numpy as np

# Redirect to current folder
sys.path.append("./")

from hardware.futek import Big100NmFutek

# Mechanical Constants
GR_ACTPACK = 1                      # Actuator Geabox
GR_TRANS = 75/11
GR_BOSTONGEAR = 50                  # Gear Ratio from Boston Gear

T = 10                              # Period Time
TIME_TO_STEP = 1.0
FREQUENCY = 200
DT = 1 / FREQUENCY
WAIT = 5

if __name__ == '__main__': 
    osl = OpenSourceLeg(frequency=200, file_name="osl_calib")
    
    osl.clock.report = True
    
    osl.add_joint(name="knee", port = "/dev/ttyACM1", gear_ratio=GR_ACTPACK*GR_BOSTONGEAR, dephy_log=True)
    osl.add_joint(name="ankle", port = "/dev/ttyACM0", gear_ratio=GR_ACTPACK*GR_BOSTONGEAR, dephy_log=True)
    
    osl.log.add_attributes(osl, ["timestamp", "_frequency"])
    log_info = ["output_position", "output_velocity", "accelx", 
                "motor_voltage", "motor_current", "battery_voltage", 
                "battery_current"]
    osl.log.add_attributes(osl.knee, log_info)
    osl.log.add_attributes(osl.ankle, log_info)

    # picam2 = Picamera2()
    # picam2.configure(
    #         picam2.create_video_configuration(
    #             raw={"size":(1640,1232)}, # raw size 
    #             main={"size": (640, 480)} # scaled size
    #             )
    #         )
    # picam2.set_controls({"FrameRate": 30})
    # encoder = H264Encoder()
    # picam2.start_recording(encoder, 'test_modified5.h264')

    with osl: 
        osl.knee.start()
        osl.ankle.start()
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
        osl.knee.update()
        osl.ankle.update()
        init_pos_left = osl.ankle.output_position
        init_pos_right = osl.knee.output_position
        osl.ankle.set_output_position(position=init_pos_left)
        osl.knee.set_output_position(position=init_pos_right)
        # init_pos = pos
        i = 0
        init_time = osl.clock.time()
        T = osl.clock.time_since()
        t_0 = 25
        A = 2*np.pi
        
        for t in osl.clock:
            osl.update()
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
            
            position_command = -position_command
            position_command_right = position_command + init_pos_right
            position_command_left = -position_command + init_pos_left
            osl.knee.set_output_position(position=position_command_right)
            osl.ankle.set_output_position(position=position_command_left)
            
        # picam2.stop_recording()