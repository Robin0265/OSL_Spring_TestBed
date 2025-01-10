import sys
import opensourceleg.tools.units as units
from opensourceleg.osl import OpenSourceLeg
# from picamera2 import Picamera2, Preview
# from picamera2.encoders import H264Encoder, Quality
# from libcamera import controls
import numpy as np
import time

# Redirect to current folder
sys.path.append("./")

from hardware.futek import Big100NmFutek

# Mechanical Constants
GR_ACTPACK = 1                      # Direct actuator
GR_TRANS = 75/11
GR_BOSTONGEAR = 50                  # Gear Ratio from Boston Gear


T = 10                              # Period Time

TIME_TO_STEP = 1.0
FREQUENCY = 200
DT = 1 / FREQUENCY

k = np.pi/(48*50)

if __name__ == '__main__': 
    osl = OpenSourceLeg(frequency=200, file_name="osl")
    osl.add_joint(name="knee", port = "/dev/ttyACM0", gear_ratio=GR_ACTPACK*GR_BOSTONGEAR, dephy_log=True)
    osl.log.add_attributes(osl, ["timestamp", "_frequency"])
    log_info = ["output_position", "output_velocity", "accelx", 
                "motor_voltage", "motor_current", "battery_voltage", 
                "battery_current"]
    osl.log.add_attributes(osl.knee, log_info)

    # picam2 = Picamera2()
    # picam2.configure(picam2.create_video_configuration(main={"size": (1600, 1200)}))
    # picam2.set_controls({"FrameRate": 30})
    # picam2.set_controls({"AfMode": controls.AfModeEnum.Auto})
    # encoder = H264Encoder()

    with osl: 
        # START RECORDING
        # picam2.start_preview(Preview.DRM)
        # picam2.start_recording(encoder, 'test.h264', quality=Quality.HIGH)
        # TODO: ADD FUTEK SENSOR
        # ACTPACK TEST BEGINS


        osl.knee.start()
        osl.knee.set_mode(osl.knee.control_modes.position)
        osl.knee.set_position_gains(
            kp = 150, 
            ki = 150, 
            kd = 100, 
            ff = 0,
        )
        osl.knee.update()
        init_pos = osl.knee.output_position
        osl.knee.set_output_position(position=init_pos)
        # init_pos = pos
        i = 0
        init_time = osl.clock.time()
        T = osl.clock.time_since()
        t_0 = 25
        A = 2*np.pi
        for t in osl.clock:
            osl.update()
            if t < t_0:
                position_command = A/t_0*t
            elif t < 3*t_0:
                position_command = -A/t_0*t + 2*A
            elif t < 4*t_0:
                position_command = A/t_0*t - 4*A
            else:
                break

            position_command = position_command + init_pos
            osl.knee.set_output_position(position=position_command)
            osl.log.update()
            
    # finally: 
        # picam2.stop_recording()