import sys
import opensourceleg.tools.units as units
from opensourceleg.osl import OpenSourceLeg
from picamera2 import Picamera2, Preview
from picamera2.encoders import H264Encoder, Quality
from libcamera import controls
import numpy as np

# Redirect to current folder
sys.path.append("./")

from hardware.futek import Big100NmFutek

# Mechanical Constants
GR_ACTPACK = 1                      # Direct actuator
GR_TRANS = 75/11
GR_BOSTONGEAR = 50                  # Gear Ratio from Boston Gear

kt = 0.11                           # back emf const

T = 25                              # Period Time

if __name__ == '__main__': 
    osl = OpenSourceLeg(frequency=200, file_name="osl")
    osl.add_joint(name="knee", port = "/dev/ttyACM0", gear_ratio=GR_ACTPACK*GR_BOSTONGEAR)
    osl.log.add_attributes(osl, ["timestamp"])
    osl.log.add_attributes(osl.knee, ["output_position"])
    osl.log.add_attributes(locals(), ["abs_comp_angle","abs_angle","joint_angle"])
    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(main={"size": (1600, 1200)}))
    picam2.set_controls({"FrameRate": 30})
    # picam2.set_controls({"AfMode": controls.AfModeEnum.Auto})

    encoder = H264Encoder()
    try: 
        # START RECORDING
        picam2.start_preview(Preview.DRM)
        picam2.start_recording(encoder, 'test.h264', quality=Quality.HIGH)
        # TODO: ADD FUTEK SENSOR
        # ACTPACK TEST BEGINS
        osl.knee.start()
        osl.knee.set_mode(osl.knee.control_modes.voltage)
        init_pos = osl.knee.output_position
        voltage = 2000 
        for t in osl.clock:
            
            osl.knee.set_voltage(value=voltage)
            osl.log.info(
                units.convert_from_default(osl.knee.output_position, units.position.rad)
            )
            osl.update()

            if (osl.clock.time()<=T):
                voltage = 2000
            elif (osl.clock.time()>T) and (osl.clock.time()<=T*3):
                voltage = -2000
            elif (osl.clock.time()>T*3) and (osl.clock.time()<=T*4):
                voltage = 2000
            else:
                break

            
            
    finally: 
        osl.knee.stop()
        picam2.stop_recording()