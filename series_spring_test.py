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


T = 25                              # Period Time

TIME_TO_STEP = 1.0
FREQUENCY = 200
DT = 1 / FREQUENCY

k = np.pi/(48*50)

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
        osl.knee.set_mode(osl.knee.control_modes.position)

        osl.knee.update()
        pos = osl.knee.output_position
        init_pos = pos
        i = 0
        for t in osl.clock:
            
            pos = pos + k
            
            osl.knee.set_position_gains(
                # kp=200, 
                # ki=200, 
                # kd=50, 
            )
            osl.knee.set_output_position(position=pos)
            osl.knee.update()
            current_pos = osl.knee.output_position
            
            # osl.log.info(
            #     units.convert_from_default(osl.knee.output_position, units.position.rad)
            # )
            print("####")
            print(current_pos)
            print("----")
            print(init_pos)
            
            # print(current_pos)
            if (current_pos - init_pos)>np.pi/2:
                k = -np.pi/(48*50)
                i = i + 1
            elif (current_pos - init_pos)<-np.pi/2:
                k = np.pi/(48*50)
                i = i + 1
            if ((int(i/50)==2) & (abs(current_pos - init_pos)<1e-1)):
                break

            
    finally: 
        osl.knee.stop()
        picam2.stop_recording()