import sys
import opensourceleg.tools.units as units
from opensourceleg.osl import OpenSourceLeg
from picamera2 import Picamera2, Preview
from picamera2.encoders import H264Encoder, Quality

# Redirect to current folder
sys.path.append("./")

from hardware.futek import Big100NmFutek

# Mechanical Constants
GR_ACTPACK = 9
GR_TRANS = 75/11
GR_BOSTONGEAR = 50

if __name__ == '__main__': 
    osl = OpenSourceLeg(frequency=200, file_name="osl")
    osl.add_joint(name="knee", port = "/dev/ttyACM0", gear_ratio=GR_ACTPACK)
    osl.log.add_attributes(osl, ["timestamp"])
    osl.log.add_attributes(osl.knee, ["output_position"])
    osl.log.add_attributes(locals(), ["abs_comp_angle","abs_angle","joint_angle"])
    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(main={"size": (800, 600)}))

    encoder = H264Encoder()
    try: 
        # START RECORDING
        picam2.start_preview(Preview.DRM)
        picam2.start_recording(encoder, 'test.h264', quality=Quality.HIGH)
        # TODO: ADD FUTEK SENSOR
        # ACTPACK TEST BEGINS
        osl.knee.start()
        osl.knee.set_mode(osl.knee.control_modes.voltage)
        for t in osl.clock:
            osl.knee.set_voltage(value=2000)
            osl.log.info(
                units.convert_from_default(osl.knee.motor_position, units.position.deg)
            )
            osl.update()
    finally: 
        osl.knee.stop()
        picam2.stop_recording()