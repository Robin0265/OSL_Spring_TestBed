import sys
from opensourceleg.osl import OpenSourceLeg

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
    with osl:
        osl.knee.set_mode(osl.knee.control_modes.voltage)
        for t in osl.clock:
            osl.update()