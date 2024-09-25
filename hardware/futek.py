from hardware.AdcManager import AdcManager
import numpy as np
from time import time, sleep


class Big100NmFutek(AdcManager):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.torque = -42.0
        self.bias = 0
    def get_torque(self):
        torque_rating = 100 # 100 Nm = 5V
        self.torque = (self.volts-2.5-self.bias)/2.5*torque_rating
    def calibrate_loadcell(self,Calibrate=True):
        if Calibrate:
            self.update()
            voltage_cal = []
            print("calibrating loadcell")
            start_time = time()
            while time() - start_time < 5:
                self.update()
                voltage_cal.append(self.volts)
                
            avg_volt = np.mean(np.array(voltage_cal))
            bias = avg_volt - 2.5
            print("Bias: {} V".format(bias))
            self.update()
            self.bias = bias
            np.save(file=f"./futek_offset.npy", arr=bias)        
        else:
            bias = np.load('futek_offset.npy')    
            print("Setting Bias: {} V".format(bias))
            self.bias = bias
            
        return bias
    def set_bias(self,bias):
        self.bias = bias
        print("Bias set to: {} V".format(bias))

        
if __name__ == "__main__":

    futek = Big100NmFutek()
    sleep(1)

    futek.calibrate_loadcell()

    try:
        while True:
            futek.update()
            futek.get_torque()
            print('Torque: %d Nm'% futek.torque)
            sleep(0.2)

    except KeyboardInterrupt:
        print('  Exiting.')
