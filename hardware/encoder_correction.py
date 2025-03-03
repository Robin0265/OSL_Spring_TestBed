import numpy as np
import sys
from collections import deque
import threading
from time import sleep

def nonlinear_compensation(osl,enc,gear_ratio=0,Calibrate=True):
    if gear_ratio == 0:
        gear_ratio = osl.knee.gear_ratio
    if Calibrate:
        with osl:
            print(f"Calibrating {enc.name} encoder")
            osl.knee.set_mode(osl.knee.control_modes.voltage)
            osl.update(log_data=False)
            ang_0 = osl.knee.motor_position/gear_ratio
            joint_angs = []
            output_angs = []
            while abs(osl.knee.motor_position/gear_ratio - ang_0) < 4.0*np.pi:
                enc._update()
                joint_angs.append(enc.abs_ang)
                output_angs.append(osl.knee.motor_position/gear_ratio)
                osl.knee.set_voltage(-2000)  # mV
                osl.update(log_data=False)
                sleep(1/osl._frequency)
            osl.knee.set_voltage(0.0)
            joint_angs = np.array(joint_angs) + 2*np.pi
            output_angs = np.array(output_angs)    
            output_angs = output_angs - output_angs[0] + joint_angs[0]
            jp_map = joint_angs[(joint_angs >= -np.pi) & (joint_angs <= np.pi)]
            op_map = output_angs[(joint_angs >= -np.pi) & (joint_angs <= np.pi)]
            
            enc.P = np.polyfit(jp_map,op_map,25)
            enc.rotations = 0
            # p2 = np.polynomial.Polynomial.fit(jp_map,op_map,deg=5) # this is supposedly a more up-to-date version of polyfitting
                
            np.save(file=f"./joint_{enc.name}_cal.npy", arr=enc.P)
    else:
        enc.P = np.load(f"joint_{enc.name}_cal.npy")
        
def backlash_centering_by_hand(osl,enc,Calculate=True):
    
    if Calculate:
        # Calculate backlash
        bcklsh_angs = []
        output_angs = []
        print('Rotate the output back and forth through the backlash ROM')
        t = 0
        while t < 5:
            t = t + 1/osl._frequency
            enc._update()
            bcklsh_angs.append(enc.abs_comp_ang)
            osl.update(log_data=False)
            output_angs.append(osl.knee.output_position)
            sleep(1/osl._frequency)
        bcklsh_angs = np.array(bcklsh_angs)
        joint_ang_init = (np.max(bcklsh_angs) + np.min(bcklsh_angs))/2
        backlash_meas = abs(np.max(bcklsh_angs) - np.min(bcklsh_angs))
        output_angs_sorted = list(output_angs)
        output_angs_sorted.sort()
        index_median = np.int_(np.ceil(len(output_angs)/2))
        output_ang_init = output_angs_sorted[index_median]
        print("Backlash: {} rad".format(backlash_meas))
        # offset = joint_ang_init - osl.knee.output_position
        
        output_ang_adjust = output_ang_init
        counter = 0
        if output_ang_init > 0:
            dir = -1
        else:
            dir = 1
        while abs(output_ang_adjust) > np.pi:
            counter = counter + 1
            output_ang_adjust = output_ang_adjust + dir*2*np.pi
        offset_abs = joint_ang_init - output_ang_adjust
        offset = offset_abs + dir*counter*2*np.pi        
        
        np.save(file=f"./angle_offset.npy", arr=offset_abs)        
    else:
        osl.update(log_data=False)
        enc._update()
        joint_ang_init = enc.abs_comp_ang
        output_ang_init = osl.knee.output_position
        
        output_ang_adjust = output_ang_init
        counter = 0
        if output_ang_init > 0:
            dir = -1
        else:
            dir = 1
        while abs(output_ang_adjust) > np.pi:
            counter = counter + 1
            output_ang_adjust = output_ang_adjust + dir*2*np.pi
        offset_abs = np.load('angle_offset.npy')    
        offset = offset_abs + dir*counter*2*np.pi  

        # Check if the offset seems unreasonable
        if abs(output_ang_init + offset - joint_ang_init) > 8*np.pi/180:
            raise NotImplementedError("Offset seems too large, please recalculate")
                
    return offset,joint_ang_init

class Spring_Model:
    def __init__(self, spring_type):
        self.spring_type = spring_type
        self.parameters = self.get_spring_parameters(spring_type)
        self.backlash = self.parameters["backlash"]
        self.L = self.parameters["L"]
        self.c = self.parameters["c"]
        self.K = self.parameters["K"]
   
    def get_spring_parameters(self, descriptor):
        # Define a dictionary to map descriptor strings to their parameters
        descriptor_map = {
            "Intermediate SEA": {"K": 26.064505361566674, "backlash": 0.028421120738356, "L": 0.021445283871969, "c": 10.158709443070482},
            "Output SEA": {"K": 529.0906220831318, "backlash": 0.014283904430584, "L": 0.006095583139728, "c": 31.010879050173200},
            # Add more descriptors and their corresponding parameters here
        }
        
        # Return the parameters corresponding to the given descriptor
        return descriptor_map.get(descriptor, "Descriptor not found")
            
    def backlash_comp_smooth(self,ang_err):
        h = self.backlash/2.0
        
        # Set up logit function
        d = 1.0/4.0/self.c - self.c*(h+self.L)**2.0
        y_logit = lambda x: d*np.log((0.5 + self.c*x)/(0.5 - self.c*x))
        
        if abs(ang_err) < (h + self.L):
            deflection = y_logit(ang_err)
        elif ang_err >= 0:
            deflection = ang_err - h
        else:
            deflection = ang_err + h
            
        return deflection  

    def backlash_comp_inverse(self,deflection_des):
        h = self.backlash/2.0
        
        # Set up logit function
        d = 1.0/4.0/self.c - self.c*(h+self.L)**2.0
        y_sigmoid = lambda x: (np.exp(x/d) - 1)/(2*self.c*(np.exp(x/d) + 1))
        
        if abs(deflection_des) < self.L:
            ang_cmd = y_sigmoid(deflection_des)
        elif deflection_des >= 0:
            ang_cmd = deflection_des + h
        else:
            ang_cmd = deflection_des - h
            
        return ang_cmd  
    
class Basic_LPF:
    """
    Super simple first order lowpass IIR filter
    """
    def __init__(self, start_value, f_sample, f_cutoff):
        self._previous = start_value        
        self.new_fraction = 2*np.pi*f_cutoff/f_sample
        
    def update(self, new_value):
        output = self._previous * (1-self.new_fraction) + self.new_fraction * new_value
        self._previous = output
        return output

class Median_Filter:
    """
        Median Filter with adjustable window
    """
    
    def __init__(self, start_value, window):
        self.vals = [start_value]
        # self.vals.append(start_value)
        self.window = window
        
    def update(self, new_value):
        
        # Median filter on measurement
        if len(self.vals) > (self.window-1):
            self.vals.pop(0)
            self.vals.append(new_value)
        else:
            self.vals.append(new_value)       
            
        vals_sorted = list(self.vals)
        vals_sorted.sort()
        index = (np.ceil(len(self.vals)/2)).astype(int) - 1 # subtract 1 because of zero indexing
        val_filt = vals_sorted[index] 
        
        return val_filt
            
def current_control(tau_output,vel_output,GR_output,version='simple'):
    k_t = 0.1108
    GR_actpack = 9
    fc = 0.171 # Nm at actuator output
    fg = 0.0821 # Nm/A at actuator output
    smooth_sign = lambda x: x/(abs(x)+0.05)
    smoother_sign = lambda x: x/(abs(x)+5.0)

    tau_a = tau_output/GR_output # calculate desired ActPack torque
    vel_a = vel_output*GR_output
    
    num = tau_a + smooth_sign(vel_a)*fc # numerator
    den = k_t*GR_actpack - smoother_sign(tau_a*vel_a + abs(vel_a)*fc)*fg # denominator
    Iq = num/den*1000 # q-axis current command (mA)
    I_simple = 1000.0*tau_a/(GR_actpack*k_t)
    
    if version == 'simple':
        return I_simple
    elif version == 'full':
        return Iq
    else:
        raise NotImplementedError("Version not recognized")
    
class Moving_Avg_Filter:
    """
        Moving Average Filter with adjustable window
    """
    
    def __init__(self, start_value, window):
        self.vals = [start_value]
        # self.vals.append(start_value)
        self.window = window
        
    def update(self, new_value):
        
        # Median filter on measurement
        if len(self.vals) > (self.window-1):
            self.vals.pop(0)
            self.vals.append(new_value)
        else:
            self.vals.append(new_value)       
            
        val_filt = sum(self.vals)/len(self.vals)
        
        return val_filt
    
class TorqueSensorThread(threading.Thread):
    def __init__(self, sensor, update_interval=0.002):
        super().__init__()
        self.sensor = sensor
        self.update_interval = update_interval
        self._stop_event = threading.Event()
        self.latest_torque = 0.0
        self.lock = threading.Lock()

    def run(self):
        while not self._stop_event.is_set():
            self.sensor.update()
            with self.lock:
                self.sensor.get_torque()
                self.latest_torque = self.sensor.torque
            sleep(self.update_interval)

    def stop(self):
        self._stop_event.set()

    def get_latest_torque(self):
        with self.lock:
            return self.latest_torque
        
def calc_velocity_timescale(frequency):
    return (0.0019419*frequency + 0.320216)/frequency # coefficients calculated in measurement_analysis.m

# ###### This is all coming from https://www.samproell.io/posts/yarppg/yarppg-live-digital-filter/ #################################

# class LiveFilter:
#     """Base class for live filters.
#     """
#     def process(self, x):
#         # do not process NaNs
#         if np.isnan(x):
#             return x

#         return self._process(x)

#     def __call__(self, x):
#         return self.process(x)

#     def _process(self, x):
#         raise NotImplementedError("Derived class must implement _process")
    

# class LiveLFilter(LiveFilter):
#     def __init__(self, b, a):
#         """Initialize live filter based on difference equation.

#         Args:
#             b (array-like): numerator coefficients obtained from scipy.
#             a (array-like): denominator coefficients obtained from scipy.
#         """
#         self.b = b
#         self.a = a
#         self._xs = deque([0] * len(b), maxlen=len(b))
#         self._ys = deque([0] * (len(a) - 1), maxlen=len(a)-1)
        
#     def _process(self, x):
#         """Filter incoming data with standard difference equations.
#         """
#         self._xs.appendleft(x)
#         y = np.dot(self.b, self._xs) - np.dot(self.a[1:], self._ys)
#         y = y / self.a[0]
#         self._ys.appendleft(y)

#         return y
    