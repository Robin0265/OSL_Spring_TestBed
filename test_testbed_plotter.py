from operator import neg
import numpy as np
import matplotlib.pyplot as plt
# from model_params import *
import scipy.io
from Vision.plot_cal import circle_find_3_points
from Vision.video_reading_test import test, CircleAnnotator, CircleTracker
import os.path
from scipy.signal import argrelextrema
from scipy.signal import find_peaks
# from scipy.integrate import cumtrapz

# Model Parameters (tuned here)
rad_2_enc = 2607.59458762
J_m = 0.12e-3 # Kg m^2 [LeePanRouse2019IROS]
b_m = 0.16e-3 # Nm s / rad [LeePanRouse2019IROS]
c_m = 9.1e-3 # Nm [LeePanRouse2019IROS]
J_t = 8e-5 # Kg m^2 empirical--tuned to fit torque sensor data
b_t = 2e-5*50 # Nm s/rad (old_plotme.py)
c_t = 2.5/50 # Nm (old_plotme.py)
K_tau = 0.14 # Nm/A [LeePanRouse2019IROS]
K_emf = 0.14 # Vs/rad [LeePanRouse2019IROS]
R_phase = 3./2.*186e-3 # Ohms [LeePanRouse2019IROS] + empirical multiplication by 5 (presumbably losses in the circuit or voltage/current sensor?)
K_spring = 120# 150 # Nm/rad estimated
spring_degree_limit = 15 # degrees
# Position Control Parameters
# kp=200 # was 50
# ki=0 # was 3
dephy_kp_gain_conversion = .025# 1e0 # Converts dephy units to q-axis volts per radian
sign = np.sign

def simple_velocity(X, T, n=20, s=.001):
    return (X[n:]-X[:-n]) / (T[n:]-T[:-n]) *s

def simple_midpoint(T, n=20):
    return T[n:]*.5+.5*T[:-n]

def cumulative_sum(array):
    array = np.array(array)
    x = 0
    for i in range(len(array)):
        x+=array[i]
        array[i]=x
    return array

conversion_to_spring_frame_degrees= lambda x: x/(45.5111*50.)
ticks_to_motor_radians = lambda x: x*(1)
# torque_sensor_callibrated_volts_2_Nm = np.vectorize(lambda x: 1.0014*x+0.0044 if x>=0 else .9998*x + 0.0018)
torque_sensor_callibrated_volts_2_Nm = np.vectorize(lambda x: 20*x)

class SEATestbedPlotter(object):
    def __init__(self, act0_file, act1_file, adc0_file=None):

        act0 = np.loadtxt(act0_file, delimiter=",", skiprows=1)
        act1 = np.loadtxt(act1_file, delimiter=",", skiprows=1)
        self.has_adc = self.has_traj = self.has_ctrl = False

        if adc0_file: 
            adc0 = np.loadtxt(adc0_file, delimiter=",", skiprows=1)
            self.has_adc = True

        self._data=dict(headers=[],data=[])
        self._line_index=-1

        init_pi_time = min([act0[0,0],act1[0,0],adc0[0,0]]) if self.has_adc else (min([act0[0,0],act1[0,0]]))  #, ctrldat[0,0]])
        self.initial_angle_offset = 0

        self.add_line("a0_t", "pi_time", act0[:,0]-init_pi_time)
        self.add_line("a0_ts", "State time", act0[:,1])
        self.add_line("a0_x", "Motor enc angle", act0[:,2]-act0[0,2])
        self.add_line("a0_xd", "Motor enc velocity", act0[:,3])
        self.add_line("a0_xdd", "Motor enc acceleration", act0[:,4])  
        self.add_line("a0_vm", "Motor deph voltage", act0[:,5]) 
        self.add_line("a0_im", "Motor deph current", act0[:,6])  
        self.add_line("a0_vb", "Battery deph voltage", act0[:,7])
        self.add_line("a0_ib", "Battery deph current", act0[:,8])
        self.add_line("tau", "Futek Torque Reading", act0[:, -1])

        self.add_line("a1_t", "pi_time", act1[:,0]-init_pi_time)
        self.add_line("a1_ts", "State time", act1[:,1])
        self.add_line("a1_x", "Motor enc angle", -act1[:,2]+act1[:,2])
        self.add_line("a1_xd", "Motor enc velocity", act1[:,3])
        self.add_line("a1_xdd", "Motor enc acceleration", act1[:,4])  
        self.add_line("a1_vm", "Motor deph voltage", act1[:,5]) 
        self.add_line("a1_im", "Motor deph current", act1[:,6])  
        self.add_line("a1_vb", "Battery deph voltage", act1[:,7])
        self.add_line("a1_ib", "Battery deph current", act1[:,8])

        if self.has_adc: self.add_line("adc_t", "ADC pi Time", adc0[:,0]-init_pi_time)
        if self.has_adc: self.add_line("adc_v", "ADC Voltage", adc0[:,1])
        if self.has_adc: self.add_line("adc_d", "ADC sampling duration (seconds)", adc0[:,2])

        self.generate_easy_derived_signals()

    def add_line(self, shortname, name, value):
        self._line_index+=1
        self._data["headers"].append(name)
        self._data["data"].append(value)
        setattr(self, shortname, np.array(self._data["data"][self._line_index]))
        return self._line_index

    def generate_easy_derived_signals(self):
        if self.has_adc: self.add_line("futek_v", "torque sensor voltage signal", 2*(self.adc_v-self.adc_v[0]))
        # if self.has_adc: self.add_line("tau", "torque sensor torque signal", torque_sensor_callibrated_volts_2_Nm(self.futek_v))
        self.add_line("v0", "q-axis voltage act0", self.a0_vm*1e-3 * np.sqrt(3./2.))
        self.add_line("i0", "q-axis current act0", self.a0_im*1e-3 * .537/np.sqrt(2.))
        self.add_line("v1", "q-axis voltage act1", -self.a1_vm*1e-3 * np.sqrt(3./2.))
        self.add_line("i1", "q-axis current act1", -self.a1_im*1e-3 * .537/np.sqrt(2.))
        self.add_line("phi_0", "motor-side motor angle for act0 in radians", ticks_to_motor_radians(self.a0_x) + self.initial_angle_offset)
        self.add_line("phid_0", "motor-side motor vel for act0 in radians/s", ticks_to_motor_radians(self.a0_xd)*1e3)
        self.add_line("phidd_0", "motor-side motor acc for act0 in radians/s/s", self.a0_xdd)

        self.add_line("i0_prime", "q-axis current act0 inferred from model", self.v0/R_phase - self.phid_0*K_emf/R_phase)
        self.add_line("v0_prime", "q-axis voltage act0 inferred from model", self.i0*R_phase + self.phid_0*K_emf)

        self.add_line("phi_1", "motor-side motor angle for act1 in radians", ticks_to_motor_radians(-self.a1_x) + self.initial_angle_offset)
        self.add_line("phid_1", "motor-side motor velocity for act1 in radians per second", ticks_to_motor_radians(-self.a1_xd)*1e3)
        self.add_line("phidd_1", "motor-side motor acceleration for act1 in radians per second squared", -self.a1_xdd)

        self.add_line("i1_prime", "q-axis current act1 inferred from model", self.v1/R_phase - self.phid_1*K_emf/R_phase)
        self.add_line("v1_prime", "q-axis voltage act1 inferred from model", self.i1*R_phase + self.phid_1*K_emf)

        self.add_line("theta_0", "spring-side angle (dev0), radians", self.phi_0)
        self.add_line("theta_1", "spring-side angle (dev1), radians", self.phi_1)
        self.add_line("delta_s", "spring-side deflection, radians", self.theta_0-self.theta_1)


        self.add_line("tau_m_1", "spring-side angle (dev1), radians", self.i1*K_tau)


def main(cal_folder,inner_mask,outer_mask,test_folder,defl_trq_file='/defl_torque.csv'):
    # Calculate camera_angs 
    if not os.path.exists(test_folder + '/camera_enabled_angles.csv'):

        blue_cam_angs, red_cam_angs, cam_time = test(file = test_folder + '/Stiffness_Measure_250326_164341.h264',
                                                    inner_mask_loc = inner_mask,
                                                    outer_mask_loc = outer_mask,
                                                    pre_mask_save_loc = test_folder + '/camera_enabled_pre_mask.png',
                                                    red_cal_save_loc = None,
                                                    blue_cal_save_loc = None)
        blue_cam_angs = blue_cam_angs - blue_cam_angs[0]
        red_cam_angs = red_cam_angs - red_cam_angs[0]
        cam_enabled_angs = np.vstack((cam_time,blue_cam_angs,red_cam_angs)).T

        with open(test_folder + '/camera_enabled_angles.csv', 'w') as f:
            np.savetxt(f, cam_enabled_angs, fmt='%.7f', delimiter=", ")

    else:
        cam_enabled_angs = np.loadtxt(test_folder + '/camera_enabled_angles.csv', delimiter=',')
        cam_time = cam_enabled_angs[:,0]
        blue_cam_angs = cam_enabled_angs[:,1]
        red_cam_angs = cam_enabled_angs[:,2]

    if False: # for viewing tests without camera data
        stp = SEATestbedPlotter(test_folder + 'camera_enabled_dev0.csv',
                            test_folder + 'camera_enabled_dev1.csv',
                            test_folder + 'camera_enabled_volts.csv')
        red_cam_angs = stp.theta_0
        blue_cam_angs = stp.theta_1
        cam_time = stp.a0_t

    if False:
        stp = SEATestbedPlotter(test_folder + 'camera_enabled_dev0.csv',
                        test_folder + 'camera_enabled_dev1.csv')
        red_enc_angs = stp.theta_0
        blue_enc_angs = stp.theta_1
        enc_time = stp.a0_t
        torque = red_enc_angs*0.0
        trq_time = stp.a0_t


    # Read encoder_angs from SEA_Testbed_Plotter
    stp = SEATestbedPlotter(test_folder + '/motor_enc_calib_0.csv',
                            test_folder + '/motor_enc_calib_0.csv',
                            # test_folder + '/camera_enabled_volts.csv'
                            )
    red_enc_angs = stp.theta_0
    blue_enc_angs = stp.theta_1
    enc_time = stp.a0_t
    torque = stp.tau
    trq_time = stp.a0_t

    # Align the timing based on angle peaks
    pks,_ = find_peaks(red_cam_angs,height=1*np.pi/180,distance=500)
    negpks,_ = find_peaks(-red_cam_angs,height=1*np.pi/180,distance=500)
    print(pks, negpks) # suspciciously, these lists are both empty.
    # negpks[0] = negpks[0] - 1
    red_cam_pks = np.sort(np.hstack((pks,negpks)))
    pks,_ = find_peaks(red_enc_angs,height=1*np.pi/180,distance=1500)
    negpks,_ = find_peaks(-red_enc_angs,height=1*np.pi/180,distance=1500)
    red_enc_pks = np.sort(np.hstack((pks,negpks)))

    red_cam_det = np.gradient(red_cam_angs,cam_time)
    red_enc_det = np.gradient(red_enc_angs,enc_time)
    threshold = 0.0*np.pi/180
    thresh_cross_cam = np.diff(red_cam_det > threshold, prepend=False)
    thresh_cross_enc = np.diff(red_enc_det > threshold, prepend=False)
    cross_ind_cam = np.argwhere(thresh_cross_cam)[:,0]
    grad_cross_cam = np.gradient(cross_ind_cam)
    cam_ends_cross = np.diff(grad_cross_cam > 10*np.mean(grad_cross_cam[0:10]), prepend=False)
    cam_start_ind = cross_ind_cam[np.argwhere(cam_ends_cross)[0,0]] # this is giving me problems. 
    cam_end_ind = cross_ind_cam[np.argwhere(cam_ends_cross)[-1,0]-1]

    cross_ind_enc = np.argwhere(thresh_cross_enc)[:,0]
    grad_cross_enc = np.gradient(cross_ind_enc)
    enc_ends_cross = np.diff(grad_cross_enc > 10*np.mean(grad_cross_enc[0:10]), prepend=False)
    enc_start_ind = cross_ind_enc[np.argwhere(enc_ends_cross)[0,0]]
    enc_end_ind = cross_ind_enc[np.argwhere(enc_ends_cross)[-1,0]-1]

    red_cam_pks = np.hstack((cam_start_ind,red_cam_pks,cam_end_ind))
    red_enc_pks = np.hstack((enc_start_ind,red_enc_pks,enc_end_ind))
    plt.plot(cam_time,blue_cam_angs)
    plt.plot(cam_time,red_cam_angs)
    plt.plot(enc_time, blue_enc_angs)
    plt.plot(enc_time, red_enc_angs)
    plt.show()
    # print(cam_time[red_cam_pks])
    # print(enc_time[red_enc_pks])

    # red_cam_pks = np.hstack((cam_start_ind,red_cam_pks,cam_end_ind))
    # red_enc_pks = np.hstack((enc_start_ind,red_enc_pks,enc_end_ind))


    # t_diff = enc_time[red_enc_pks] - cam_time[red_cam_pks]
    # if np.max(t_diff) > 0.2:
    #     print('WARNING: AUTOMATIC TIME ADJUSTMENT SHIFTED BY %.3f SECONDS. PLEASE CHECK THAT IT IS WORKING PROPERLY.'%np.max(t_diff))
    #     print('Time shifts: ',t_diff)

    # new_cam_time = cam_time[0:red_cam_pks[0]]
    # for i in range(len(red_cam_pks)-1):
    #     new_seg = np.linspace(enc_time[red_enc_pks[i]],enc_time[red_enc_pks[i+1]],red_cam_pks[i+1]-red_cam_pks[i]+1)
    #     new_cam_time = np.hstack((new_cam_time,new_seg[0:-1]))

    # end_seg = cam_time[red_cam_pks[-1]:] + t_diff[-1]
    # new_cam_time = np.hstack((new_cam_time,end_seg))
    # cam_time = new_cam_time


    # Second calibration, interpolate cam_angs to enc_angs
    inv_blue = np.loadtxt(cal_folder + '/inv_blue.csv', delimiter=',')
    inv_red = np.loadtxt(cal_folder + '/inv_red.csv', delimiter=',')
    inv_blue = np.flip(inv_blue, axis=0)
    inv_red = np.flip(inv_red, axis=0)

    blue_cam_ang_cal = np.interp(blue_cam_angs,inv_blue[:,0],inv_blue[:,1])
    red_cam_ang_cal = np.interp(red_cam_angs,inv_red[:,0],inv_red[:,1])

    red_cam_ang_cal_res = np.interp(enc_time[red_enc_pks[0]:red_enc_pks[-1]], 
                                    cam_time[red_cam_pks[0]:red_cam_pks[-1]], 
                                    red_cam_ang_cal[red_cam_pks[0]:red_cam_pks[-1]])
    blue_cam_ang_cal_res = np.interp(enc_time[red_enc_pks[0]:red_enc_pks[-1]], 
                                     cam_time[red_cam_pks[0]:red_cam_pks[-1]], 
                                     blue_cam_ang_cal[red_cam_pks[0]:red_cam_pks[-1]])
    
    red_enc_ang_res = np.interp(cam_time[red_cam_pks[0]:red_cam_pks[-1]], 
                                enc_time[red_enc_pks[0]:red_enc_pks[-1]], 
                                red_enc_angs[red_enc_pks[0]:red_enc_pks[-1]])
    blue_enc_ang_res = np.interp(cam_time[red_cam_pks[0]:red_cam_pks[-1]], 
                                 enc_time[red_enc_pks[0]:red_enc_pks[-1]], 
                                 blue_enc_angs[red_enc_pks[0]:red_enc_pks[-1]])
    # Validation thing
    # red_cam_rng_res = np.interp(enc_time,cam_time,red_cam_angs)
    # plt.figure(5)
    # plt.plot(enc_time[red_enc_pks[0]:red_enc_pks[-1]], 
    #          red_cam_ang_cal_res - red_enc_angs[red_enc_pks[0]:red_enc_pks[-1]], 'r')
    # plt.plot(enc_time[red_enc_pks[0]:red_enc_pks[-1]], 
    #          blue_cam_ang_cal_res - blue_enc_angs[red_enc_pks[0]:red_enc_pks[-1]], 'b')
    # p = np.polyfit(red_cam_rng_res,red_enc_angs,1)
    # print("Fit: ",p)
    
    plt.figure(6)
    plt.plot(cam_time[red_cam_pks[0]:red_cam_pks[-1]], 
             red_cam_ang_cal[red_cam_pks[0]:red_cam_pks[-1]] - red_enc_ang_res, 'r')
    plt.plot(cam_time[red_cam_pks[0]:red_cam_pks[-1]],
             blue_cam_ang_cal[red_cam_pks[0]:red_cam_pks[-1]] - blue_enc_ang_res, 'b')
    
    plt.figure(1)
    plt.plot(cam_time,red_cam_angs*180/np.pi,'r')
    plt.plot(cam_time,blue_cam_angs*180/np.pi,'b')

    plt.plot(enc_time,blue_enc_angs*180/np.pi,'c')
    plt.plot(enc_time,red_enc_angs*180/np.pi,'m')

    plt.plot(cam_time,red_cam_ang_cal*180/np.pi,'g')
    plt.plot(cam_time,blue_cam_ang_cal*180/np.pi,'k')

    plt.plot(cam_time,red_cam_det)
    plt.plot(enc_time,red_enc_det)

    # plt.plot(trq_time,-torque)    

    defl = (- blue_cam_ang_cal + red_cam_ang_cal)*180/np.pi
    torque_res = np.interp(cam_time, trq_time, torque)
    enc_defl = (blue_enc_angs - red_enc_angs)*180/np.pi

    defl_torque = np.vstack((defl,torque_res)).T
    # defl_torque = np.vstack((enc_defl,torque)).T

    # plt.figure(2)
    # plt.plot(cam_time,defl)
    # plt.plot(enc_time,enc_defl)

    plt.figure(2)
    plt.plot(enc_defl,torque)
    plt.plot(defl,torque_res)
    plt.legend(['Motor Encoder Measurement','Optical Measurement'])
    plt.xlabel('Deflection (deg)')
    plt.ylabel('Torque (Nm)')

    with open(test_folder + defl_trq_file, 'w') as f:
        np.savetxt(f, defl_torque, fmt='%.7f', delimiter=", ")



if __name__ == '__main__':

    # main(cal_folder='data/08_30_22_T13/',
    #         inner_mask = 'inner_mask0830.png',
    #         outer_mask = 'outer_mask0830.png',
    #         test_folder='data/09_02_22_T1/',
    #         defl_trq_file='S4_mencoder.csv')

    # main(cal_folder='data/08_30_22_T13/',
    #         inner_mask = 'inner_mask0830.png',
    #         outer_mask = 'outer_mask0830.png',
    #         test_folder='data/09_02_22_T3/',
    #         defl_trq_file='S4_mencoder.csv')

    # main(cal_folder='data/08_30_22_T13/',
    #         inner_mask = 'inner_mask0830.png',
    #         outer_mask = 'outer_mask0830.png',
    #         test_folder='data/09_02_22_T4/',
    #         defl_trq_file='S4_mencoder.csv')
    # plt.legend(['No Lubricant','Graphite Lubricant','Graphite SLOW'])


    # main(cal_folder='data/08_16_22_T1/',
    #         inner_mask = 'inner_mask5.png',
    #         outer_mask = 'outer_mask3.png',
    #         test_folder='data/08_16_22_T4/',
    #         defl_trq_file='S1_14_19.csv')    

    main(cal_folder='./cal_folder',
            inner_mask = 'inner_mask_0326.png',
            outer_mask = 'outer_mask_0326.png',
            test_folder='./meas_folder',
        )
    plt.show()
"""
Bibliography
walking_knee_78kg.mat
    [LeePanRouse2019IROS] Ung Hee Lee, Chen-Wen Pan, and Elliott J. Rouse "Empirical Characterization
        of a High-performance Exterior-rotor Type Brushless DC Motor and Drive" 2019 IEEE/RSJ
        International Conference on Intelligent Robots and Systems (IROS). pp. 8012 -- 8019, Macau,
        China, November 4-8, 2019.
"""
