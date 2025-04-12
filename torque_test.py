from opensourceleg.osl import OpenSourceLeg
import numpy as np
from time import strftime, time
from hardware.futek import Big100NmFutek
from hardware.encoder import AS5048A_Encoder
from hardware.encoder_correction import Spring_Model, calc_velocity_timescale, nonlinear_compensation, Median_Filter, Basic_LPF, backlash_centering_by_hand, TorqueSensorThread
from control.pid_zach import PID
from picamera2 import Picamera2, Preview
from picamera2.encoders import H264Encoder
import time


GR_ACTPACK = 1
GR_TRANS = 75/11
GR_BOSTONGEAR = 50

WAIT = 5

if __name__ == "__main__":

    osl = OpenSourceLeg(frequency=500, file_name='Torque_Sensor_Test'+strftime("%y%m%d_%H%M%S")) # 200 Hz
    # osl.add_joint(name="knee", port = None, gear_ratio=GR_ACTPACK)
    osl.add_joint(name="knee", port = "/dev/ttyACM0", gear_ratio=GR_BOSTONGEAR*GR_ACTPACK)
    # osl.add_joint(name="ankle", port = None, gear_ratio=GR_ACTPACK*GR_BOSTONGEAR)

    torqueSensor = Big100NmFutek()
    torque_sensor_thread = TorqueSensorThread(torqueSensor, update_interval=1.0/osl._frequency)
    
    
    # picam2 = Picamera2()
    # picam2.configure(
    #         picam2.create_video_configuration(
    #             raw={"size":(1640,1232)}, # raw size 
    #             main={"size": (640, 480)} # scaled size
    #             )
    #         )
    # picam2.set_controls({"FrameRate": 30})
    # encoder = H264Encoder()
    # picam2.start_preview(Preview.DRM)
    # Initialize joint encoder:
    # knee_enc = AS5048A_Encoder(
    #         name="output",
    #         basepath="/",
    #         bus="/dev/i2c-1",
    #         A1_adr_pin=False,
    #         A2_adr_pin=True,
    #         zero_position=0,
    #         )
    # knee_enc._start()
    # knee_enc._update()
    
    # Configure Logging
    
    osl.log.add_attributes(osl, ["timestamp", "_frequency"])
    log_info = ["output_position", "output_velocity", "accelx", 
                "motor_voltage", "motor_current", "battery_voltage", 
                "battery_current"]
    osl.log.add_attributes(osl.knee, log_info)
    # osl.log.add_attributes(osl.ankle, log_info)
    osl.log.add_attributes(locals(), ["tau_futek"])
    tau_futek = 0
    # osl.log.add_attributes(locals(), ["tau_meas","joint_ang_pre","output_ang_pre","ang_err","ang_err_filt","deflection","ang_err_pre"])
    # picam2.start_recording(encoder, 'Stiffness_Measure_'+strftime("%y%m%d_%H%M%S")+'.h264')
    # nonlinear_compensation(osl,knee_enc,Calibrate=False)
    with osl:
        # Calculate Offsets for Futek and Angles (to center in backlash)
        torqueSensor.calibrate_loadcell(Calibrate=True) # recalibrates Futek or looks up previous calibration value
        # offset,joint_ang_init = backlash_centering_by_hand(osl,knee_enc,Calculate=True) # calculates angle offset to center in backlash, or looks up previous offset
        input("Hit enter to continue...")

        # osl.knee.set_mode(osl.knee.control_modes.current)
        osl.knee.set_mode(osl.knee.control_modes.position)
        # osl.knee.set_current_gains(40,400,110) # Kp, Ki, Kff
        
        osl.knee.set_position_gains(
            kp = 300, 
            ki = 150, 
            kd = 100, 
            ff = 100
        )
        
        osl.update()
        init_pos = osl.knee.output_position
        
        # Initialize filters
        f_c = 50 # Cutoff frequency (Hz)
        window = 3 # Median Filter Window
        
        ang_err_med_filter = Median_Filter(0,window)
        ang_err_lp_filter = Basic_LPF(0,osl._frequency,f_c)
        
        # Output SEA Config
        spring_type = "Output SEA"

        # # Intermediate SEA Config
        # spring_type = "Intermediate SEA"
        
        # Load spring parameters
        spring = Spring_Model(spring_type)

        # Velocity compensation
        velocity_time_scalar = calc_velocity_timescale(osl._frequency) # coefficients calculated in measurement_analysis.m

        # Set up test
        # ank_output_0 = osl.ankle.output_position
        
        # i_des = np.linspace(1000, 3000, 4)
        pos_des = np.array([10/180*np.pi, 
                            15/180*np.pi,
                            20/180*np.pi,
                            25/180*np.pi,])
        t_test = 7

        print('Test in Progress:')
        torque_sensor_thread.start()
        
        for t in osl.clock:
            tau_futek = torque_sensor_thread.get_latest_torque()
            osl.update()
            if t > WAIT:
                break
        # torque_sensor_thread.start()
        # # for i in i_des:
        # for p in pos_des:
        #     for t in osl.clock:
        #         if t < t_test:
        #             # i_command = i/t_test*t
        #             pos_command = p / t_test * t
        #         elif t < 3*t_test:
        #             # i_command = -i/t_test*t + 2*i
        #             pos_command = -p/t_test*t + 2*p
        #         elif t < 4*t_test:
        #             # i_command = i/t_test*t - 4*i
        #             pos_command = p/t_test*t - 4*p
        #         else:
        #             break
                
        #         tau_futek = torque_sensor_thread.get_latest_torque()
        #         # knee_enc._update()
        #         osl.update()
                
        #         # joint_ang_pre = knee_enc.abs_comp_ang
        #         # output_ang_pre = osl.knee.output_position + offset
        #         # ang_err_pre = output_ang_pre - joint_ang_pre
                
        #         # output_ang_adjusted = output_ang_pre + velocity_time_scalar*osl.knee.output_velocity
        #         # ang_err = output_ang_adjusted - joint_ang_pre
        #         # ang_err_filt = ang_err_lp_filter.update(ang_err_med_filter.update(ang_err))
        #         # deflection = spring.backlash_comp_smooth(ang_err_filt)    
        #         # tau_meas = deflection*spring.K*GR_ACTPACK*GR_TRANS/osl.knee.gear_ratio
                
        #         print(osl.knee.motor_current)
        #         # SAFETY CHECKS
        #         if osl.knee.winding_temperature > 100:
        #             raise ValueError("Motor above thermal limit. Quitting!")            
        #         # Check battery voltages
        #         if osl.knee.battery_voltage/1000 > 43:
        #             print("Knee voltage {}".format(1/1000*osl.knee.battery_voltage))
        #             raise ValueError("Battery voltage above 43 V")
        #         if osl.knee.battery_voltage/1000 < 20:
        #             print("Knee voltage {}".format(1/1000*osl.knee.battery_voltage))
        #             raise ValueError("Battery voltage below 32 V")
        #         if np.abs(tau_futek) > 90:
        #             print("Torque too high: {} Nm".format(tau_futek))
        #             break
        #         if osl.knee.motor_current/1000 > 8:
        #             print("Knee Current {}".format(1/1000*osl.knee.motor_current))
        #             raise ValueError("Motor Current above 6 A")
                
        #         osl.knee.set_output_position(init_pos + pos_command)
                
            
        osl.clock.stop()
        # knee_enc._stop()
        torque_sensor_thread.stop()
        torque_sensor_thread.join()
        time.sleep(3)
        # picam2.stop_preview()
        # picam2.stop_recording()
        print("Test complete :)")