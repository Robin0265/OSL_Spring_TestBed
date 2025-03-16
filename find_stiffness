from opensourceleg.osl import OpenSourceLeg
import numpy as np
from time import strftime, time
from hardware.futek import Big100NmFutek
from hardware.encoder import AS5048A_Encoder
from hardware.encoder_correction import Spring_Model, calc_velocity_timescale, nonlinear_compensation, Median_Filter, Basic_LPF, backlash_centering_by_hand, TorqueSensorThread
from control.pid_zach import PID


GR_ACTPACK = 9
GR_TRANS = 75/11
GR_BOSTONGEAR = 50

if __name__ == "__main__":

    osl = OpenSourceLeg(frequency=500, file_name='Stiffness_Measure_'+strftime("%y%m%d_%H%M%S")) # 200 Hz
    # osl.add_joint(name="knee", port = None, gear_ratio=GR_ACTPACK)
    osl.add_joint(name="knee", port = None, gear_ratio=GR_ACTPACK*GR_TRANS)
    # osl.add_joint(name="ankle", port = None, gear_ratio=GR_ACTPACK*GR_BOSTONGEAR)

    torqueSensor = Big100NmFutek()
    torque_sensor_thread = TorqueSensorThread(torqueSensor, update_interval=1.0/osl._frequency)

    # Initialize joint encoder:
    knee_enc = AS5048A_Encoder(
            name="output",
            basepath="/",
            bus="/dev/i2c-1",
            A1_adr_pin=False,
            A2_adr_pin=True,
            zero_position=0,
            )
    knee_enc._start()
    knee_enc._update()
    
    # Configure Logging
    actpack_vars_2_log = ["output_position", "output_velocity", "motor_position", "motor_velocity",
                            "winding_temperature", "case_temperature", "motor_current", 
                            "motor_voltage","battery_voltage","battery_current",
                            "joint_position", "joint_velocity"]
    osl.log.add_attributes(osl, ["timestamp"])
    osl.log.add_attributes(osl.knee, actpack_vars_2_log)
    osl.log.add_attributes(torqueSensor, ["torque"])
    osl.log.add_attributes(locals(), ["tau_meas","joint_ang_pre","output_ang_pre","ang_err","ang_err_filt","deflection","ang_err_pre"])

    nonlinear_compensation(osl,knee_enc,Calibrate=False)

    with osl:
        osl.update(log_data=False)   
        # Calculate Offsets for Futek and Angles (to center in backlash)
        torqueSensor.calibrate_loadcell(Calibrate=True) # recalibrates Futek or looks up previous calibration value
        offset,joint_ang_init = backlash_centering_by_hand(osl,knee_enc,Calculate=True) # calculates angle offset to center in backlash, or looks up previous offset
        input("Hit enter to continue...")

        osl.knee.set_mode(osl.knee.control_modes.current)
        osl.knee.set_current_gains(40,400,110) # Kp, Ki, Kff
        osl.update(log_data=False)           
        
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
        
        i_des = np.linspace(2000, 8000, 4)
        t_test = 4

        print('Test in Progress:')
        torque_sensor_thread.start()
        for i in i_des:
            for t in osl.clock:
                if t < t_test:
                    i_command = i/t_test*t
                elif t < 3*t_test:
                    i_command = -i/t_test*t + 2*i
                elif t < 4*t_test:
                    i_command = i/t_test*t - 4*i
                else:                
                    break
                
                tau_futek = torque_sensor_thread.get_latest_torque()
                knee_enc._update()
                osl.update(log_data=False)
                
                joint_ang_pre = knee_enc.abs_comp_ang
                output_ang_pre = osl.knee.output_position + offset
                ang_err_pre = output_ang_pre - joint_ang_pre
                
                output_ang_adjusted = output_ang_pre + velocity_time_scalar*osl.knee.output_velocity
                ang_err = output_ang_adjusted - joint_ang_pre
                ang_err_filt = ang_err_lp_filter.update(ang_err_med_filter.update(ang_err))
                deflection = spring.backlash_comp_smooth(ang_err_filt)    
                tau_meas = deflection*spring.K*GR_ACTPACK*GR_TRANS/osl.knee.gear_ratio
                
                # SAFETY CHECKS
                if osl.knee.winding_temperature > 100:
                    raise ValueError("Motor above thermal limit. Quitting!")            
                # Check battery voltages
                if osl.knee.battery_voltage/1000 > 43:
                    print("Knee voltage {}".format(1/1000*osl.knee.battery_voltage))
                    raise ValueError("Battery voltage above 43 V")
                if osl.knee.battery_voltage/1000 < 20:
                    print("Knee voltage {}".format(1/1000*osl.knee.battery_voltage))
                    raise ValueError("Battery voltage below 32 V")
                if np.abs(tau_futek) > 90:
                    print("Torque too high: {} Nm".format(tau_futek))
                    break
                
                osl.knee.set_current(i_command)
                osl.log.data()
            
        osl.clock.stop()
        knee_enc._stop()
        torque_sensor_thread.stop()
        torque_sensor_thread.join()    
        print("Test complete :)")