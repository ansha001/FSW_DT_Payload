""" State variable initializer
Create the files that contain the backups of battery channel data and experiment parameters
AZ, 2025 April 21
"""

import json
import os
import numpy as np
    
if __name__ == "__main__":
    param_file   = os.getcwd() + '/PARAMS_LIST.json'
    fsw_file     = os.getcwd() + '/FSW_BACKUP.json'
    ch0_file     = os.getcwd() + '/CH0_BACKUP.json'
    ch1_file     = os.getcwd() + '/CH1_BACKUP.json'
    ch2_file     = os.getcwd() + '/CH2_BACKUP.json'
    ekf_cap_file = os.getcwd() + '/EKF_BACKUP.npy' 
    cyc_cap_file = os.getcwd() + '/CYC_BACKUP.npy'
    beta_cap_file= os.getcwd() + '/BETA_BACKUP.npy'
    
    NUM_EKF_CAP = 600
    NUM_CYC_CAP = 360
    NUM_BETA_CAP = 75
    beta_window = 3e5
    beta_prediction_rate = beta_window / NUM_BETA_CAP
    cap0_as = 44.0728 * 3.6
    cap1_as = 44.3632 * 3.6
    cap2_as = 43.9598 * 3.6
    cap3_as = 44.4840 * 3.6
    HEADER = '\x30\x20\x30\x20\x30\x20\x30\x20'
    
    param_vals = {"DT_CHECK_S" : 0.2,          # time step between important checks
        "DT_SENSORS_S" : 0.1,         # time step between reading sensors
        "DT_LOG_S"  : 1.0,            # time step between logs, 1 s all times except during pulse (10 Hz)
        "DT_LOG2_S" : 180,             # time step between type 2 logs, normally 10 minutes
        "DT_LOG3_S" : 900,            # time step between type 3 logs, normally 15 minutes
        "DT_HEAT_S" : 90,            # time step between checking heater
        "DT_FAST_S" : 1,              # time step between fast loop of EKF
        "DT_SLOW_S" : 1000,           # time step between slow loop of EKF - this isn't used anymore, "slow_loop_rate" is now used
        "DT_BACKUP_S" : 60,           # time step between backups
        "CHG_LIMIT_V" : 4.20,         # voltage to stop charging
        "DIS_LIMIT_V" : 2.75,         # voltage to stop discharging
        "FB_PULSE1_V" : 3.10,         # fallback voltage for 65% SOC
        "FB_PULSE2_V" : 3.00,         # fallback voltage for 30% SOC
        "SOC_PULSE1" : 0.65,          # SOC to perform pulse 1
        "SOC_PULSE2" : 0.30,          # SOC to perform pulse 2
        "TIME_CYCLE_REST_S" : 300,    # time to rest between charge/discharge cycles, seconds  [NOMINALLY  5 MINUTES]
        "TIME_PULSE_REST_S" : 600,    # time to rest after pulse tests, seconds                [NOMINALLY 10 MINUTES]
        "TIME_PULSE_TEST_S" :  10,    # time to hold current during pulse tests, seconds       [NOMINALLY 10 SECONDS]
        "TIME_TEST_REST_S" : 3600,    # time to rest before and after char test, seconds       [NOMINALLY  1 HOUR]
        "TEMP_MAX_C" :  65,           # above this, go to safe mode
        "TEMP_MIN_C" : -10,           # below this, go to safe mode
        "TEMP_HEATER_ON_C"  : 22,     # below this, turn heater on
        "TEMP_HEATER_OFF_C" : 30,     # above this, turn heater off
        "TEMP_CHG_MAX_C"   : 60,      # above this, stop charging
        "TEMP_CHG_UPPER_C" : 19,      # above this, charge setpoint is -45 mA
        "TEMP_CHG_LOWER_C" : 10,       # above this, charge setpoint is -27 mA
        "TEMP_CHG_MIN_C"   : 0,       # below this, stop charging
        
        "CHG_UPPER_SETPT_MA" : -45,   # nominal setpoint
        "CHG_LOWER_SETPT_MA" : -27,   # cold setpoint
        "CHG_MIN_SETPT_MA"   : -10,    # minimum setpoint
        
        "CHG_SETPT_DELTA_MA" : 0.5,     # if charging current is outside bound, increment potentiometer
        "CHG_LOW_SETPT_MA" : -10,       # setpoint for 1/10 C charging
        "CHG_LOW_SETPT_DELTA_MA" : 0.3, #bound for when in low charge mode
        "DIS_SETPT_MA" : 45,            # discharge setpoint
        "DIS_SETPT_DELTA_MA" : 0.5,
        "DIS_LOW_SETPT_MA" : 4.5,       # setpoint for 1/10 C discharging
        "DIS_LOW_SETPT_DELTA_MA" : 0.3, #bound for when in low discharge mode
        
        "DIS_TRANS_MA" : 4.9,           #current to transition to low current mode
        "CHG_VAL_INIT" : 9,
        "DIS_VAL_INIT" : 120,
        "NUM_CYCLES_PER_TEST" : 20,     # typically 20
        "num_boots" : 0,
        "ALPHA_EKF" : 0.04,
        "ALPHA_CYC" : 0.02,
        "NUM_EKF_CAP" : NUM_EKF_CAP,
        "NUM_CYC_CAP" : NUM_CYC_CAP,
        "NUM_BETA_CAP" : NUM_BETA_CAP,
                  
        "R_state"  : 0.015,                     # Measurement noise covariance
        "Q_state1" : 1e-6,     # Process noise covariance
        "Q_state2" :    1,     # Process noise covariance
        "Q_param"  : 1e-1,                      # Parameter process noise
        "R_param"  : 4.1e-4,                # Parameter measurement noise
        "slow_loop_rate" : 2100,
        "beta_window" : beta_window,
        "beta_prediction_rate" : beta_prediction_rate,
        "forecast_horizon" : 6e5,
        "HEADER" : HEADER, }

    json_str = json.dumps(param_vals) #convert to json str
    
    with open(param_file, 'w') as json_out:
        json_out.write(json_str)
        
    with open(param_file, 'r') as json_in:  #for bugging, read what we just wrote
        parsed = json.loads(json_in.read())
        print(json.dumps(parsed, indent=4, sort_keys=False))
        
    ch_vals = {"state" : "DIS",
        "state_prev" : "DIS",
        "mode" : "CYCLE",
        "test_sequence" : 0,
        "total_cycles" : 0,
        "total_rpts" : 0,
        "volt_v" : 0,     
        "temp_c" : 20,
        "cycle_count" : 0,
        "chg_val" : 9,
        "dis_val" : 120,
        "chg_low_val" : 200,
        "dis_low_val" : 120,
        "pulse_state" : False,
        "en_chg_state" : False,
        "en_dis_state" : False,
        "en_cur_state" : False,
        "update_act" : False,
        "cc_capacity_mas" : 0,
        "cc_soc_mas" : 0,
        "last_cyc_cap_mah" : -1,
        "R_SHUNT_OHMS" : 1.0,
        "est_capacity_as" : cap0_as,   # Initial capacity in As
        "est_soc" : 0,
        "est_cov_state00" : 1e-7, #np.zeros((2, 2)),  # 2x2 covariance matrix for state EKF
        "est_cov_state01" : 0,
        "est_cov_state10" : 0,
        "est_cov_state11" : 1e-7,
        "est_cov_param" : 1e-1,
        "update_counter" : 0,
        "update_counter_beta" : 0,
        "pred_cyc_mah" : 0,
        "pred_ekf_mah" : 0,
        "pred_beta_mah" : 0, }

    json_str = json.dumps(ch_vals) #convert to json str
    with open(ch0_file, 'w') as json_out:
        json_out.write(json_str)
        
    #set values unique to ch1
    ch_vals["est_capacity_as"] = cap1_as
    json_str = json.dumps(ch_vals) #convert to json str
    with open(ch1_file, 'w') as json_out:
        json_out.write(json_str)
    
    #set values unique to ch2
    ch_vals["est_capacity_as"] = cap2_as
    json_str = json.dumps(ch_vals) #convert to json str
    with open(ch2_file, 'w') as json_out:
        json_out.write(json_str)
        
    #for debugging, read what we just wrote
    with open(ch0_file, 'r') as json_in:  
        parsed = json.loads(json_in.read())
        print(json.dumps(parsed, indent=4, sort_keys=False))
        
    # make np array files for capacity estimate storage
    ekf_cap_est_mah = -1*np.ones([NUM_EKF_CAP,3])
    cyc_cap_est_mah = -1*np.ones([NUM_CYC_CAP,3])
    beta_cap_est_as =  1*np.ones([NUM_BETA_CAP,3])
    beta_cap_est_as[:,0] = cap0_as * beta_cap_est_as[:,0]
    beta_cap_est_as[:,1] = cap1_as * beta_cap_est_as[:,1]
    beta_cap_est_as[:,2] = cap2_as * beta_cap_est_as[:,2]
    np.save(ekf_cap_file, ekf_cap_est_mah)
    np.save(cyc_cap_file, cyc_cap_est_mah)
    np.save(beta_cap_file, beta_cap_est_as)
    
    #for debugging, read what we just wrote
    ekf_cap_est_mah = np.load(ekf_cap_file)
    cyc_cap_est_mah = np.load(cyc_cap_file)
    print(ekf_cap_est_mah.size)
    print(ekf_cap_est_mah)
    print(cyc_cap_est_mah.size)
    print(cyc_cap_est_mah)

    # initialize buffer backups for groups 1, 2, 3
    LOG_BASE_DIR = os.getcwd() + '/log'
    os.makedirs(LOG_BASE_DIR, exist_ok=True)

    buffers_backup = {
        "1": [],
        "2": [],
        "3": []
    }

    with open(os.path.join(LOG_BASE_DIR, 'buffers_backup.json'), 'w') as f:
        json.dump(buffers_backup, f)
    print("[INFO] initialized buffer backup to empty lists")

    # initialize pointers for each group
    for group_id in [1, 2, 3]:
        group_folder = os.path.join(LOG_BASE_DIR, f'group{group_id}')
        os.makedirs(group_folder, exist_ok=True)
        pointer_path = os.path.join(group_folder, '.last_sent_pointer')
        with open(pointer_path, 'w') as f:
            f.write('0')
        print(f"[INFO] initialized pointer for group {group_id} to 0.")

    # initialize global pointer
    global_pointer_path = os.path.join(LOG_BASE_DIR, '.last_sent_group')
    with open(global_pointer_path, 'w') as f:
        f.write('0')
    print("[INFO] initialized global pointer to 0")
