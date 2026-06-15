from collections import deque
import serial
import time
import csv
import sys
import os

import interface_ascii as psu  

# ==========================================
# CONFIGURATION
# ==========================================
ESP32_PORT = 'COM4'       # Ensure this matches your ESP32
BAUD_RATE = 115200

FAULT = -999.0
WAIT_TIME_SECONDS = 180   # Total transient time (3 Minutes)
MILESTONES = [30, 60, 90, 120, 150] # Seconds at which to capture intermediate states

def is_valid(*vals):
    return all(v != FAULT for v in vals)

def is_fan_safe(target_rpm, actual_rpm):
    """Checks if the actual fan RPM is within 5% of the target RPM."""
    if actual_rpm < 0: 
        return False 
    if target_rpm == 0:
        return actual_rpm < 50
    
    error_percent = (abs(target_rpm - actual_rpm) / target_rpm) * 100.0
    return error_percent <= 5.0

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_FILENAME = os.path.join(SCRIPT_DIR, 'peltier_mlr_transient_curve_data.csv')

def thermal_flush(esp32):
    """
    Blasts the fan, cuts heat, and waits for T_hot to reach ambient baseline.
    """
    print("\n[FLUSH] Initiating Thermal Flush with Block Averaging...")
    psu.psu_SetCurrent(0.0)
    time.sleep(0.2)      
    psu.psu_OutputOFF()  
    time.sleep(0.2)        
    
    esp32.write(b"4000\n")          
    esp32.reset_input_buffer()
    
    WINDOW_SIZE = 40
    HALF_WINDOW = WINDOW_SIZE // 2
    
    hot_history = deque(maxlen=WINDOW_SIZE)
    amb_history = deque(maxlen=WINDOW_SIZE)
    
    DRIFT_TOLERANCE = 0.15     
    BASELINE_TOLERANCE = 3.50  
    MAX_FLUSH_TIME_S = 300    
    
    start_time = time.time()
    
    while True:
        elapsed_time = time.time() - start_time
        if elapsed_time > MAX_FLUSH_TIME_S:
            print(f"\n[FLUSH] Timeout reached ({MAX_FLUSH_TIME_S}s). Forcing next trial to prevent hanging.")
            break

        raw_data = esp32.readline()
        if not raw_data:
            continue

        line = raw_data.decode('utf-8', errors='ignore').strip()
        if line:
            data_list = line.split(',')
            if len(data_list) == 6:
                try:
                    t_hot = float(data_list[4])   
                    t_amb = float(data_list[3])   
                    
                    if not is_valid(t_hot, t_amb):
                        continue
                        
                    hot_history.append(t_hot)
                    amb_history.append(t_amb)
                    
                    if len(hot_history) == WINDOW_SIZE:
                        hot_list = list(hot_history)
                        amb_list = list(amb_history)
                        
                        old_hot_avg = sum(hot_list[:HALF_WINDOW]) / HALF_WINDOW
                        new_hot_avg = sum(hot_list[HALF_WINDOW:]) / HALF_WINDOW
                        new_amb_avg = sum(amb_list[HALF_WINDOW:]) / HALF_WINDOW
                        
                        drift = abs(old_hot_avg - new_hot_avg)
                        diff_to_ambient = abs(new_hot_avg - new_amb_avg)
                        
                        print(f"Cooling [{int(elapsed_time)}s]... Avg Thot: {new_hot_avg:.2f}°C | Drift: {drift:.2f} | ΔTamb: {diff_to_ambient:.2f}    ", end='\r')
                        
                        if drift <= DRIFT_TOLERANCE and diff_to_ambient <= BASELINE_TOLERANCE:
                            print("\n[FLUSH] Averaged thermal baseline detected! Ready for next trial.")
                            break 
                    else:
                        print(f"Collecting baseline samples... Buffer: {len(hot_history)}/{WINDOW_SIZE}", end='\r')
                        
                except ValueError:
                    pass

def get_valid_reading(esp32):
    """Helper function to grab one clean, valid reading from the ESP32.
       NOTE: Buffer flushing is now handled by the caller for tighter timing control.
    """
    while True:
        raw_data = esp32.readline()
        if not raw_data:
            continue
        line = raw_data.decode('utf-8', errors='ignore').strip()
        if line:
            data_list = line.split(',')
            if len(data_list) == 6:
                try:
                    floats = [float(x) for x in data_list]
                    if is_valid(*floats):
                        return floats  
                except ValueError:
                    pass

def main():
    print("Connecting to Hardware...")
    
    psu.psu_Connect()
    psu.psu_OutputOFF()
    psu.psu_SetVoltage(15.4)
    
    try:
        esp32 = serial.Serial(port=ESP32_PORT, baudrate=BAUD_RATE, timeout=2)
        esp32.setDTR(True)  
        time.sleep(2)       
    except Exception as e:
        print(f"Failed to connect to ESP32 on {ESP32_PORT}: {e}")
        psu.psu_Disconnect()
        sys.exit(1)

    print("Hardware Connected. Ensuring safe start state...")
    thermal_flush(esp32)

    write_header = not os.path.exists(CSV_FILENAME) or os.path.getsize(CSV_FILENAME) == 0

    with open(CSV_FILENAME, mode='a', newline='') as file:
        writer = csv.writer(file)
        
        if write_header:
            header = [
                'Trial', 'Timestamp', 'Target_Current', 'Target_RPM', 
                'Init_Actual_Current', 'Init_Actual_RPM', 'Init_PWM_Duty',
                'Init_Tamb', 'Init_Thot', 'Init_Tcold'
            ]
            for m in MILESTONES:
                header.extend([f'Amps_{m}s', f'Tamb_{m}s', f'Thot_{m}s', f'Tcold_{m}s'])
            
            header.extend(['Final_Amps', 'Final_Tamb', 'Final_Thot', 'Final_Tcold'])
            
            writer.writerow(header)
            file.flush()
        
        test_fan_rpms = [1400, 1800, 2600, 2800, 3400, 3600, 4000] #list(range(4000, 1399, -200)) + list(range(1600, 4001, 200))
        test_currents = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0]
        
        cycle_count = 1
        trial_number = 1
        
        while True:
            print(f"\n=======================================================")
            print(f"   STARTING TRANSIENT DATA COLLECTION CYCLE #{cycle_count}")
            print(f"=======================================================\n")
            
            for target_rpm in test_fan_rpms:
                for amps in test_currents:
                    
                    print(f"\n--- TRIAL {trial_number} ---")
                    
                    # 1. Spool up the fan and verify RPM is stable
                    fan_is_ready = False
                    while not fan_is_ready:
                        print(f"Setting fan to {target_rpm} RPM. Waiting 5s for PID to stabilize...")
                        esp32.write(f"{target_rpm}\n".encode())
                        time.sleep(5)
                        
                        esp32.reset_input_buffer() # Flush old RPM data
                        readings = get_valid_reading(esp32)
                        actual_rpm = readings[1]
                        
                        if is_fan_safe(target_rpm, actual_rpm):
                            print(f"-> Fan verified! Actual: {actual_rpm} RPM. Proceeding.")
                            fan_is_ready = True
                        else:
                            print(f"-> RPM mismatch! Target: {target_rpm} | Actual: {actual_rpm}. Retrying...")

                    # 2. Set PSU Current & Output ON
                    print(f"Applying {amps}A from PSU...")
                    psu.psu_SetCurrent(amps)
                    psu.psu_OutputON()
                    
                    # Give the PSU and INA226 200ms to settle (Methodologically defensible t=0 buffer)
                    time.sleep(0.2)
                    
                    # 3. Destroy stale serial packets
                    esp32.reset_input_buffer()

                    # 4. Capture first telemetry packet BEFORE starting the timer
                    print("Capturing true initial state...")
                    init_readings = get_valid_reading(esp32)
                    
                    init_actual_curr = init_readings[0]
                    init_actual_rpm  = init_readings[1]
                    init_pwm_duty    = init_readings[2]
                    init_t_amb       = init_readings[3]
                    init_t_hot       = init_readings[4]
                    init_t_cold      = init_readings[5]
                    
                    print(f"-> Init Packet Captured: {init_actual_curr:.3f}A | T_cold: {init_t_cold:.2f}°C")

                    # 5. Start Timer (t = 0)
                    start_t = time.time()
                    print(f"Timer started (t=0s). Monitoring transient curve for {WAIT_TIME_SECONDS} seconds...")

                    # 6. Continue running & capture data at milestones
                    captured_milestones = {}
                    
                    while True:
                        elapsed = time.time() - start_t
                        if elapsed >= WAIT_TIME_SECONDS:
                            break
                        
                        for m in MILESTONES:
                            if m not in captured_milestones and elapsed >= m:
                                # We do not explicitly flush here because the manual readline() at the bottom
                                # of this loop keeps the buffer perfectly synchronized.
                                intermediate_readings = get_valid_reading(esp32)
                                captured_milestones[m] = (
                                    intermediate_readings[0],
                                    intermediate_readings[3], 
                                    intermediate_readings[4], 
                                    intermediate_readings[5]
                                )
                                print(f"   -> [Milestone {m}s] Amps: {intermediate_readings[0]:.3f} | T_cold: {intermediate_readings[5]:.2f}°C")
                        
                        # Empty the serial buffer
                        esp32.readline() 
                        
                        # CPU efficiency sleep
                        time.sleep(0.1)

                    # 7. Capture final data at 180s
                    print(f"Capturing final thermal state at {WAIT_TIME_SECONDS}s...")
                    esp32.reset_input_buffer() # One final flush to guarantee absolute accuracy for the 180s mark
                    final_readings = get_valid_reading(esp32)
                    final_actual_curr = final_readings[0]
                    final_t_amb, final_t_hot, final_t_cold = final_readings[3], final_readings[4], final_readings[5]

                    # 8. Build the row and log it to CSV
                    timestamp = int(time.time())
                    
                    csv_row = [
                        trial_number, timestamp, amps, target_rpm, 
                        init_actual_curr, init_actual_rpm, init_pwm_duty,
                        init_t_amb, init_t_hot, init_t_cold
                    ]
                    
                    for m in MILESTONES:
                        if m in captured_milestones:
                            csv_row.extend(list(captured_milestones[m]))
                        else:
                            csv_row.extend([FAULT, FAULT, FAULT, FAULT]) 
                            
                    csv_row.extend([final_actual_curr, final_t_amb, final_t_hot, final_t_cold])
                    
                    writer.writerow(csv_row)
                    file.flush() 
                    
                    print(f"-> LOG COMPLETE (Trial {trial_number}): {amps}A, {target_rpm}RPM | Tcold: {init_t_cold:.2f}°C -> {final_t_cold:.2f}°C")

                    # Thermal flush before moving to the next current point
                    thermal_flush(esp32)
                    
                    # Increment trial counter for the next run
                    trial_number += 1
            
            cycle_count += 1

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n[USER ABORT] Emergency shutdown initiated. Turning off PSU and maxing Fan...")
        try:
            psu.psu_SetCurrent(0.0)
            psu.psu_OutputOFF()
            psu.psu_Disconnect()
            if 'esp32' in locals():
                esp32.write(b"4000\n") 
                esp32.close()
        except:
            pass