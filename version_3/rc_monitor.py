import time
import csv
from pymavlink import mavutil

# Pixhawk connection setup (adjust COM port if needed)
print("Connecting to Pixhawk...")
connection = mavutil.mavlink_connection('COM6', baud=115200)
connection.wait_heartbeat()
print("Heartbeat detected! Monitoring RC Channels...")

# Open CSV file to log data
with open('rc_values_log.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Timestamp', 'CH1 (Roll)', 'CH2 (Pitch)', 'CH3 (Throttle)', 'CH4 (Yaw)'])

    try:
        while True:
            msg = connection.recv_match(type='RC_CHANNELS', blocking=True)
            if msg:
                timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
                ch1 = msg.chan1_raw
                ch2 = msg.chan2_raw
                ch3 = msg.chan3_raw
                ch4 = msg.chan4_raw
                
                print(f"[{timestamp}] Roll: {ch1} | Pitch: {ch2} | Throttle: {ch3} | Yaw: {ch4}")
                writer.writerow([timestamp, ch1, ch2, ch3, ch4])
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nMonitoring stopped. Log saved successfully.")
