import time

def monitor_battery(current_voltage):
    LOW_THRESHOLD = 10.8
    CRITICAL_THRESHOLD = 10.5
    
    print(f"Current Battery Voltage: {current_voltage}V")
    if current_voltage <= CRITICAL_THRESHOLD:
        return "CRITICAL FAILSAFE: Triggering Immediate LAND!"
    elif current_voltage <= LOW_THRESHOLD:
        return "LOW BATTERY FAILSAFE: Triggering RTL (Return to Launch)!"
    else:
        return "Battery Status: HEALTHY"

# Simulation loop
mock_voltages = [12.2, 11.5, 10.9, 10.7, 10.4]
for v in mock_voltages:
    status = monitor_battery(v)
    print(status)
    print("-" * 40)
    time.sleep(1)
