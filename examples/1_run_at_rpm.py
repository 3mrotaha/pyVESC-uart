"""
Example 1 – Running the Motor at RPM
=====================================
Connects to VESC, spins the motor to a target electrical RPM,
monitors live telemetry for 5 seconds, then stops the motor cleanly.

Usage:
    python 01_run_at_rpm.py

Expected output:
    Spinning motor to 3000 ERPM...
      RPM:     1204  |  Current:   1.80 A  |  Duty:  12.3%
      RPM:     2761  |  Current:   2.15 A  |  Duty:  21.7%
      RPM:     3001  |  Current:   1.93 A  |  Duty:  23.4%
      RPM:     3000  |  Current:   1.88 A  |  Duty:  23.4%
    Stopping motor...

Note: VESC uses electrical RPM. To get mechanical RPM, divide by
      the number of motor pole pairs.
"""

import time
import sys
sys.path.append("..")   # adjust if vesc_uart.py is in a different location

from vesc_uart import VescUart

SERIAL_PORT = "/dev/ttyUSB0"   # Windows: "COM3"
TARGET_RPM  = 3000             # Electrical RPM
DURATION    = 5                # Seconds to run

vesc = VescUart(SERIAL_PORT)

print(f"Spinning motor to {TARGET_RPM} ERPM...")
vesc.set_rpm(TARGET_RPM)

start = time.time()
while time.time() - start < DURATION:
    if vesc.get_values():
        print(f"  RPM: {vesc.data.rpm:>8.0f}  |  "
              f"Current: {vesc.data.avg_motor_current:>6.2f} A  |  "
              f"Duty: {vesc.data.duty_cycle_now * 100:>5.1f}%")
    else:
        print("  -- No response from VESC --")
    time.sleep(0.5)

print("Stopping motor...")
vesc.set_rpm(0)
time.sleep(1)
vesc.close()