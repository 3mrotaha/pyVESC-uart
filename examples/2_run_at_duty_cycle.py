"""
Example 2 – Running the Motor at Duty Cycle
=============================================
Ramps the motor from 0% to 50% duty cycle in 5% steps,
printing actual duty and RPM at each step, then stops cleanly.

Usage:
    python 02_run_at_duty_cycle.py

Expected output:
    Ramping duty cycle from 0% to 50%...
      Duty set:   0%  |  Actual duty:   0.0%  |  RPM:       0
      Duty set:   5%  |  Actual duty:   5.0%  |  RPM:     620
      Duty set:  10%  |  Actual duty:  10.0%  |  RPM:    1252
      Duty set:  20%  |  Actual duty:  20.0%  |  RPM:    2481
      Duty set:  30%  |  Actual duty:  30.0%  |  RPM:    3724
      Duty set:  40%  |  Actual duty:  40.0%  |  RPM:    4951
      Duty set:  50%  |  Actual duty:  50.0%  |  RPM:    6180
    Stopping motor (duty = 0)...

Warning: Always ramp gradually. Jumping to a high duty cycle
         instantly can trigger overcurrent fault codes.
"""

import time
import sys
sys.path.append("..")

from vesc_uart import VescUart

SERIAL_PORT = "/dev/ttyUSB0"   # Windows: "COM3"

vesc = VescUart(SERIAL_PORT)

print("Ramping duty cycle from 0% to 50%...")

for percent in range(0, 55, 5):       # 0%, 5%, 10%, ..., 50%
    duty = percent / 100.0
    vesc.set_duty(duty)
    time.sleep(0.4)                   # allow motor to respond

    if vesc.get_values():
        print(f"  Duty set: {percent:>3d}%  |  "
              f"Actual duty: {vesc.data.duty_cycle_now * 100:>5.1f}%  |  "
              f"RPM: {vesc.data.rpm:>7.0f}")
    else:
        print(f"  Duty set: {percent:>3d}%  |  -- No response --")

print("Stopping motor (duty = 0)...")
vesc.set_duty(0.0)
time.sleep(1)
vesc.close()