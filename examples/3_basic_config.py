"""
Example 3 – Basic Configuration
=================================
Reads live VESC status (voltage, fault code, controller ID) and
prints the current Speed PID gains from motor configuration (MCCONF).

Usage:
    python 03_basic_configuration.py

Expected output:
    === VESC Basic Configuration Check ===

    Live Status:
      Input Voltage  : 24.3 V
      Controller ID  : 0
      Fault Code     : 0 (OK)

    Motor Configuration (Speed PID):
      Kp             : 0.004000
      Ki             : 0.004000
      Kd             : 0.000100
      Kd Filter      : 0.010000
      Min ERPM       : 900.0
      Allow Braking  : True

Note: Requires VESC app mode set to UART in VESC Tool.
"""

import sys
sys.path.append("..")

from vesc_uart import VescUart

SERIAL_PORT = "/dev/ttyUSB0"   # Windows: "COM3"

vesc = VescUart(SERIAL_PORT)
vesc.set_debug(False)           # Set True to inspect raw UART packets

print("=== VESC Basic Configuration Check ===\n")

# --- Live status ---
if vesc.get_values():
    print("Live Status:")
    print(f"  Input Voltage  : {vesc.data.input_voltage:.1f} V")
    print(f"  Controller ID  : {vesc.data.controller_id}")
    print(f"  Fault Code     : {vesc.data.fault_code} "
          f"({'OK' if vesc.data.fault_code == 0 else 'FAULT'})")
else:
    print("ERROR: Could not reach VESC. Check wiring and baud rate.")

print()

# --- Motor configuration ---
print("Motor Configuration (Speed PID):")
config = vesc.get_mcconf()

if config:
    print(f"  Kp             : {config['s_pid_kp']:.6f}")
    print(f"  Ki             : {config['s_pid_ki']:.6f}")
    print(f"  Kd             : {config['s_pid_kd']:.6f}")
    print(f"  Kd Filter      : {config['s_pid_kd_filter']:.6f}")
    print(f"  Min ERPM       : {config['s_pid_min_erpm']:.1f}")
    print(f"  Allow Braking  : {config['s_pid_allow_braking']}")
else:
    print("  ERROR: Could not read motor configuration.")

vesc.close()