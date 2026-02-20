"""
Example 5 – Advanced Configuration (Speed PID Tuning)
=======================================================
Reads the current Speed PID gains from VESC flash, applies new
values for Kp, Ki, and Kd individually, then reads them back to
verify the write was successful.

Enable DEBUG = True to see every raw UART packet (hex) sent and
received during the read-modify-write cycle.

Usage:
    python 05_advanced_pid_tuning.py

Expected output (debug off):
    === Speed PID Tuning ===

    Current PID gains:
      Kp = 0.004000
      Ki = 0.004000
      Kd = 0.000100

    Applying new gains: Kp=0.005, Ki=0.003, Kd=5e-05
      set_kp result: OK
      set_ki result: OK
      set_kd result: OK

    Verified gains after write:
      Kp = 0.005000  (target: 0.005000)  ✓
      Ki = 0.003000  (target: 0.003000)  ✓
      Kd = 0.000050  (target: 0.000050)  ✓

With debug enabled you will also see hex output such as:
    Sending: 02 05 0e 00 00 00 00 a1 b2 03
    Received payload: 458 bytes
    First 20 bytes: 0e 00 00 00 ...
    Parsed Kp: 0.0040000001
    SUCCESS: Kp changed from 0.004000 to 0.005000

Typical Kp/Ki/Kd ranges: 0.0001 – 0.01
"""

import time
import sys
sys.path.append("..")

from vesc_uart import VescUart

SERIAL_PORT = "/dev/ttyUSB0"   # Windows: "COM3"
DEBUG       = False             # Set True for raw hex packet inspection

# New PID gains to apply
NEW_KP = 0.005000
NEW_KI = 0.003000
NEW_KD = 0.000050

vesc = VescUart(SERIAL_PORT)
vesc.set_debug(DEBUG)

print("=== Speed PID Tuning ===\n")

# --- Step 1: Read current gains ---
config = vesc.get_mcconf()
if config:
    print("Current PID gains:")
    print(f"  Kp = {config['s_pid_kp']:.6f}")
    print(f"  Ki = {config['s_pid_ki']:.6f}")
    print(f"  Kd = {config['s_pid_kd']:.6f}\n")
else:
    print("ERROR: Failed to read configuration. Is VESC in UART app mode?")
    vesc.close()
    sys.exit(1)

# --- Step 2: Write new gains ---
print(f"Applying new gains: Kp={NEW_KP}, Ki={NEW_KI}, Kd={NEW_KD}")

ok_kp = vesc.set_kp(NEW_KP)
ok_ki = vesc.set_ki(NEW_KI)
ok_kd = vesc.set_kd(NEW_KD)

print(f"  set_kp result: {'OK' if ok_kp else 'FAILED'}")
print(f"  set_ki result: {'OK' if ok_ki else 'FAILED'}")
print(f"  set_kd result: {'OK' if ok_kd else 'FAILED'}")

# --- Step 3: Verify written values ---
time.sleep(0.3)
verify = vesc.get_mcconf()

if verify:
    print("\nVerified gains after write:")
    for label, target, key in [
        ("Kp", NEW_KP, "s_pid_kp"),
        ("Ki", NEW_KI, "s_pid_ki"),
        ("Kd", NEW_KD, "s_pid_kd"),
    ]:
        actual = verify[key]
        match  = "✓" if abs(actual - target) < 1e-6 else "✗ MISMATCH"
        print(f"  {label} = {actual:.6f}  (target: {target:.6f})  {match}")
else:
    print("ERROR: Could not verify written configuration.")

vesc.close()