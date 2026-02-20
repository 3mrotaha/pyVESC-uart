"""
Example 4 – Reading and Analyzing Telemetry Values
====================================================
Runs a continuous monitoring loop that prints a formatted status
table with all key telemetry fields. Flags faults and temperature
warnings automatically.

Press Ctrl+C to stop monitoring.

Usage:
    python 04_read_telemetry.py

Expected output:
  Time |     RPM |   Duty | Mot A |  In A |  Volt | FET°C | Mot°C | Ah Used | Fault
  0.0s |    2001 | 23.4%  |  1.82 |  1.65 |  24.3 |  38.1 |  25.0 |   0.001 | None
  0.5s |    2001 | 23.4%  |  1.83 |  1.66 |  24.3 |  38.2 |  25.1 |   0.002 | None
  ...

Warnings (⚠) are shown when:
  - Any fault code is active
  - MOSFET temperature exceeds 70°C
  - Motor temperature exceeds 80°C
"""

import time
import sys
sys.path.append("..")

from vesc_uart import VescUart, FaultCode

SERIAL_PORT  = "/dev/ttyUSB0"   # Windows: "COM3"
MONITOR_RPM  = 2000             # ERPM to spin during monitoring (set 0 to monitor without running)
POLL_INTERVAL = 0.5             # seconds between readings

FAULT_NAMES = {
    FaultCode.FAULT_CODE_NONE            : "None",
    FaultCode.FAULT_CODE_OVER_VOLTAGE    : "Over Voltage",
    FaultCode.FAULT_CODE_UNDER_VOLTAGE   : "Under Voltage",
    FaultCode.FAULT_CODE_DRV             : "DRV Fault",
    FaultCode.FAULT_CODE_ABS_OVER_CURRENT: "Abs Over Current",
    FaultCode.FAULT_CODE_OVER_TEMP_FET   : "Over Temp FET",
    FaultCode.FAULT_CODE_OVER_TEMP_MOTOR : "Over Temp Motor",
}

vesc = VescUart(SERIAL_PORT)

if MONITOR_RPM:
    vesc.set_rpm(MONITOR_RPM)
    print(f"Motor set to {MONITOR_RPM} ERPM. Press Ctrl+C to stop.\n")
else:
    print("Monitoring in passive mode (motor not driven). Press Ctrl+C to stop.\n")

print(f"{'Time':>6} | {'RPM':>7} | {'Duty':>6} | {'Mot A':>5} | "
      f"{'In A':>5} | {'Volt':>5} | {'FET°C':>5} | {'Mot°C':>5} | {'Ah Used':>7} | Fault")
print("-" * 95)

try:
    elapsed = 0.0
    while True:
        if vesc.get_values():
            d = vesc.data
            fault_name = FAULT_NAMES.get(d.fault_code, f"Code {d.fault_code}")
            warning = " ⚠" if d.fault_code != 0 or d.temp_mosfet > 70 or d.temp_motor > 80 else ""

            print(f"{elapsed:>5.1f}s | "
                  f"{d.rpm:>7.0f} | "
                  f"{d.duty_cycle_now*100:>5.1f}% | "
                  f"{d.avg_motor_current:>5.2f} | "
                  f"{d.avg_input_current:>5.2f} | "
                  f"{d.input_voltage:>5.1f} | "
                  f"{d.temp_mosfet:>5.1f} | "
                  f"{d.temp_motor:>5.1f} | "
                  f"{d.amp_hours:>7.3f} | "
                  f"{fault_name}{warning}")
        else:
            print(f"{elapsed:>5.1f}s | -- No response from VESC --")

        time.sleep(POLL_INTERVAL)
        elapsed += POLL_INTERVAL

except KeyboardInterrupt:
    print("\nMonitoring stopped by user.")

finally:
    vesc.set_rpm(0)
    time.sleep(0.5)
    vesc.close()