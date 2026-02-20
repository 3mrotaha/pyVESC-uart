# VescUart — Python UART Library for VESC Motor Controllers

A Python library for communicating with **VESC** (Vedder Electronic Speed Controller) over a serial UART connection using `pyserial`. Based on the original Arduino VescUart library by SolidGeek.

---

## Table of Contents

- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [API Reference](#api-reference)
- [Examples](#examples)
  - [Example 1 – Running the Motor at RPM](#example-1--running-the-motor-at-rpm)
  - [Example 2 – Running the Motor at Duty Cycle](#example-2--running-the-motor-at-duty-cycle)
  - [Example 3 – Basic Configuration](#example-3--basic-configuration)
  - [Example 4 – Reading and Analyzing Telemetry Values](#example-4--reading-and-analyzing-telemetry-values)
  - [Example 5 – Advanced Configuration (PID Tuning)](#example-5--advanced-configuration-pid-tuning)
- [Telemetry Data Fields](#telemetry-data-fields)
- [Fault Codes](#fault-codes)
- [Troubleshooting](#troubleshooting)
- [License](#license)

---

## Features

- Read full VESC telemetry: RPM, voltage, current, temperature, duty cycle, energy counters, fault codes, and more
- Control motor via **RPM**, **duty cycle**, **current**, or **brake current**
- Read and write **motor configuration** (MCCONF) over UART
- Tune **Speed PID** gains (Kp, Ki, Kd) programmatically
- Built-in **CRC16** packet framing compliant with the VESC UART protocol
- **Debug mode** for raw hex inspection of every packet sent and received

---

## Requirements

- Python 3.7+
- [`pyserial`](https://pyserial.readthedocs.io/) library

```
pip install pyserial
```

---

## Installation

Simply place `vesc_uart.py` in your project directory and import it:

```python
from vesc_uart import VescUart
```

No additional build steps are needed.

---

## Quick Start

```python
from vesc_uart import VescUart

vesc = VescUart("/dev/ttyUSB0")        # Connect to VESC

if vesc.get_values():                  # Request telemetry
    print(f"RPM: {vesc.data.rpm}")
    print(f"Voltage: {vesc.data.input_voltage} V")

vesc.set_rpm(3000)                     # Spin motor at 3000 ERPM
vesc.close()
```

> **Windows users:** replace `/dev/ttyUSB0` with your COM port, e.g. `"COM3"`.

---

## API Reference

### `VescUart(serial_port, baudrate=115200, timeout=0.1)`
Opens the serial connection to the VESC.

| Parameter     | Type    | Default    | Description                              |
|---------------|---------|------------|------------------------------------------|
| `serial_port` | `str`   | —          | Port path, e.g. `/dev/ttyUSB0` or `COM3` |
| `baudrate`    | `int`   | `115200`   | Serial baud rate                         |
| `timeout`     | `float` | `0.1`      | Read timeout in seconds                  |

---

### Control Methods

| Method                          | Description                                              |
|---------------------------------|----------------------------------------------------------|
| `set_rpm(rpm)`                  | Set target electrical RPM (positive or negative)         |
| `set_duty(duty)`                | Set duty cycle, `0.0` – `1.0` (e.g. `0.5` = 50%)       |
| `set_current(current)`          | Set motor current in Amps                                |
| `set_brake_current(current)`    | Set regenerative braking current in Amps                 |
| `set_handbrake_current(current)`| Set handbrake (hold) current in Amps                     |
| `set_nunchuck_values()`         | Send nunchuck joystick state to VESC                     |

---

### Telemetry Methods

| Method          | Returns | Description                                        |
|-----------------|---------|----------------------------------------------------|
| `get_values()`  | `bool`  | Polls VESC and populates `vesc.data` with telemetry|
| `print_values()`| —       | Prints all telemetry fields to the console         |

---

### Configuration Methods

| Method              | Returns          | Description                                      |
|---------------------|------------------|--------------------------------------------------|
| `get_mcconf()`      | `dict` or `None` | Reads VESC motor config (MCCONF) over UART       |
| `set_kp(kp_value)`  | `bool`           | Updates Speed PID Kp in VESC flash               |
| `set_ki(ki_value)`  | `bool`           | Updates Speed PID Ki in VESC flash               |
| `set_kd(kd_value)`  | `bool`           | Updates Speed PID Kd in VESC flash               |

---

### Utility Methods

| Method                | Description                              |
|-----------------------|------------------------------------------|
| `set_debug(True/False)` | Toggle raw packet debug output         |
| `close()`             | Close the serial port cleanly            |

---

## Examples

All runnable examples live in the [`examples/`](examples/) folder. Each file is self-contained with its own docstring describing expected output and usage notes.

| File | Description |
|------|-------------|
| [`examples/01_run_at_rpm.py`](examples/01_run_at_rpm.py) | Spin motor to a target electrical RPM and monitor live telemetry for 5 seconds |
| [`examples/02_run_at_duty_cycle.py`](examples/02_run_at_duty_cycle.py) | Ramp motor from 0% to 50% duty cycle in steps, printing actual duty and RPM at each step |
| [`examples/03_basic_configuration.py`](examples/03_basic_configuration.py) | Read live VESC status (voltage, fault code) and print current Speed PID gains from MCCONF |
| [`examples/04_read_telemetry.py`](examples/04_read_telemetry.py) | Continuous monitoring loop with a formatted table of all telemetry fields, fault name resolution, and temperature warnings |
| [`examples/05_advanced_pid_tuning.py`](examples/05_advanced_pid_tuning.py) | Read current PID gains, write new Kp/Ki/Kd values, and verify them — with optional raw packet debug output |

Run any example directly:

```bash
cd examples
python 01_run_at_rpm.py
```

> Before running, open each file and update the `SERIAL_PORT` variable to match your system (e.g. `/dev/ttyUSB0` on Linux/macOS or `COM3` on Windows).

---

## Telemetry Data Fields

After a successful `get_values()` call, all fields below are available on `vesc.data`:

| Field                 | Type    | Unit  | Description                            |
|-----------------------|---------|-------|----------------------------------------|
| `temp_mosfet`         | `float` | °C    | MOSFET bridge temperature              |
| `temp_motor`          | `float` | °C    | Motor temperature (if sensor fitted)   |
| `avg_motor_current`   | `float` | A     | Average current through motor phases   |
| `avg_input_current`   | `float` | A     | Average current drawn from battery     |
| `duty_cycle_now`      | `float` | 0–1   | Current PWM duty cycle                 |
| `rpm`                 | `float` | ERPM  | Electrical RPM                         |
| `input_voltage`       | `float` | V     | Battery / input voltage                |
| `amp_hours`           | `float` | Ah    | Energy discharged (accumulated)        |
| `amp_hours_charged`   | `float` | Ah    | Energy regenerated (accumulated)       |
| `watt_hours`          | `float` | Wh    | Power drawn (accumulated)              |
| `watt_hours_charged`  | `float` | Wh    | Power regenerated (accumulated)        |
| `tachometer`          | `int`   | steps | Motor position steps (signed)          |
| `tachometer_abs`      | `int`   | steps | Absolute position steps                |
| `fault_code`          | `int`   | —     | Active fault code (0 = no fault)       |
| `pid_pos`             | `float` | deg   | PID position estimate                  |
| `controller_id`       | `int`   | —     | VESC CAN ID                            |

---

## Fault Codes

| Code | Name                    | Meaning                                    |
|------|-------------------------|--------------------------------------------|
| 0    | `FAULT_CODE_NONE`       | Normal operation                           |
| 1    | `FAULT_CODE_OVER_VOLTAGE` | Input voltage too high                   |
| 2    | `FAULT_CODE_UNDER_VOLTAGE` | Input voltage too low                   |
| 3    | `FAULT_CODE_DRV`        | Gate driver fault                          |
| 4    | `FAULT_CODE_ABS_OVER_CURRENT` | Absolute overcurrent protection     |
| 5    | `FAULT_CODE_OVER_TEMP_FET` | MOSFET temperature limit exceeded       |
| 6    | `FAULT_CODE_OVER_TEMP_MOTOR` | Motor temperature limit exceeded      |

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| `get_values()` always returns `False` | Wrong port or baud rate | Verify port with `ls /dev/tty*` or Device Manager; ensure VESC app baud matches |
| `get_mcconf()` returns `None` | VESC not in UART app mode | In VESC Tool, set App → General → App to use → `UART` |
| Motor does not spin after `set_rpm()` | No keepalive / current limit | Check max current in VESC Tool; send commands in a loop to keep VESC alive |
| Fault code `4` on high duty ramp | Overcurrent | Ramp duty cycle gradually; reduce motor current limit |
| PID write succeeds but gains do not change | Payload offset mismatch | Enable `set_debug(True)` and check payload length vs. expected 458 bytes |

