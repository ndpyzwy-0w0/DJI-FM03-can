# DJI FM03 CAN

[简体中文](README.md) | **English**

CAN protocol reverse engineering and standalone control for the DJI FM03 focus motor. This project includes ESP32 adapter firmware, a desktop control app, capture analysis tools, and reproducible research evidence. It supports speed control, automatic travel calibration, coordinate feedback, and experimental closed-loop positioning.

Core control has been demonstrated on hardware: the motor can be driven without an RS3, internal travel calibration can be triggered, and post-calibration coordinates can be read. Automatic positioning and strict targeting are implemented in software; their mechanical accuracy and reliability still need hardware validation.

## Features

- **Standalone motion control:** forward/reverse speed, zero-speed stop, timed motion, and pulses.
- **Automatic travel calibration:** trigger the motor's internal limit-finding routine and display phase changes; successful user tests and post-calibration soft-limit evidence are recorded.
- **Coordinate feedback and recording:** validity/age display and A/B coordinate marks; the currently observed range is 0–4096.
- **Target positioning:** move to a coordinate or A/B mark, optional approach slowdown, configurable minimum speed and timeout.
- **Strict targeting:** low-speed corrections; success requires an exact target coordinate that remains stable after ordinary motion is confirmed zero. Stalls in coordinate progress, invalid feedback, and timeouts report failure.
- **Automatic experiment logs:** raw received UART bytes, command/write results, status/coordinates, A/B marks, and positioning events with recoverable offline summaries.
- **Protocol research tools:** CAN capture, per-CAN-ID DUML reassembly, CRC validation, packet construction, and comparative analysis.

The current control firmware is **1.2** and the desktop app is **1.6**. The positioning loop runs on the host and uses ordinary speed commands. PID has not been implemented.

## Hardware and operating modes

The tested setup uses the DJI RS Focus Motor (2022 model, FM03), an ESP32-S3-DevKitC-1 N16R8, and an SN65HVD230 CAN transceiver. CAN runs at **1 Mbps** and UART at **921600 baud**. The current control firmware uses **GPIO4/TX and GPIO5/RX**.

- **Passive capture:** retain the RS3 ↔ FM03 connection and use the `listen_1m` environment.
- **Standalone control:** use `motor_control`; the RS3 must be physically disconnected from the test bus while transmitting.

Verify your module's supply, common ground, CANH/CANL, termination, and silent-mode configuration. Listener wiring must not be reused unchanged for active transmission; wiring and early validation notes are retained in the [historical README](README.history.md). The motor keeps its original firmware; this project's firmware runs on the external ESP32.

## Quick start

The tested software environment is Windows / Python 3.12. Run in PowerShell:

```powershell
git clone https://github.com/ndpyzwy-0w0/DJI-FM03-can.git
cd DJI-FM03-can
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe tools/motor_control.py
```

Starting the app does not automatically connect, ARM, calibrate, or move the motor. The adapter needs the appropriate firmware. These commands build only; they do not flash hardware:

```powershell
.\.venv\Scripts\python.exe -m platformio run -e motor_control
# For passive capture only:
.\.venv\Scripts\python.exe -m platformio run -e listen_1m
```

Before flashing, verify the device and whether a capture is active. Historical replay/diagnostic environments have their own trigger behavior and are not substitutes for these two standard entry points.

Read the [current operating guide](reports/20261010_strict_positioning_guide.md) before controlling hardware. On a verified standalone setup, connect manually, ARM, then STOP and wait for ordinary motion status to become zero. Calibrate manually when needed, and obtain fresh phase-3 coordinates before using A/B marks or target positioning.

**There is no verified cancel command for internal calibration.** Ordinary STOP, DISARM, and the watchdog do not guarantee cancellation. Ordinary stops and timed commands are also not hardware stop guarantees during communication or firmware faults.

## Build a single-file Windows app

PyInstaller 6.22.3 has been tested locally:

```powershell
.\.venv\Scripts\python.exe -m pip install pyinstaller==6.22.3
.\.venv\Scripts\python.exe tools/build_host_app.py --name FM03_MotorControl_StrictPosition
```

The output is `dist/FM03_MotorControl_StrictPosition.exe`. `dist/`, virtual environments, build caches, and local backups are excluded from the source repository; their mention does not imply downloadable release assets are available.

## Code and evidence

- [src/motor_control.cpp](src/motor_control.cpp): adapter control firmware and UART commands.
- [src/motor_status.h](src/motor_status.h): motor status parsing.
- [tools/motor_control.py](tools/motor_control.py): desktop control app.
- [tools/position_control.py](tools/position_control.py): reusable coordinate controller without GUI/serial dependencies, for integration into other control projects.
- [tools/control_session.py](tools/control_session.py): automatic logging and offline summaries.
- [captures/INDEX.md](captures/INDEX.md) and [CAPTURE_FILE_MAP.md](CAPTURE_FILE_MAP.md): raw evidence and action mappings.
- [reports/](reports/): protocol analyses, validation records, and operating guides.

The coordinate controller requires fresh status/feedback plus send and logging callbacks from its caller; see the [strict-targeting guide](reports/20261010_strict_positioning_guide.md) for its interface. Standalone embedded execution still requires porting. This repository covers FM03 protocols and generic control only.

## Validation and limitations

The current implementation has passed **67 relevant offline software tests**, covering fake serial devices, logging, status, calibration, and positioning. Run the positioning subset (23 tests) and protocol self-checks below; these do not open a real serial port:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tools -p test_position_control.py -v
.\.venv\Scripts\python.exe tools/capture.py --self-check
.\.venv\Scripts\python.exe tools/analyze_protocol.py --self-check
```

Automatic calibration has successful hardware evidence. Using mechanical marks, the user confirmed that repeated calibrations produced matching mechanical positions for the same coordinate under the tested mounting/travel conditions. Software tests and a strict success predicate are not measurements of mechanical positioning accuracy.

Still unresolved:

- Automatic positioning success rates, bidirectional backlash, load effects, and minimum usable speed.
- Whether coordinates originate from an encoder or an internal reference, and the physical meaning of 4096. Do not assume degrees, millimeters, or encoder resolution.
- Mapping after travel changes, coordinate/limit retention across power cycles or resets, calibration cancellation, and complete fault semantics.
- Other unknown messages, compatibility across devices, and long-term reliability.

The original firmware of the motor's GD32F303CCT6 has not been extracted. An ESP32 backup is not motor firmware. The `firmware-extract` research branch has not been merged or uploaded. See [PROJECT_STATUS.md](PROJECT_STATUS.md) for the full scope.

## Further reading

- [Strict targeting and minimum speed](reports/20261010_strict_positioning_guide.md)
- [Automatic experiment logging](reports/20261010_control_logging_guide.md)
- [Hardware calibration test](reports/20261010_183747_calibration_findings.md)
- [Coordinate/mechanical-position confirmation](reports/20261010_mechanical_coordinate_confirmation.md)
- [Soft-limit evidence](reports/20261009_virtual_limits_findings.md)
- [Historical README](README.history.md), [work log](WORK_LOG.md), and [current handoff](HANDOFF.md)

Historical documents retain their assumptions and stage-specific status; use the project status and latest handoff for current conclusions. Raw captures are preserved unchanged. Device timestamps represent driver dequeue times, and UART logs are not complete CAN bus captures. Research and detailed operating documents are currently mainly in Chinese.
