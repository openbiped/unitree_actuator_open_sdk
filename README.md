# unitree-actuator-open-sdk

This repository provides a **Unitree Actuator SDK compatible** C++ and Python API.
It is in no way affiliated with Unitree. Use at your own discretion.

## Notice

Supports: GO-M8010-6 motor, GO2, A1 motor, B1 motor

## Build

```
make
```

## Add user to dialout group

```
sudo usermod -aG dialout $USER
```

Log out and back in. Now you dont need to do sudo to access /dev/ttyUSB0

## Test Motor

GO-M8010
```
./build/x86_64-unknown-linux-gnu/debug/example_goM8010_6_motor_output
```

GO-2 Motor test
```
./build/x86_64-unknown-linux-gnu/debug/example_go2_motor_output
```

## Change Motor ID

```
./build/x86_64-unknown-linux-gnu/debug/go1_changeid /dev/ttyUSB 0 1
```
Change Go-M8010 motor ID from 0 to 1

```
./build/x86_64-unknown-linux-gnu/debug/go2_changeid /dev/ttyUSB 0 1
```
Change Go2 motor ID from 0 to 1

## Astral UV (https://docs.astral.sh/uv/getting-started/installation)

```
cd python
uv run examples/example_go2_motor.py
```

## Python

The wheel/package exports the same public names as the upstream Unitree Python wrapper:

- `SerialPort`
- `MotorCmd`, `MotorData`
- `MotorType`, `MotorMode`
- `queryMotorMode()`, `queryGearRatio()`

The native extension lives at `unitree_actuator_open_sdk._unitree_actuator_open_sdk` and is re-exported
from `unitree_actuator_open_sdk/__init__.py`.

## Local python build

```bash
make python
```

This will place a local dev package at:

```
python/lib/unitree_actuator_open_sdk/
  __init__.py
  _unitree_actuator_open_sdk*.so
```

so the upstream-style examples under `python/examples` can run unchanged.
