# IGVC Software 2027

This repository contains the full software stack for our IGVC 2027 entry, <TBD>. This includes the robot control software, communication schemas, and firmware.

## Overview

```bash
├── firmware # Embedded firmware for microcontrollers and similar devices
```

## Getting Started

### Cloning

```bash
git clone https://github.com/SoonerRobotics/igvc_software_2027.git
cd igvc_software_2026
```

### Setup

TODO

## Project Breakdown

### `firmware/` - Firmware

The [firmware/](firmware/) folder contains firmware for all of our microcontrollers and devices. The following tree gives a short description of each. Each individual folder, e.g. [firmware/can_controller](firmware/can_controller), contains their own README file which describes the firmware and any special tools needed to build/deploy it.

```bash
firmware/
├── estop_receiver # Listens for EStop signals and motor instructions sent by the EStop Remote
├── estop_remote # Sends EStop signals and motor instructions to the EStop Receiver
├── hub # Routes power, ground, and other signals around the robot and emits per device voltage/current telemetry
└── safety_lights # Controls the safety lights on the robot
```
