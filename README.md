# HaptiFeet

A hands-free navigational aid for the visually impaired that uses haptic feedback to convey spatial awareness.

![HaptiFeet Device](docs/images/device.jpg)

## Overview

HaptiFeet is a wearable assistive device designed to help visually impaired individuals navigate their environment through haptic feedback. The system uses LiDAR sensing to detect nearby objects and provides intuitive vibration patterns through eight independently controlled haptic modules positioned around the user's waist.

**Team:** Isak Keyes, Ben Wolf, Pakorn Jantacumma, Wei Chen Zhang  
**Course:** UW Madison ECE453 FA25

## Features

- **360° Spatial Awareness**: LiDAR sensor provides 0.72° resolution with up to 500 samples per rotation
- **Hands-Free Operation**: Worn around the waist, leaving hands free for other tasks
- **8 Haptic Zones**: Individual vibration motors cover a 120° forward field of view in 15° sectors
- **Distance Mapping**: Vibration intensity increases as objects get closer
- **Adjustable Sensitivity**: Potentiometer allows users to scale overall haptic intensity
- **Persistent Calibration**: EEPROM stores haptic calibration data across power cycles
- **Rechargeable**: Built-in battery management with USB-C charging

## Hardware Architecture

### Core Components

- **Microcontroller**: PSoC6 (CY8CPROTO-063-BLE)
- **Distance Sensor**: LiDAR with UART communication, 10Hz scanning frequency
- **Haptic Actuators**: 8x Eccentric Rotating Mass (ERM) motors
- **Haptic Drivers**: DRV2605 I²C-controlled haptic driver chips with auto-calibration
- **Memory**: EEPROM for storing calibration presets
- **Power Management**: 
  - MP2637GR-Z battery charging IC
  - TLV62569DBVR boost converter (3.6V → 5.0V)
  - TPS62569DBVR buck converter (5.0V → 3.3V)
  - Automatic switching between USB and battery power
  - Simultaneous charging and operation support

### Power System Features

- Rechargeable lithium-ion batteries
- USB-C charging port
- Boost converter for battery voltage regulation
- Buck converter for logic voltage supply
- Current and voltage limiting for safe operation
- Over-temperature, reverse voltage, and short circuit protection
- Visual LED indicators for power status (5V and 3.3V)

### PCB Design

- **Main Board**: 100mm × 35mm custom PCB
- **Haptic Modules**: 8 individual 35mm × 35mm boards
- Bidirectional analog multiplexers for individual haptic addressing
- Clock gating on I²C lines to enable same-address communication
- Shunt resistor-based ID assignment for each haptic module

## Software Architecture

### Module Structure

```
task_core.c          # Main control loop and state management
├── task_lidar.c     # LiDAR data acquisition and parsing
├── task_poten.c     # Potentiometer reading for sensitivity adjustment
├── task_haptics.c   # Haptic motor control and intensity mapping
└── task_eeprom.c    # Calibration data persistence
```

### Control Flow

1. **Initialization**
   - Read EEPROM for saved haptic calibration presets
   - If presets found → Load and apply to haptic drivers
   - If no presets → Run calibration routine and save to EEPROM

2. **Main Loop**
   - Read LiDAR sensor data (360° scan at 10Hz)
   - Read potentiometer for global intensity scaling
   - Process LiDAR data into 8 sectors (15° each, covering 120° FOV)
   - Calculate vibration intensity for each sector based on minimum distance
   - Update haptic motors via I²C communication

### LiDAR Processing

- Continuous rotation scanning at 10Hz
- Data collected via UART communication
- 120° field of view segmented into 8 sectors
- Distance measurements mapped to haptic intensity
- Frame validation and error checking

## Enclosure Design

- 3D-printed custom housing
- Ergonomic waist-mounted design
- Integrated mounting for:
  - LiDAR sensor (forward-facing)
  - Main PCB
  - Battery pack
  - Power switch
  - USB-C charging port
  - Potentiometer knob
  - Status LEDs

## Building the Project

### Hardware Requirements

- PSoC6 development kit (CY8CPROTO-063-BLE)
- LiDAR sensor module
- 8x ERM haptic actuators
- 8x DRV2605 haptic driver boards
- Custom PCB (files in `/hardware/pcb/`)
- Power management components
- 3D-printed enclosure (files in `/hardware/enclosure/`)

### Software Requirements

- PSoC Creator IDE
- Cypress PSoC Programmer

### Building Firmware

1. Open the project in PSoC Creator
2. Build the project (Build → Build HaptiFeet)
3. Program the PSoC6 device

## Usage

1. **Power On**: Flip the power switch
2. **Wait for Initialization**: Green LED indicates system ready
3. **Adjust Sensitivity**: Turn potentiometer to set comfortable vibration level
4. **Navigate**: Wear device around waist with sensor facing forward
   - Stronger vibrations = closer objects
   - Vibration location corresponds to object direction

## Challenges Overcome

- **Enclosure Alignment**: Adjusted case holes through careful modification
- **Power Supply**: Buck converter current limitations resolved with supplementary module
- **Battery Fit**: Modified battery holder to accommodate larger cells
- **PCB Fixes**: Hand-soldered missing connections and resistor adjustments
- **PSoC Power**: Switched to 5V supply via KitProg for reliability

## Future Improvements

- [ ] Integrate PSoC directly into main PCB
- [ ] Reduce PCB footprint and eliminate unused space
- [ ] Switch to lithium polymer batteries for slimmer profile
- [ ] Separate LiDAR housing for better weight distribution
- [ ] Embed cable connectors into housing for cleaner design
- [ ] Integrate haptic modules directly into wearable strap

## Project Structure

```
HaptiFeet/
├── firmware/
│   ├── task_core.c
│   ├── task_lidar.c
│   ├── task_haptics.c
│   ├── task_eeprom.c
│   └── task_poten.c
├── hardware/
│   ├── pcb/
│   │   ├── main_board/
│   │   ├── haptic_modules/
│   │   └── schematics/
│   └── enclosure/
│       └── 3d_models/
├── docs/
│   └── presentation.pdf
└── README.md
```

## Schematics

### Main Board Connections
- MCU module with EEPROM, LiDAR, and haptic headers
- Power management (boost, buck, charging IC)
- USB interface
- Potentiometer input

### Haptic Modules
- DRV2605 driver with ERM actuator
- Analog multiplexer for I²C addressing
- ID resistor configuration

See `/hardware/pcb/schematics/` for complete circuit diagrams.

## License

This project was developed as part of ECE453 at UW Madison.

## Acknowledgments

- UW Madison ECE Department
- ECE453 Course Staff
- Team members: Isak Keyes, Ben Wolf, Wei Chen Zhang

## Contact

For questions or collaboration opportunities, please reach out through the GitHub repository.

---

**Note**: This project was completed in Fall 2025 as a senior design project for ECE453 at the University of Wisconsin-Madison.
