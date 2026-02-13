# ROS2 Parameter Demo - Workflow Components

This project contains a ROS2-based ultrasonic data acquisition and processing system with real-time visualization and parameter control capabilities.

## Overview

The system consists of three main ROS2 nodes that work together:

1. **Publisher Node** (`publisher.py`) - Interfaces with Vitesse hardware to acquire ultrasonic A-scan data
2. **Processor Node** (`processor.py`) - Processes raw A-scan data and displays real-time visualizations
3. **Controller Node** (`controller.py`) - Provides a GUI for monitoring and controlling system parameters

## Architecture

```
┌─────────────────┐      Ascan Message      ┌─────────────────┐
│                 │ ──────────────────────> │                 │
│    Publisher    │                         │    Processor    │
│  (Hardware I/O) │                         │ (Visualization) │
└─────────────────┘                         └─────────────────┘
         ↑                                           ↑
         │                                           │
         │          ROS2 Parameters                  │
         └───────────────────┬───────────────────────┘
                             │
                    ┌────────────────┐
                    │   Controller   │
                    │     (GUI)      │
                    └────────────────┘
```

## Components

### Publisher Node (`ascan_publisher`)

**Purpose**: Interfaces with Vitesse ultrasonic hardware to acquire A-scan waveform data from up to 8 channels.

The publisher does not include code on temperature gathering. You should implement your own temperature aquisition code, and there is a segment in the code where you can do this.

**Key Features**:

- Configures and initializes Vitesse hardware via SPI interface
- Publishes A-scan data at a configurable rate (PRF)
- Supports multi-channel operation with individual channel enable/disable

**Parameters**:

- `numAverages` (int, 1-1000): Number of waveform averages for noise reduction
- `channelsOnReceive` (int[8]): Receive channel enable flags [1,0,0,0,0,0,0,0] (read-only)
- `channelsOnDrive` (int[8]): Channels on driver array (which channels to drive)
- `numCycles` (int, 1-3): Number of excitation cycles (read-only)
- `recordLength` (float): Recording duration in seconds (read-only)
- `PRF` (int, 1-5000 Hz): Pulse repetition frequency (read-only)
- `phaseArrayMicro` (int[8]): Per-channel phase delays in microseconds (read-only)
- `delayArrayMicro` (int[8]): Per-channel recording delays in microseconds (read-only)
- `samplingMode` (int, 16 or 24): Sampling mode (16 or 24 bit)
- `pulseFrequency` (int): Pulse frequency in Hz
- `opFrequency` (int): Operation frequency in Hz

**Published Topics**:

- `/ascan` (interfaces/msg/Ascan): Contains active channels, encoded waveform data, and temperature

### Processor Node (`ascan_processor`)

**Purpose**: Processes raw A-scan data and provides real-time visualization with automatic peak detection.

**Key Features**:

- Real-time matplotlib-based visualization for active channels
- Automatic subplot arrangement based on number of active channels
- Peak detection and thickness calculation
- Multiple processing modes (first peak, multi-echo, zero crossing)
- Temperature-corrected measurements

**Parameters** (per channel, ch1-ch8):

- `ch{N}_sampling_freq` (int): Sampling frequency in Hz
- `ch{N}_lowbound_time` (int): Lower bound time in microseconds
- `ch{N}_minimum_thickness` (float): Minimum detectable thickness in mm
- `ch{N}_wave_velocity` (float): Wave velocity in m/s
- `ch{N}_num_cycles` (int): Number of cycles for processing
- `ch{N}_signal_frequency` (int): Signal frequency in Hz
- `ch{N}_threshold_snr` (float): SNR threshold in dB
- `ch{N}_noise_width` (int): Noise calculation window width
- `ch{N}_calibration_index` (int): Calibration reference index
- `ch{N}_firstPeak` (bool): Enable first peak detection mode
- `ch{N}_multiEcho` (bool): Enable multi-echo mode
- `ch{N}_zeroCrossing` (bool): Enable zero crossing detection
- `ch{N}_temperatureCorrected` (bool): Enable temperature correction

**Subscribed Topics**:

- `/ascan` (interfaces/msg/Ascan): Receives waveform data from publisher

### Controller Node (`parameter_manager_gui`)

**Purpose**: Provides a PyQt5-based GUI for real-time parameter monitoring and control.

**Key Features**:

- Automatic discovery of ROS2 nodes and their parameters
- Tabbed interface with separate tabs for each active channel
- Parameter filtering based on configuration file
- Support for various parameter types (sliders, spinboxes, checkboxes)
- Real-time parameter updates with visual feedback
- Configurable display names and parameter bounds

**Configuration** (`config.json`):

- `multipleTabs`: Enable/disable tabbed interface
- `whitelist`: Enable parameter whitelisting
- `hideReadOnlyParameters`: Hide read-only parameters
- `whitelistNodes`: List of nodes to display
- `whitelistParameters`: Parameters to show per node
- `parameterDisplayNames`: Custom display names for parameters
- `parameter_bounds`: Min/max bounds for numeric parameters

## Channel Numbering

The system uses 1-based channel numbering (1-8) for user-facing interfaces:

- Channel parameters are named `ch1_*` through `ch8_*`
- Display shows "Channel 1" through "Channel 8"
- The `channelsOnReceive` parameter uses array indices 0-7 internally but presents channels as 1-8 to users

## Message Format

### Ascan Message

```
int32[] active_channels    # List of active channel numbers (1-8)
string[] ascan_data       # Base64-encoded waveform data for each active channel
float32 temperature       # Current temperature reading
```

## Dependencies

- ROS2 Humble or later (tested on Jazzy) (Desktop Variant is required for processor and controller)
- Vitesse ultrasonic system (for publisher)

## Installation

1. Under the project root folder, run

```bash
colcon build
```

to build all the packages.

## Usage

### Starting the System

1. Launch the publisher node (requires Vitesse hardware):

```bash
ros2 run ros_parameter_demo publisher
```

2. Launch the processor node for visualization:

```bash
ros2 run ros_parameter_demo processor
```

3. Launch the controller GUI:

```bash
ros2 run ros_parameter_demo controller
```

### Configuring Channels

To enable specific receive channels, modify the `channelsOnReceive` parameter in the publisher file. For example:

- `[1,0,0,0,0,0,0,0]` - Only channel 1 active
- `[1,0,1,1,0,0,0,0]` - Channels 1, 3, and 4 active
- `[1,1,1,1,1,1,1,1]` - All 8 channels active

### Processing Modes

Each channel can operate in different processing modes:

- **First Peak**: Detects the first significant peak in the waveform
- **Multi-Echo**: Processes multiple reflections
- **Zero Crossing**: Uses zero-crossing detection for improved accuracy
- **Temperature Corrected**: Applies temperature compensation to measurements

Note: First Peak and Multi-Echo modes are mutually exclusive.

If you set both in the controller, the processor falls back to which none of the four parameters are set.

## Configuration Files

- `config.json`: Controller GUI configuration
- Channel parameters can be saved/loaded via ROS2 parameter services

## Troubleshooting

1. **No visualization appears**: Ensure at least one channel is enabled in `channelsOnReceive`
2. **Controller doesn't show parameters**: Controller won't show anything until the publishing node is on. Check that nodes are running and `config.json` whitelist is correct.
3. **Hardware errors**: Verify Vitesse device is connected and permissions are set correctly.

## Future Enhancements

- Add data logging capabilities
- Implement parameter presets
- Add export functionality for processed data
- Support for additional processing algorithms
- Integration with ROS2 bag recording
