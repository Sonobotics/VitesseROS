# SONUS Vitesse Python API

The SONUS Vitesse Python API is a high-performance interface designed to integrate and control the Sonobotics SONUS Vitesse data acquisition system through Python. This API enables users to configure the device, acquire data, and perform advanced operations seamlessly.

## Table of Contents
1. [Introduction](#introduction)
2. [Installation](#installation)
   - [Prerequisites](#prerequisites)
   - [Driver Installation](#driver-installation)
3. [Quick Start](#quick-start)
4. [Key Features](#key-features)
   - [Context Manager Support](#context-manager-support)
   - [Method Chaining](#method-chaining)
5. [API Reference](#api-reference)
   - [Vitesse Class](#vitesse-class)
   - [Methods](#methods)
6. [Examples](#examples)

## Introduction

The SONUS Vitesse Python API provides a comprehensive interface for the SONUS Vitesse data acquisition system, offering high-speed functionality for configuring channels, setting parameters, and retrieving processed ultrasonic data in Python for further analysis.

## Installation

### Prerequisites

1. Python >= 3.8 (Tested with Python 3.12.3; Python 3.9+ strongly recommended)
2. Required packages:
   ```bash
   pip install -r requirements.txt
   ```

### Driver Installation

#### Windows
Navigate to the `drivers/windows_FTDI` folder and run the executable:
```
CDM212364_Setup.exe
```

#### Linux x86_64
Navigate to the VitesseAPI folder and run:
```bash
cd drivers
sudo bash x86_64_install.sh
```

#### Linux ARM
Navigate to the VitesseAPI folder and run:
```bash
cd drivers
sudo bash arm_install.sh
```

## Quick Start

### Recommended: Using Context Manager
```python
from VitesseAPI import initialiseVitesse

# Using context manager ensures proper cleanup
with initialiseVitesse() as V:
    V.listDevices()  # List all connected devices

    # Acquire data
    array = V.getArray()
    print(f"Acquired data shape: {array.shape}")
    # Device automatically closes when exiting the context
```

### Traditional Approach
```python
from VitesseAPI import Vitesse

# Initialize device
V = Vitesse()
V.listDevices()  # List all connected devices
V.initialise()   # Initialize the first available device

# Acquire data
array = V.getArray()
print(f"Acquired data shape: {array.shape}")

# Finally, close the device
V.closeDevice()
```

## Key Features

### Context Manager Support

The Vitesse class implements Python's context manager protocol, allowing you to use it with the `with` statement. This ensures that the device connection is properly closed even if an error occurs during operation.

**Benefits:**
- Automatic cleanup of device resources
- Exception-safe operation
- Cleaner, more Pythonic code
- Prevents resource leaks

**Example:**
```python
from VitesseAPI import Vitesse

with Vitesse().initialise() as V:
    data = V.getArray()
    # Device automatically closes when exiting this block
    # Even if an exception occurs!
```

or you could do

```python
from VitesseAPI import initialiseVitesse

with initialiseVitesse() as V:
    data = V.getArray()
    # Device automatically closes when exiting this block
    # Even if an exception occurs!
```

### Method Chaining

All configuration methods return `self`, allowing you to chain multiple method calls together. This makes configuration more concise and readable.

**Example:**
```python
with Vitesse().initialise() as V:
    # Chain multiple configuration calls
    V.setAverages(100) \
     .setPrf(1000) \
     .setRecordLength(50e-6)
    
    # Or use setConfig for all-in-one configuration with chaining
    V.setConfig(
        opFrequency=3.6e6,
        channelsOnDrive=[1,0,0,0,0,0,0,0],
        channelsOnReceive=[1,0,0,0,0,0,0,0],
        numAverages=100,
        PRF=1000,
        recordLength=50e-6
    )
```

## API Reference

### Vitesse Class

The main class for interacting with the SONUS Vitesse device.

#### Constructor
```python
V = Vitesse()
```

### Methods

#### `initialise(serialNumber: Optional[str] = None, simulation: bool = False) -> Self`
Initializes a connected Vitesse device, or a virtual (simulated) device if `simulation` is `True`.

**Parameters:**
- `serialNumber` (optional): The serial number of the specific device to connect to. If not provided, connects to the first available device.
- `simulation` (optional): If `True`, initializes a virtual simulated device instead of a physical one. Defaults to `False`.

**Returns:**
- `Self`: Returns the instance for method chaining.

**Example:**
```python
V.initialise()  # Connect to first available device
# or
V.initialise("12345")  # Connect to specific device
# With chaining
V.initialise().setConfig()  # Initialize and configure in one line
# Simulation mode
V.initialise(simulation=True)  # Initialize a virtual device
```

#### `listDevices() -> list[tuple[str, str, int]]`
Lists all connected Vitesse devices.

**Returns:**
List of tuples containing (serialNumber, deviceName, numberOfChannels)

**Example:**
```python
devices = V.listDevices()
for serial, name, channels in devices:
    print(f"Device: {name}, Serial: {serial}, Channels: {channels}")
```

#### `setConfig(numCycles: int = 2, channelsOnReceive: list[int] = [1,0,0,0,0,0,0,0], channelsOnDriver: list[int] = [1,0,0,0,0,0,0,0], PRF: int = 1000, numAverages: int = 100, recordLength: float = 200e-6, phaseArrayMicro: list[int] = [0,0,0,0,0,0,0,0], delayArrayMicro: list[int] = [0,0,0,0,0,0,0,0], peripheralsOnArray: list[int] = [0,0,0,0,0,0,0,0], samplingMode: int = 24, pulseFrequency: int = 200000000, opFrequency: int = 3600000, encoderWheelbase: int = 40, wheelRadius: float = 19, encoderCpr: int = 2048, targetClock: int = 50000000) -> Self`
Configures all device parameters at once. This is recommended over setting each parameter manually, since this ensures the correct precedence of the parameters.

**Returns:**
- `Self`: Returns the instance for method chaining.

**Parameters:**
- `numCycles` (1-3): Number of cycles per symbol
- `channelsOnReceive`: List of 0s and 1s indicating which receive channels to enable
- `channelsOnDriver`: List of 0s and 1s indicating which driving channels to enable
- `PRF` (1-5000): Pulse Repetition Frequency in Hz
- `numAverages` (1-1000): Number of averages for data acquisition
- `recordLength`: Recording length in seconds
- `phaseArrayMicro`: Phase delays in microseconds for each channel
- `delayArrayMicro`: Record delays in microseconds for each channel
- `peripheralsOnArray`: List of 0s and 1s indicating which peripherals to enable
- `samplingMode`: Length in bits of the sampling data (16 or 24)
- `pulseFrequency`: Pulse frequency in Hz
- `opFrequency`: Operating frequency in Hz
- `encoderWheelbase`: Encoder wheelbase value
- `wheelRadius`: Encoder wheel radius
- `encoderCpr`: Encoder Counts Per Revolution
- `targetClock`: Target FPGA clock frequency in Hz

#### `checkValidity(phaseArrayMicro: list[int] | None = None, delayArrayMicro: list[int] | None = None, recordLength: float | None = None, PRF: int | None = None) -> Self`
Validates the configuration parameters to ensure they don't violate timing constraints.

If no value is given for a parameter, the current value stored in the instance will be used.

**Returns:**
- `Self`: Returns the instance for method chaining.

**Raises:**
- `ValueError` if configuration is invalid

#### `setSymbol(numChips: int, numCycles: int) -> Self`
Configures the excitation symbol parameters.

**Parameters:**
- `numChips` (1-100): Number of chips
- `numCycles` (1-3): Number of cycles per symbol

**Returns:**
- `Self`: Returns the instance for method chaining.

#### `setChannelDrive(self, channelsOnDrive: list[int]) -> Self`
Enables or disables specific driver channels (not to be confused with setChannelReceive).

**Parameters:**
- `channelsOnDrive`: List of 8 integers (0 or 1) for channels 1-8 indicating which channels to enable.

**Example:**
```python
V.setChannelDrive([1, 1, 0, 0, 0, 0, 0, 0])  # Enable driver channels 1 and 2
```

**Returns:**
- `Self`: Returns the instance for method chaining.

#### `setChannelReceive(self, channelsOnReceive: list[int]) -> Self`
Enables or disables specific receiver channels for getArray.

**Parameters:**
- `channelsOnReceive`: List of 8 integers (0 or 1) for channels 1-8 indicating which channels to enable.

**Example:**
```python
V.setChannelReceive([1, 1, 0, 0, 0, 0, 0, 0])  # Enable receiver channels 1 and 2
```

**Returns:**
- `Self`: Returns the instance for method chaining.

#### `setAverages(numAverages: int) -> Self`
Sets the number of averages for noise reduction.

**Parameters:**
- `numAverages` (1-1000): Number of acquisitions to average

**Returns:**
- `Self`: Returns the instance for method chaining.

#### `setPrf(PRF: int) -> Self`
Sets the Pulse Repetition Frequency.

**Parameters:**
- `PRF` (1-5000): Frequency in Hz

**Returns:**
- `Self`: Returns the instance for method chaining.

#### `setRecordLength(recordLength: float) -> Self`
Sets the recording window length.

**Parameters:**
- `recordLength`: Length in seconds
  - 8 channels: 0-100 μs
  - 4 channels: 0-200 μs
  - 1 channel: 0-800 μs

**Returns:**
- `Self`: Returns the instance for method chaining.

#### `setTriggerPhasing(phaseArrayMicro: list[int]) -> Self`
Sets trigger phase delays for beam steering.

**Parameters:**
- `phaseArrayMicro`: List of 8 phase delays in microseconds

**Returns:**
- `Self`: Returns the instance for method chaining.

#### `setRecordDelay(delayArrayMicro: list[int]) -> Self`
Sets recording delays for each channel.

**Parameters:**
- `delayArrayMicro`: List of 8 delay values in microseconds

**Returns:**
- `Self`: Returns the instance for method chaining.

#### `getArray() -> numpy.ndarray[tuple[int, int], np.dtype[np.float64]]`
Acquires data from the device.

**Returns:**
- 2D numpy array with shape (numChannelsOn, recordPoints)
- Each row contains data from one enabled channel

**Example:**
```python
data = V.getArray()
print(f"Data shape: {data.shape}")
print(f"Channel 1 data: {data[0]}")
```

#### `closeDevice() -> None`
Safely closes the device connection. It is strongly recommended to always call closeDevice() before end of session.

## Examples

### Basic Data Acquisition
```python
from VitesseAPI import Vitesse

# Using context manager for automatic cleanup
with Vitesse().initialise() as V:
    devices = V.listDevices()
    print(f"Found {len(devices)} devices")
    
    # Configure for single channel, high-speed acquisition using method chaining
    V.setConfig(
        opFrequency=3.6e6,
        numCycles=2,
        channelsOnDrive=[1,0,0,0,0,0,0,0],
        channelsOnReceive=[1,0,0,0,0,0,0,0],
        numAverages=100,
        PRF=1000,
        recordLength=50e-6
    )
    
    # Acquire 10 datasets
    for i in range(10):
        data = V.getArray()
        print(f"Acquisition {i+1}: {data.flatten()}")
    # Device automatically closes when exiting the context
```

### Multi-Channel with Phasing
```python
from VitesseAPI import Vitesse
import matplotlib.pyplot as plt

with Vitesse().initialise() as V:
    # Enable 4 channels with phased array configuration using method chaining
    V.setConfig(
        channelsOnDrive=[1,1,1,1,0,0,0,0],     # Channels 1-4
        channelsOnReceive=[1,1,1,1,0,0,0,0],     # Channels 1-4
        phaseArrayMicro=[0, 0.5, 1.0, 1.5, 0, 0, 0, 0],  # Linear phasing
        recordLength=50e-6
    ).setAverages(100)  # Chain additional configuration
    
    # Acquire and plot
    data = V.getArray()
    
    plt.figure(figsize=(10, 6))
    for i in range(4):
        plt.subplot(2, 2, i+1)
        plt.plot(data[i])
        plt.title(f"Channel {i+1}")
        plt.xlabel("Sample")
        plt.ylabel("Amplitude")
    plt.tight_layout()
    plt.show()
```

### Real-Time Plotting
```python
from VitesseAPI import Vitesse
import matplotlib.pyplot as plt
from VitesseAPI.utils import getConfig, getVersion

# Using context manager ensures device closes even on KeyboardInterrupt
with Vitesse().initialise() as V:
    
    # Configure parameters with method chaining
    V.setConfig(
        opFrequency=3.6e6,
        numCycles=2,
        channelsOnDrive=[1,0,0,0,0,0,0,0],
        channelsOnReceive=[1,0,0,0,0,0,0,0],
        numAverages=100,
        PRF=1000,
        recordLength=50e-6
    )
    
    # Print version and config for verification
    getVersion(V)
    getConfig(V)

    # Setup plot
    fig, ax = plt.subplots()
    ln1, = plt.plot([], [])
    ax.set_xlim(0, V.recordPoints*V.numChannelsOnReceive)
    ax.set_ylim(-2048, 2048)
    ax.set_xlabel('Time')
    ax.set_ylabel('Arb. Amplitude')
    
    # Real-time acquisition loop
    try:
        while True:
            array = V.getArray()
            array = array.flatten()
            x_data = range(0, len(array))
            ln1.set_data(x_data, array)
            plt.pause(0.001)
    except KeyboardInterrupt:
        print('Operation Interrupted.')
    # Device automatically closes when exiting the context
    print('Device Closed!')
```

### Real-Time Plotting with Simulated Data
```python
from VitesseAPI import Vitesse
import matplotlib.pyplot as plt

# Using context manager ensures device closes even on KeyboardInterrupt
with Vitesse().initialise(simulation=True) as V:
    
    # Configure parameters with method chaining
    V.setConfig(
        opFrequency=3.6e6,
        numCycles=2,
        channelsOnDrive=[1,0,0,0,0,0,0,0],
        channelsOnReceive=[1,0,0,0,0,0,0,0],
        numAverages=100,
        PRF=1000,
        recordLength=50e-6
    )
    
    # Setup plot
    fig, ax = plt.subplots()
    ln1, = plt.plot([], [])
    ax.set_xlim(0, V.recordPoints*V.numChannelsOnReceive)
    ax.set_ylim(-2048, 2048)
    ax.set_xlabel('Time')
    ax.set_ylabel('Arb. Amplitude')
    
    # Real-time acquisition loop
    try:
        while True:
            array = V.getArray()
            array = array.flatten()
            x_data = range(0, len(array))
            ln1.set_data(x_data, array)
            plt.pause(0.001)
    except KeyboardInterrupt:
        print('Operation Interrupted.')
    # Device automatically closes when exiting the context
    print('Device Closed!')
```

### Advanced Configuration with Method Chaining
```python
from VitesseAPI import initialiseVitesse

with initialiseVitesse() as V:
    # List available devices
    devices = V.listDevices()
    print(f"Available devices: {devices}")
    
    # Initialize and configure in a single chain
    V.setSymbol(7, 2) \
     .setChannelDrive([1, 1, 0, 0, 0, 0, 0, 0]) \
     .setAverages(500) \
     .setPrf(2000) \
     .setRecordLength(100e-6) \
     .setTriggerPhasing([0, 2.5, 0, 0, 0, 0, 0, 0]) \
     .setRecordDelay([0, 5, 0, 0, 0, 0, 0, 0])
    
    # Validate configuration
    V.checkValidity([0, 2.5, 0, 0, 0, 0, 0, 0], 
                    [0, 5, 0, 0, 0, 0, 0, 0], 
                    100e-6, 
                    2000)
    
    # Acquire data
    data = V.getArray()
    print(f"Data shape: {data.shape}")
```

### Error Handling with Context Manager
```python
from VitesseAPI import Vitesse

try:
    with Vitesse() as V:
        V.initialise()
        
        # This configuration might raise an error
        V.setConfig(
            PRF=10000,  # High PRF
            recordLength=200e-6,  # Long record
            channelsOnDrive=[1,1,1,1,1,1,1,1]  # All channels
        )
        
        data = V.getArray()
        
except ValueError as e:
    print(f"Configuration error: {e}")
except IOError as e:
    print(f"Device error: {e}")
# Device is guaranteed to be closed properly
```