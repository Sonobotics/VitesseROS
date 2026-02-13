import numpy as np
import math
from tabulate import tabulate
import struct
import time


def ext_temp(np_list):
    """
    Converts 2-byte RTD data [byte2, byte3] into Celsius temperature.
    Assumes PT100 and 3900 Ohm reference resistor.
    """
    if len(np_list) != 2:
        raise ValueError(
            "np_list must contain exactly two bytes: [byte2, byte3]")

    byte2, byte3 = np_list
    byte2 = int(byte2)
    byte3 = int(byte3)

    # Reconstruct 15-bit RTD ADC code
    data = (byte2 << 7) + (byte3 >> 1)

    # Convert ADC code to resistance
    ratio = data / 32768.0
    RREF = 3900  # ohms
    resistance = RREF * ratio

    # Callendar-Van Dusen inverse formula
    iCVD_A = 3.9083e-3
    iCVD_B = -5.775e-7
    PT100_NOMINAL = 100.0  # ohms

    Z1 = -iCVD_A
    Z2 = iCVD_A ** 2 - (4 * iCVD_B)
    Z3 = (4 * iCVD_B) / PT100_NOMINAL
    Z4 = 2 * iCVD_B
    try:
        temp = Z2 + (Z3 * resistance)
        temp = (math.sqrt(temp) + Z1) / Z4
    except:
        temp = 0
    return temp


def int_temp(np_list):
    """
    Convert two-byte FPGA/XADC output (from internal temperature register)
    into degrees Celsius. Expects np.uint8 array of [MSB, LSB] from do_out[15:0].

    Top 12 bits (do_out[15:4]) represent the temperature.

    Parameters:
    -----------
    np_list : np.ndarray of shape (2,), dtype=np.uint8
        The 16-bit XADC DRP readout, split into two bytes.

    Returns:
    --------
    float : temperature in degrees Celsius
    """
    if len(np_list) != 2 or np_list.dtype != np.uint8:
        raise ValueError("Input must be a NumPy array of two uint8 bytes.")

    # Combine into 16-bit word
    raw16 = (int(np_list[0]) << 8) | int(np_list[1])

    # Extract the top 12 bits (MSB-justified)
    temp_code = raw16 >> 4

    # Convert to °C using internal FPGA formula
    temperature_c = (temp_code * 503.975 / 4096.0) - 273.15

    return temperature_c


def dec_enc(bytes_array):
    """
    Decode a 4-byte array [MSB, ..., LSB] into an integer encoder count.
    Assumes LS7366R is in 32-bit mode, MSB-first SPI.
    """
    if len(bytes_array) != 4:
        raise ValueError("Expected exactly 4 bytes.")

    # Convert byte array to integer (MSB first)
    count = int.from_bytes(bytes_array, byteorder='big', signed=False)
    return count


def dec_enc_float(bytes_array):
    """
    Decode a 4-byte array [MSB, ..., LSB] into an integer encoder count.
    Assumes LS7366R is in 32-bit mode, MSB-first SPI.
    """
    if len(bytes_array) != 4:
        raise ValueError("Expected exactly 4 bytes.")

    # Convert byte array to integer (MSB first)
    count = int.from_bytes(bytes_array, byteorder='big', signed=False)

    return struct.unpack('>f', struct.pack('>I', count))[0]


def empty(bytes_array):
    return None


def float24_to_decimal(bin_str: str) -> float:
    """
    Converts a 24-bit binary string (Float24 format) to a decimal number.

    Float24 format: 1 sign bit, 7 exponent bits (bias 63), 16 mantissa bits.
    Input: bin_str — a string of 24 bits (e.g., "010011111110001001000000")

    Returns:
        Decimal float representation.
    """
    assert len(bin_str) == 24, "Input must be a 24-bit binary string."

    # Parse fields
    sign_bit = int(bin_str[0], 2)
    exponent_bits = int(bin_str[1:8], 2)
    fraction_bits = int(bin_str[8:], 2)

    bias = 63
    if exponent_bits == 0:
        # Subnormal number
        exponent = 1 - bias
        mantissa = fraction_bits / (1 << 16)
    else:
        exponent = exponent_bits - bias
        mantissa = 1 + (fraction_bits / (1 << 16))

    value = ((-1) ** sign_bit) * mantissa * (2 ** exponent)
    return value


def float16_to_decimal(bin_str: str) -> float:
    """
    Converts a 16-bit binary string (Float16 format) to a decimal number.

    Float16 format: 1 sign bit, 7 exponent bits (bias 63), 8 mantissa bits.
    Input: bin_str — a string of 16 bits (e.g., "0101111000000000")

    Returns:
        Decimal float representation.
    """
    assert len(bin_str) == 16, "Input must be a 16-bit binary string."

    # Parse fields
    sign_bit = int(bin_str[0], 2)
    exponent_bits = int(bin_str[1:8], 2)
    fraction_bits = int(bin_str[8:], 2)

    bias = 63
    if exponent_bits == 0:
        # Subnormal number
        exponent = 1 - bias
        mantissa = fraction_bits / (1 << 8)
    else:
        exponent = exponent_bits - bias
        mantissa = 1 + (fraction_bits / (1 << 8))

    value = ((-1) ** sign_bit) * mantissa * (2 ** exponent)
    return value


def decode_version_old(version_u16):
    # Split into 4 nibbles (4 bits each)
    n1 = (version_u16 >> 12) & 0xF
    n2 = (version_u16 >> 8) & 0xF
    n3 = (version_u16 >> 4) & 0xF
    n4 = version_u16 & 0xF

    return [n1, n2, n3, n4]


def decode_version_new(version_u16: int):
    # Extract fields
    major = (version_u16 >> 8) & 0xFF   # upper 8 bits
    minor = (version_u16 >> 4) & 0xF    # next 4 bits
    beta = version_u16 & 0xF           # lowest 4 bits

    return [major, minor, beta]


def getVersion(obj):
    Byte = obj.spiDevice.read(1)
    time.sleep(obj.READ_DELAY)
    ByteBack = np.frombuffer(Byte, dtype=np.uint8)
    print("\n\nVersion Control")
    version_u16 = obj.version_array
    version_display = ""
    for i, element in enumerate(version_u16):
        if i != len(version_u16)-1:
            version_display = version_display + str(element) + "."
        else:
            version_display = version_display + str(element)

    api_version_display = ""
    version_u16 = obj.APIVersion
    for i, element in enumerate(version_u16):
        if i != len(version_u16)-1:
            api_version_display = api_version_display + str(element) + "."
        else:
            api_version_display = api_version_display + str(element)

    rows = [
        ["Firmware version", f"{version_display}", ""],
        ["API Version", f"{api_version_display}", ""],
        ["Maximum Number of Channels", obj.maxChannels, ""]

    ]

    print(tabulate(rows, headers=["Parameter",
          "Value", "Extra"], tablefmt="grid"))


def getConfig(obj):
    print("\n\nCurrent Set Configuration Parameters")

    sensorArray = obj.sensorArray
    peripheralsOnArray = obj.peripheralsOnArray

    enabled_sensors = [
        sensor
        for sensor, enabled in zip(sensorArray, peripheralsOnArray)
        if enabled and sensor != "NA"
    ]

    rows = [
        ["Sensor Operating Frequency",
            f"{obj.opFrequency/1_000_000:.2f}", "MHz"],
        ["ADC Sampling Frequency",
            f"{int(obj.ADC_FREQ)/1_000_000:.2f}", "MHz"],
        ["Pulser Frequency", f"{obj.pulseFrequency/1_000_000:.2f}", "MHz"],
        ["Pulse Repetition Frequency (PRF)", obj.prf, "Hz"],
        ["Number of Averages", obj.numAverages, ""],
        ["Record Length", obj.recordLength*1e6, "us"],
        ["Additional Data", enabled_sensors],
        ["Data Sampling Mode", obj.samplingMode, "Bits"],
        ["Channels Receiving", f"{obj.enabledChannelDrivers}", ""],
        ["Channels Driving", f"{obj.enabledChannelReceive}", ""],
        ["Maximum Number of Channels", obj.maxChannels, ""],
    ]
    print(tabulate(rows, headers=["Parameter",
          "Value", "Units"], tablefmt="grid"))


def getEncoderConfig(obj):
    print("\n\nCurrent Enconder Parameters")

    rows = [
        ["Encoder CPR", f"{obj.encoder_cpr}", ""],
        ["Encoder Wheelbase", f"{obj.encoder_wheelbase}", "mm"],
        ["Encoder Radius", f"{obj.encoder_radius}", "mm"],
    ]
    print(tabulate(rows, headers=["Parameter",
          "Value", "Units"], tablefmt="grid"))
