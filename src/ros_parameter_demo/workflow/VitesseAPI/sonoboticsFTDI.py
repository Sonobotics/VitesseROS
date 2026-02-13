import ctypes
from ctypes import wintypes, create_string_buffer
import math
import platform
import os
import sys
import time
import subprocess
from typing import Union

# =========================== importing ftd2xx drivers ===========================


def _copy_dll_to_system32():
    """Copy ftd2xx.dll to Windows system32 folder with admin privileges"""
    if not sys.platform.startswith("win"):
        return True

    try:
        # Get paths
        source_dll = _get_resource_path("drivers/windows_FTDI/ftd2xx.dll")
        system32_path = os.path.join(os.environ['WINDIR'], 'System32')
        dest_dll = os.path.join(system32_path, 'ftd2xx.dll')

        # Check if DLL already exists in system32
        if os.path.exists(dest_dll):
            return True

        # Get the PowerShell script path
        ps_script = _get_resource_path("copy_ftdi_dll.ps1")

        # Create PowerShell command to run with elevation
        ps_command = [
            "powershell.exe",
            "-ExecutionPolicy", "Bypass",
            "-WindowStyle", "Hidden",
            "-Command",
            f"Start-Process powershell.exe -ArgumentList '-ExecutionPolicy Bypass -File \"{ps_script}\" -SourcePath \"{source_dll}\" -DestPath \"{dest_dll}\"' -Verb RunAs -Wait"
        ]

        # Run the PowerShell command
        result = subprocess.run(ps_command, capture_output=True, text=True)

        # Check if the file was successfully copied
        if os.path.exists(dest_dll):
            return True
        else:
            if result.stderr:
                pass
            return False

    except Exception as e:
        return False


def _get_resource_path(relative_path: str):
    """Get path to resource file"""
    package = __package__ or __name__.split('.')[0]

    # Nuitka onefile compatibility
    if hasattr(sys.modules[__name__], '__compiled__'):
        return os.path.join(os.path.dirname(sys.argv[0]), package, relative_path)

    try:
        import importlib.resources
        resources = importlib.resources.files(package)
        with importlib.resources.as_file(resources.joinpath(relative_path)) as path:
            return path
    except:
        # Fallback for Python < 3.9 where importlib.resources is not available
        import pkg_resources  # type: ignore # nopep8
        return pkg_resources.resource_filename(package, relative_path)  # type: ignore # nopep8

# Dynamically loads the external C++ Library for interacting with the FTDI device, depending on the platform.
# Returns reference to the library.


def _setup_driver():

    if sys.platform.startswith("win"):
        # Copy ftd2xx.dll to system32 before loading the main library
        try:
            lib = ctypes.CDLL(_get_resource_path("libraries/ftdiHandler.dll"))
        except FileNotFoundError:
            if not _copy_dll_to_system32():
                raise FileExistsError("Error copying ftd2xx.dll over")
            lib = ctypes.CDLL(_get_resource_path("libraries/ftdiHandler.dll"))

    elif platform.machine() == 'x86_64':
        lib = ctypes.CDLL(_get_resource_path("libraries/ftdiHandler64.so"))
        os.system('sudo rmmod ftdi_sio 2>/dev/null')
    elif platform.machine() == 'aarch64':
        lib = ctypes.CDLL(_get_resource_path("libraries/ftdiHandler.so"))
        # Always sanitize the file first by replacing CRLF with LF
        os.system(
            f"sed -i -e 's/\r$//' {_get_resource_path('drivers/linux_arm_FTDI/unload_ftdi.sh')}")
        subprocess.run(["sudo", "bash", _get_resource_path(
            "drivers/linux_arm_FTDI/unload_ftdi.sh")], check=True)
        os.system('sudo rmmod ftdi_sio 2>/dev/null')
    else:
        raise OSError('Incompatible with this operating system!')

    # =========================== c++ function setup ===========================

    lib.uartRead.restype = ctypes.c_int
    lib.uartRead.argtypes = [ctypes.c_void_p,
                             wintypes.DWORD, ctypes.POINTER(ctypes.c_char)]
    lib.spiRead.restype = ctypes.c_int
    lib.spiRead.argtypes = [ctypes.c_void_p,
                            ctypes.c_uint32, ctypes.POINTER(ctypes.c_char)]
    lib.uartWrite.argtypes = [ctypes.c_void_p, ctypes.c_char_p, wintypes.DWORD]
    lib.uartWrite.restype = ctypes.c_int
    lib.spiWrite.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint8]
    lib.spiWrite.restype = ctypes.c_int
    lib.connect_device.argtypes = [
        ctypes.c_char_p, ctypes.POINTER(ctypes.c_void_p)]
    lib.connect_device.restype = ctypes.c_int
    lib.connect_device_num.argtypes = [
        ctypes.c_int, ctypes.POINTER(ctypes.c_void_p)]
    lib.connect_device_num.restype = ctypes.c_int
    lib.set_baud_rate.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.set_baud_rate.restype = ctypes.c_int
    lib.read_eeprom.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_char), ctypes.POINTER(
        ctypes.c_char), ctypes.POINTER(ctypes.c_char), ctypes.POINTER(ctypes.c_char)]
    lib.read_eeprom.restype = ctypes.c_int
    lib.dump_eeprom.argtypes = [ctypes.c_void_p]
    lib.dump_eeprom.restype = ctypes.c_int
    lib.write_eeprom.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_char), ctypes.POINTER(
        ctypes.c_char), ctypes.POINTER(ctypes.c_char), ctypes.POINTER(ctypes.c_char)]
    lib.write_eeprom.restype = ctypes.c_int
    lib.configureSPI.argtypes = [ctypes.c_void_p]
    lib.freeReadBuffer.argtypes = [ctypes.c_char_p]
    lib.close.restype = ctypes.c_int
    lib.setTimeouts.restype = ctypes.c_int
    lib.setUSBParameters.restype = ctypes.c_int

    return lib

# ============================== error status messages ==============================


STATUS_MESSAGES = {
    0: "Command successful",
    1: "Handle passed is invalid",
    2: "Device, or serial number, not found",
    3: "Handle used wasn't successfully opened",
    4: "I/O operation failed",
    5: "Memory allocation failed",
    6: "Invalid arguments parsed",
    7: "Invalid baud rate specified",
    8: "Tried to erase EEPROM without opening it for erase access",
    9: "Tried to write EEPROM without proper access",
    10: "Write to EEPROM or device register failed",
    11: "Could not read from EEPROM",
    12: "Could not write to EEPROM",
    13: "EEPROM erase operation failed",
    14: "Device lacks EEPROM",
    15: "EEPROM exists but is blank",
    16: "One or more function arguments are not valid",
    17: "Operation or feature not supported by the device",
    18: "A non-specific or undocumented error occurred"
}

# =========================== library classes and methods ===========================

# returns number of connected FTDI devices


def getNumDevices():
    num = _setup_driver().get_num_devices()
    if num == -1:
        raise Exception("Error getting number of devices")
    else:
        return num

# ftdi channel object


class ftdiChannel:
    """
    Template for FTDI/SPI channel used by Vitesse.
    """

    def __init__(self):
        self._closed = False

    def write(self, data: bytes) -> None:
        if self._closed:
            raise IOError("Simulated device is closed.")

    def read(self, numBytes: int) -> bytes:
        if self._closed:
            raise IOError("Simulated device is closed.")
        return bytes()

    def readEEPROM(self):
        return {
            "Manufacturer": "Sonobotics",
            "Serial Number": "SON-082001",
            "Device": "1CH SONUS Vitesse",
        }

    def close(self) -> None:
        self._closed = True


class sonoboticsFtdiChannel(ftdiChannel):
    '''
    The actual implementation of the FTDI interface.
    '''
    # constructor
    # PROTOCOL: SPI/UART
    # CONNMODE: "serialNum"/"deviceNum"
    # CONNID: the device's serial number or device number
    errorCounter: list[float] = []

    def __init__(self, protocol: str, connMode: str, connID: Union[int, bytes]):
        self.protocol = protocol
        self.connID = connID
        self.connMode = connMode
        self.lib = _setup_driver()

        # gets the devices ftHandle
        if connMode == "serialNum":
            ftHandle = ctypes.c_void_p()
            return_code = self.lib.connect_device(
                ctypes.c_char_p(connID), ctypes.byref(ftHandle))
            if return_code != 0:
                error_msg = STATUS_MESSAGES.get(
                    return_code, f"Unknown status code: {return_code}")
                raise Exception(f"Couldn't connect to device ({error_msg})")
            self.ftHandle = ftHandle.value

        elif connMode == "deviceNum":
            ftHandle = ctypes.c_void_p()
            return_code = self.lib.connect_device_num(
                connID, ctypes.byref(ftHandle))
            if return_code != 0:
                error_msg = STATUS_MESSAGES.get(
                    return_code, f"Unknown status code: {return_code}")
                raise Exception(f"Couldn't connect to device ({error_msg})")
            self.ftHandle = ftHandle.value
        else:
            raise Exception("Invalid connection mode")

        # configures the ftdi device to use SPI (if required)
        if protocol == "SPI":
            return_code = self.lib.configureSPI(ctypes.c_void_p(self.ftHandle))
            if return_code != 0:
                error_msg = STATUS_MESSAGES.get(
                    return_code, f"Unknown status code: {return_code}")
                raise Exception(f"Can't configure device ({error_msg})")

        elif protocol != "UART":
            raise Exception("Invalid connection protocol")

    # writes bytes, bytearray, int or str to device

    def write(self, data: Union[bytes, bytearray, int, str]):
        if self.protocol == "UART":
            if isinstance(data, (bytes, bytearray)):
                return_code = self.lib.uartWrite(ctypes.c_void_p(
                    self.ftHandle), ctypes.c_char_p(data), wintypes.DWORD(len(data)))
            elif isinstance(data, (int)):
                byteLength = math.ceil(data.bit_length() / 8)
                return_code = self.lib.uartWrite(ctypes.c_void_p(self.ftHandle), ctypes.c_char_p(
                    data.to_bytes(byteLength, 'big')), wintypes.DWORD(byteLength))
            elif isinstance(data, (str)):
                return_code = self.lib.uartWrite(ctypes.c_void_p(self.ftHandle), ctypes.c_char_p(
                    data.encode("utf-8")), wintypes.DWORD(len(data)))
            else:
                raise TypeError(f"Cannot intepret type of input data")

            if return_code != 0:
                error_msg = STATUS_MESSAGES.get(
                    return_code, f"Unknown status code: {return_code}")
                raise Exception(f"Can't write to device ({error_msg})")

        elif self.protocol == "SPI":
            if isinstance(data, (bytes, bytearray)):
                return_code = self.lib.spiWrite(ctypes.c_void_p(
                    self.ftHandle), ctypes.c_char_p(data), len(data))
            elif isinstance(data, (int)):
                byteLength = math.ceil(data.bit_length() / 8)
                return_code = self.lib.spiWrite(ctypes.c_void_p(self.ftHandle), ctypes.c_char_p(
                    data.to_bytes(byteLength, 'big')), byteLength)
            elif isinstance(data, (str)):
                return_code = self.lib.spiWrite(ctypes.c_void_p(
                    self.ftHandle), ctypes.c_char_p(data.encode("utf-8")), len(data))
            else:
                raise TypeError(f"Cannot intepret type of input data")

            if return_code != 0:
                error_msg = STATUS_MESSAGES.get(
                    return_code, f"Unknown status code: {return_code}")
                raise Exception(f"Can't write to device ({error_msg})")

    # reads data from device and returns as a byte array

    def read(self, numBytes: int):
        if self.protocol == "UART":
            # Allocate buffer correctly
            data = (ctypes.c_char * numBytes)()  # Creates an array of c_ubyte

            # Get a proper pointer type
            data_ptr = ctypes.cast(data, ctypes.POINTER(ctypes.c_char))

            # Call spiRead with the correct pointer type
            return_code = self.lib.uartRead(ctypes.c_void_p(
                self.ftHandle), wintypes.DWORD(numBytes), data_ptr)

            if return_code != 0:
                error_msg = STATUS_MESSAGES.get(
                    return_code, f"Unknown status code: {return_code}")
                raise Exception(f"Can't read from device ({error_msg})")

            # Convert the buffer to a Python bytearray
            py_bytes = bytearray(data.raw)

        elif self.protocol == "SPI":
            # Allocate buffer correctly
            data = (ctypes.c_char * numBytes)()  # Creates an array of c_ubyte

            # Get a proper pointer type
            data_ptr = ctypes.cast(data, ctypes.POINTER(ctypes.c_char))

            # Call spiRead with the correct pointer type
            return_code = self.lib.spiRead(ctypes.c_void_p(
                self.ftHandle), numBytes, data_ptr)

            # While error detected
            # This ensures no garbage data is leaked
            while return_code != 0:
                # Counting the errors, if > 10 times in 1 hour, raise error
                errorTimestamp = time.time()
                self.errorCounter.append(errorTimestamp)
                self.errorCounter = self.errorCounter[-10:]

                if (len(self.errorCounter) >= 10 and time.time() - self.errorCounter[0] < 3600):
                    error_msg = STATUS_MESSAGES.get(
                        return_code, f"Unknown status code: {return_code}")
                    raise Exception(
                        f"Repetitive read error occured from device ({error_msg})")

                # Try reloading driver and reconnecting SPI device
                MAX_RETRIES = 20
                for i in range(MAX_RETRIES):
                    try:
                        self.close()
                        self.__init__(
                            self.protocol, self.connMode, self.connID)
                        break
                    except:
                        time.sleep(0.1)
                        # If cannot connect to device in 20 tries, raise error
                        if i == MAX_RETRIES - 1:
                            error_msg = STATUS_MESSAGES.get(
                                return_code, f"Unknown status code: {return_code}")
                            raise Exception(
                                f"Can't read from device ({error_msg})")

                # Try reading again
                return_code = self.lib.spiRead(ctypes.c_void_p(
                    self.ftHandle), numBytes, data_ptr)

            # Convert the buffer to a Python bytearray
            py_bytes = bytearray(data.raw)

        else:
            raise ValueError("Uninterpretable self.protocol")

        return py_bytes

    def readEEPROM(self):

        # Assume you already have an FT_HANDLE from your connect function
        ft_handle = ctypes.c_void_p(self.ftHandle)  # from your earlier context

        # Allocate output buffers
        manufacturer_buf = create_string_buffer(32)
        manufacturerId_buf = create_string_buffer(16)
        description_buf = create_string_buffer(64)
        serialNumber_buf = create_string_buffer(16)

        # Call the function
        return_code = self.lib.read_eeprom(
            ft_handle,
            manufacturer_buf,
            manufacturerId_buf,
            description_buf,
            serialNumber_buf
        )

        # Check result
        if return_code != 0:
            error_msg = STATUS_MESSAGES.get(
                return_code, f"Unknown status code: {return_code}")
            raise Exception(f"EEPROM read failed ({error_msg})")

        # Convert to Python strings
        manufacturer: str = manufacturer_buf.value.decode('utf-8')
        manufacturer_id: str = manufacturerId_buf.value.decode('utf-8')
        description: str = description_buf.value.decode('utf-8')
        serial_number: str = serialNumber_buf.value.decode('utf-8')

        return {"Manufacturer": manufacturer, "Manufacturer ID": manufacturer_id, "Device": description, "Serial Number": serial_number}

    def writeEEPROM(self, manufacturer: str, manufacturer_id: str, description: str, serial_number: str):
        # Ensure inputs are encoded as bytes (c_char_p)
        manufacturer_b = manufacturer.encode('utf-8')
        manufacturer_id_b = manufacturer_id.encode('utf-8')
        description_b = description.encode('utf-8')
        serial_number_b = serial_number.encode('utf-8')

    # Call the C function
        return_code = self.lib.write_eeprom(
            ctypes.c_void_p(self.ftHandle),
            manufacturer_b,
            manufacturer_id_b,
            description_b,
            serial_number_b
        )

        if return_code != 0:
            error_msg = STATUS_MESSAGES.get(
                return_code, f"Unknown status code: {return_code}")
            raise Exception(f"EEPROM write failed ({error_msg})")

        # print("EEPROM write successful")

    def dumpEEPROM(self):
        return_code = self.lib.dump_eeprom(ctypes.c_void_p(self.ftHandle))
        if return_code != 0:
            error_msg = STATUS_MESSAGES.get(
                return_code, f"Unknown status code: {return_code}")
            raise Exception(f"Error dumping EEPROM ({error_msg})")

    # configure device timouts
    def setTimeouts(self, readTimeOut: int, writeTimeOut: int):
        return_code = self.lib.setTimeouts(ctypes.c_void_p(self.ftHandle), wintypes.DWORD(
            readTimeOut), wintypes.DWORD(writeTimeOut))
        if return_code != 0:
            error_msg = STATUS_MESSAGES.get(
                return_code, f"Unknown status code: {return_code}")
            raise Exception(f"Error setting timeouts ({error_msg})")

    # configure device USB parameters
    def setUSBParameters(self, inTransferSize: int, outTransferSize: int):
        return_code = self.lib.setUSBParameters(ctypes.c_void_p(
            self.ftHandle), wintypes.DWORD(inTransferSize), wintypes.DWORD(outTransferSize))
        if return_code != 0:
            error_msg = STATUS_MESSAGES.get(
                return_code, f"Unknown status code: {return_code}")
            raise Exception(f"Error setting USB parameters ({error_msg})")

    # configure device latency timer
    def setLatencyTimer(self, timer: int):
        return_code = self.lib.setLatencyTimer(
            ctypes.c_void_p(self.ftHandle), wintypes.CHAR(timer))
        if return_code != 0:
            error_msg = STATUS_MESSAGES.get(
                return_code, f"Unknown status code: {return_code}")
            raise Exception(f"Error setting latency timer ({error_msg})")

    # configure device baud rate
    def setBaudRate(self, baudRate: int):
        return_code = self.lib.set_baud_rate(
            ctypes.c_void_p(self.ftHandle), baudRate)
        if return_code != 0:
            error_msg = STATUS_MESSAGES.get(
                return_code, f"Unknown status code: {return_code}")
            raise Exception(f"Error setting baud rate EEPROM ({error_msg})")

    # close connection to device
    def close(self):
        return_code = self.lib.close(ctypes.c_void_p(self.ftHandle))
        if return_code != 0:
            error_msg = STATUS_MESSAGES.get(
                return_code, f"Unknown status code: {return_code}")
            raise Exception(f"Error closing device ({error_msg})")
