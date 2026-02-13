import signal
from interfaces.msg import Ascan  # type: ignore
import rclpy
from rclpy.node import Node
import rclpy.parameter
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter_event_handler import ParameterEventHandler
import base64

import os
import sys
import pathlib

sys.path.append(
    f"{pathlib.Path(__file__).parent.resolve()}")
os.chdir(f"{pathlib.Path(__file__).parent.resolve()}")

from VitesseAPI import Vitesse  # nopep8


class Publisher(Node):

    def __init__(self):
        '''
        Initialisation
        '''
        super().__init__('ascan_publisher')

        # Signal Parameters

        # Averages Range: 1 to 1000
        self.declare_parameter('numAverages', 100)

        # Rest of the parameters should be read only
        parameterDescriptor = ParameterDescriptor()
        parameterDescriptor.read_only = True

        # Cycles Range: 1 to 3
        self.declare_parameter('numCycles', 2, descriptor=parameterDescriptor)
        # Record Length Range: 0 to 100 us (8 CH), 0 to 200 us (4 CH), 0 to 800 us (8 CH)
        self.declare_parameter('recordLength', 20e-6,
                               descriptor=parameterDescriptor)
        # PRF Range: 1 to 5000 Hz
        self.declare_parameter('PRF', 1000, descriptor=parameterDescriptor)
        # Channels on e.g. [Channel 1 (on/off), Channel 2(on/off), etc.]
        self.declare_parameter('channelsOnReceive', [
                               1, 0, 0, 0, 0, 0, 0, 0], descriptor=parameterDescriptor)
        # Phasing in microseconds for each channel e.g. [Channel 1 Phase (us), Channel 2 Phase (us), etc.]
        self.declare_parameter('phaseArrayMicro', [
                               0, 0, 0, 0, 0, 0, 0, 0], descriptor=parameterDescriptor)
        # Delay in microseconds for each channel e.g. [Channel 1 Delay (us), Channel 2 Delay (us), etc.]
        self.declare_parameter('delayArrayMicro', [
                               0, 0, 0, 0, 0, 0, 0, 0], descriptor=parameterDescriptor)

        # New parameters for updated VitesseAPI
        # Channels on driver array (which channels to drive)
        self.declare_parameter('channelsOnDrive', [
                               1, 0, 0, 0, 0, 0, 0, 0], descriptor=parameterDescriptor)
        # Peripherals on array (which peripherals are enabled)
        self.declare_parameter('peripheralsOnArray', [
                               0, 0, 0, 0, 0, 0, 0, 0], descriptor=parameterDescriptor)
        # Sampling mode (16 or 24 bit)
        self.declare_parameter(
            'samplingMode', 24, descriptor=parameterDescriptor)
        # Pulse frequency in Hz
        self.declare_parameter('pulseFrequency', int(
            200e6), descriptor=parameterDescriptor)
        # Operation frequency in Hz
        self.declare_parameter('opFrequency', int(
            3.6e6), descriptor=parameterDescriptor)

        # Finally, initialise vitesse and create publisher
        self.initialise_vitesse()

        # Register average change callback
        self.handler = ParameterEventHandler(self)
        self.callback_handle = self.handler.add_parameter_callback(
            parameter_name="numAverages",
            node_name="ascan_publisher",
            callback=self.averageChangeCallback,
        )

        self.publisher = self.create_publisher(Ascan, 'ascan', 10)

        # Create timer to publish data periodically
        timer_period = 1.0 / \
            self.get_parameter('PRF').get_parameter_value().integer_value
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def averageChangeCallback(self, p: rclpy.parameter.Parameter):
        if p.value:
            value = rclpy.parameter.parameter_value_to_python(p.value)
            if isinstance(value, int):
                self.V.setAverages(value)

    def initialise_vitesse(self):
        # Get all parameters for setConfig according to new API
        numCycles = self.get_parameter(
            'numCycles').get_parameter_value().integer_value
        channelsOnReceive = list(self.get_parameter(
            'channelsOnReceive').get_parameter_value().integer_array_value)
        channelsOnDrive = list(self.get_parameter(
            'channelsOnDrive').get_parameter_value().integer_array_value)
        PRF = self.get_parameter('PRF').get_parameter_value().integer_value
        numAverages = self.get_parameter(
            'numAverages').get_parameter_value().integer_value
        recordLength = self.get_parameter(
            'recordLength').get_parameter_value().double_value
        phaseArrayMicro = list(self.get_parameter(
            'phaseArrayMicro').get_parameter_value().integer_array_value)
        delayArrayMicro = list(self.get_parameter(
            'delayArrayMicro').get_parameter_value().integer_array_value)
        peripheralsOnArray = list(self.get_parameter(
            'peripheralsOnArray').get_parameter_value().integer_array_value)
        samplingMode = self.get_parameter(
            'samplingMode').get_parameter_value().integer_value
        pulseFrequency = self.get_parameter(
            'pulseFrequency').get_parameter_value().integer_value
        opFrequency = self.get_parameter(
            'opFrequency').get_parameter_value().integer_value

        self.V = Vitesse().initialise().setConfig(
            numCycles=numCycles,
            channelsOnReceive=channelsOnReceive,
            channelsOnDrive=channelsOnDrive,
            PRF=PRF,
            numAverages=numAverages,
            recordLength=recordLength,
            phaseArrayMicro=phaseArrayMicro,
            delayArrayMicro=delayArrayMicro,
            peripheralsOnArray=peripheralsOnArray,
            samplingMode=samplingMode,
            pulseFrequency=pulseFrequency,
            opFrequency=opFrequency
        )
        print('Initialised Vitesse!')

    def timer_callback(self):
        self.fetch_ascan()

    def fetch_ascan(self):
        array = self.V.getArray()

        # Get active channel indices from channelsOnReceive
        channels_on_array = self.get_parameter(
            'channelsOnReceive').get_parameter_value().integer_array_value
        active_channels = [i for i, is_active in enumerate(
            channels_on_array) if is_active]

        # Split array data by channels - array corresponds to active channels only
        ascan_data_list = []
        if len(array.shape) == 2:  # If array is 2D [active_channels, points]
            for i in range(len(active_channels)):
                channel_data = array[i].flatten()
                encoded_data = base64.b64encode(channel_data).decode('ascii')
                ascan_data_list.append(encoded_data)
        else:  # If array is 1D, split evenly among active channels
            points_per_channel = len(array) // self.V.numChannelsOn
            for i in range(len(active_channels)):
                start_idx = i * points_per_channel
                end_idx = (i + 1) * points_per_channel
                channel_data = array[start_idx:end_idx]
                encoded_data = base64.b64encode(channel_data).decode('ascii')
                ascan_data_list.append(encoded_data)

        msg = Ascan()
        msg.active_channels = active_channels
        msg.ascan_data = ascan_data_list
        ### GET YOUR TEMPERATURE HERE ###
        msg.temperature = 0.0
        ### GET YOUR TEMPERATURE HERE ###
        self.publisher.publish(msg)

    def shutdown_vitesse(self):
        self.V.closeDevice()


signal.signal(signal.SIGINT, 0)


def main(args=None):
    rclpy.init(args=args)

    ascanPublisher = Publisher()

    try:
        rclpy.spin(ascanPublisher)
    except KeyboardInterrupt:
        pass
    finally:
        ascanPublisher.shutdown_vitesse()
        ascanPublisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
