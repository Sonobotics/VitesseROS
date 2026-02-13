import pathlib
import sys
import os
from matplotlib import pyplot as plt
from interfaces.msg import Ascan  # type: ignore
from rclpy.node import Node
import rclpy
import base64
import numpy as np

sys.path.append(
    f"{pathlib.Path(__file__).parent.resolve()}")
os.chdir(f"{pathlib.Path(__file__).parent.resolve()}")

from AscanProcessingModule import singularThicknessCalculation  # nopep8


# Acquisition Loop


class Processor(Node):

    def __init__(self):
        super().__init__('ascan_processor')

        # Declare ROS2 parameters for each channel (1-8)
        for channel in range(1, 9):
            self.declare_parameter(f'ch{channel}_firstPeak', False)
            self.declare_parameter(f'ch{channel}_multiEcho', False)
            self.declare_parameter(f'ch{channel}_zeroCrossing', False)
            self.declare_parameter(f'ch{channel}_temperatureCorrected', False)

            # Note: The parameters declared below should not be changed in
            # runtime. They are declared for read-only purpose.
            self.declare_parameter(
                f'ch{channel}_sampling_freq', 50 * (10 ** 6))
            self.declare_parameter(f'ch{channel}_lowbound_time', 4)
            self.declare_parameter(f'ch{channel}_minimum_thickness', 5.0)
            self.declare_parameter(f'ch{channel}_wave_velocity', 3260.0)
            self.declare_parameter(f'ch{channel}_num_cycles', 2)
            self.declare_parameter(
                f'ch{channel}_signal_frequency', int(3.6 * (10 ** 6)))
            self.declare_parameter(f'ch{channel}_threshold_snr', 8.0)
            self.declare_parameter(f'ch{channel}_noise_width', 28)
            self.declare_parameter(f'ch{channel}_calibration_index', 42)

        self.active_channels = []
        self.fig = None
        self.axes = []
        self.channel_plots = {}  # Store plot lines for each channel
        self.channel_peak_indices = {}  # Store peak lines for each channel
        self.subplot_initialized = False
        self.current_num_active = 0  # Track current number of active channels

        self.subscription = self.create_subscription(
            Ascan,
            'ascan',
            self.ascan_callback,
            10)

    def ascan_callback(self, msg):
        self.active_channels = msg.active_channels

        # Setup subplot arrangement based on number of active channels
        num_active = len(self.active_channels)
        if num_active == 0:
            return

        # Initialize subplots only once or when number of channels changes
        if not self.subplot_initialized or self.current_num_active != num_active:
            if self.fig is not None:
                plt.close(self.fig)  # Close existing figure

            if num_active == 1:
                self.fig, ax = plt.subplots(1, 1, figsize=(12, 6))
                self.axes = [ax]  # Make it a list for consistency
            else:
                rows = (num_active + 1) // 2  # 2 columns
                self.fig, axes_array = plt.subplots(
                    rows, 2, figsize=(15, 4*rows))
                if isinstance(axes_array, np.ndarray):
                    self.axes = axes_array.flatten()
                else:
                    self.axes = [axes_array]

            # Set up close event handler to exit program when user closes window
            def on_close(_event):
                print("Plot window closed by user. Exiting processor...")
                rclpy.shutdown()

            self.fig.canvas.mpl_connect('close_event', on_close)

            self.subplot_initialized = True
            self.current_num_active = num_active  # Update tracked number
            # Clear stored plot data since we have new subplots
            self.channel_plots.clear()
            self.channel_peak_indices.clear()
            print(f"Created subplots for {num_active} channels")

        # Skip plotting if figure doesn't exist
        if self.fig is None:
            return

        # Clear existing plots on each axis
        for ax in self.axes[:num_active]:
            ax.clear()

        # Process each active channel
        for i, channel_idx in enumerate(self.active_channels):
            if i >= len(msg.ascan_data):
                continue
            channel_idx = channel_idx + 1

            # Decode channel data
            rawData = base64.b64decode(msg.ascan_data[i])
            ascan: np.ndarray = np.frombuffer(rawData, dtype=np.float64)

            # Get parameters for this specific channel
            sampling_freq = self.get_parameter(
                f'ch{channel_idx}_sampling_freq').get_parameter_value().integer_value
            lowbound_time = self.get_parameter(
                f'ch{channel_idx}_lowbound_time').get_parameter_value().integer_value
            minimum_thickness = self.get_parameter(
                f'ch{channel_idx}_minimum_thickness').get_parameter_value().double_value
            wave_velocity = self.get_parameter(
                f'ch{channel_idx}_wave_velocity').get_parameter_value().double_value
            num_cycles = self.get_parameter(
                f'ch{channel_idx}_num_cycles').get_parameter_value().integer_value
            signal_frequency = self.get_parameter(
                f'ch{channel_idx}_signal_frequency').get_parameter_value().integer_value
            threshold_snr = self.get_parameter(
                f'ch{channel_idx}_threshold_snr').get_parameter_value().double_value
            noise_width = self.get_parameter(
                f'ch{channel_idx}_noise_width').get_parameter_value().integer_value
            calibration_index = self.get_parameter(
                f'ch{channel_idx}_calibration_index').get_parameter_value().integer_value

            firstPeak = self.get_parameter(
                f"ch{channel_idx}_firstPeak").get_parameter_value().bool_value
            multiEcho = self.get_parameter(
                f"ch{channel_idx}_multiEcho").get_parameter_value().bool_value
            zeroCrossing = self.get_parameter(
                f"ch{channel_idx}_zeroCrossing").get_parameter_value().bool_value
            temperatureCorrected = self.get_parameter(
                f"ch{channel_idx}_temperatureCorrected").get_parameter_value().bool_value

            if firstPeak and multiEcho:
                # If both are set, the processor behaves like none of these controls are set.
                # You may wish to change this to adapt to your needs.
                firstPeak, multiEcho, zeroCrossing, temperatureCorrected = False, False, False, False

            # Process the ascan data
            _hilbAscan, filtAscan, thickness_value, peak_indices, _attenuation = singularThicknessCalculation(
                ascan,
                samplingFrequency=sampling_freq,
                lowboundTime=lowbound_time,
                minimumThickness=minimum_thickness,
                waveVelocity=wave_velocity,
                numCycles=num_cycles,
                signalFrequency=signal_frequency,
                thresholdSNR=threshold_snr,
                noiseWidth=noise_width,
                calibrationIndex=calibration_index,
                temperature=float(msg.temperature),
                firstPeak=firstPeak,
                multiEcho=multiEcho,
                zeroCrossing=zeroCrossing,
                temperatureCorrected=temperatureCorrected
            )

            # Plot on the corresponding subplot
            ax = self.axes[i]
            x_data = [x / 50 for x in range(0, len(filtAscan))]
            ax.plot(x_data, filtAscan)

            # Handle case where peak_indices might be a single value or an array
            # We are dividing by 50 since the peaks are measured in indices instead of time
            # there are 50 indices per microsecond.
            if isinstance(peak_indices, (int, float, np.integer, np.floating)):
                # Single peak value
                ax.axvline(x=float(peak_indices) / 50, color="r")
            elif hasattr(peak_indices, '__iter__'):
                # Multiple peaks (list, array, etc.)
                for peak_x in peak_indices:
                    ax.axvline(x=float(peak_x) / 50, color="r")

            ax.set_xlim(0, len(filtAscan) / 50)
            ax.set_ylim(-2048, 2048)
            ax.set_xlabel('Time (μs)')
            ax.set_ylabel('Arb. Amplitude')

            # Handle thickness value which might be a single value or array
            if isinstance(thickness_value, (list, np.ndarray)) and len(thickness_value) > 0:
                # Take first value if array
                thickness_value = thickness_value[0]

            ax.set_title(
                f"Channel {channel_idx} - Thickness: {thickness_value:.2f} (mm)")

        # Hide unused subplots
        for j in range(num_active, len(self.axes)):
            self.axes[j].set_visible(False)

        plt.tight_layout()
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)

    ascanProcessor = Processor()

    rclpy.spin(ascanProcessor)

    ascanProcessor.destroy_node()


if __name__ == '__main__':
    main()
