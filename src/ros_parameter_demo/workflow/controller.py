#!/usr/bin/env python3

import sys
import signal
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import ListParameters, GetParameters, SetParameters, DescribeParameters
from rcl_interfaces.msg import ParameterValue, ParameterType, Parameter as ParameterMsg
import threading
from typing import Dict, Any, Optional
import json
import os
import re
import time

from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout,
                             QWidget, QLabel, QSlider,
                             QLineEdit, QPushButton, QSpinBox, QDoubleSpinBox,
                             QCheckBox, QScrollArea,
                             QProgressBar, QTabWidget)
from PyQt5.QtCore import Qt, pyqtSignal, QThread, QTimer
from PyQt5.QtGui import QFont
import concurrent.futures


class ParameterDiscovery(QThread):
    """Optimized parameter discovery in separate thread"""
    nodes_updated = pyqtSignal(dict)

    def __init__(self, ros_node: Node):
        super().__init__()
        self.ros_node = ros_node
        self.running = True
        self.last_nodes = set()
        self.service_clients = {}  # Cache service clients
        self.discovery_interval = 3.0  # Slower discovery when no nodes

    def run(self):
        """Main discovery loop in separate thread"""
        while self.running:
            try:
                self.discover_nodes()
                # Use interruptible sleep
                for _ in range(int(self.discovery_interval * 10)):
                    if not self.running:
                        break
                    self.msleep(100)
            except KeyboardInterrupt:
                print("Discovery thread interrupted")
                self.running = False
                break
            except Exception as e:
                print(f"Discovery error: {e}")
                # Use interruptible sleep for error case
                for _ in range(50):
                    if not self.running:
                        break
                    self.msleep(100)

    def stop(self):
        self.running = False
        self.wait()

    def discover_nodes(self):
        """Optimized node discovery with caching"""
        try:
            current_nodes = set()
            for name, ns in self.ros_node.get_node_names_and_namespaces():
                full = (ns.rstrip('/') + '/' + name) if ns and ns != '/' else name
                full = full.lstrip('/')
                current_nodes.add(full)

            # Skip if no nodes changed
            if current_nodes == self.last_nodes and len(current_nodes) > 1:
                return

            self.last_nodes = current_nodes.copy()

            # Adjust discovery interval based on node count
            if len(current_nodes) <= 1:  # Only self
                self.discovery_interval = 5.0
            else:
                self.discovery_interval = 2.0

            node_params = {}

            # Use thread pool for parallel parameter fetching
            with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
                future_to_node = {}

                for node_name in current_nodes:
                    if node_name == self.ros_node.get_name():
                        continue
                    future = executor.submit(
                        self._get_node_parameters, node_name)
                    future_to_node[future] = node_name

                for future in concurrent.futures.as_completed(future_to_node, timeout=5.0):
                    node_name = future_to_node[future]
                    try:
                        params = future.result()
                        if params:
                            node_params[node_name] = params
                    except Exception as e:
                        print(f"Failed to get parameters for {node_name}: {e}")

            if node_params:
                self.nodes_updated.emit(node_params)

        except Exception as e:
            print(f"Error during node discovery: {e}")

    def _get_node_parameters(self, node_name: str) -> Dict[str, Dict]:
        """Optimized parameter fetching with client caching"""
        try:
            # Get or create cached clients
            if node_name not in self.service_clients:
                list_client = self.ros_node.create_client(
                    ListParameters, f'/{node_name}/list_parameters')
                get_client = self.ros_node.create_client(
                    GetParameters, f'/{node_name}/get_parameters')
                describe_client = self.ros_node.create_client(
                    DescribeParameters, f'/{node_name}/describe_parameters')
                self.service_clients[node_name] = (
                    list_client, get_client, describe_client)
            else:
                list_client, get_client, describe_client = self.service_clients[node_name]

            # Quick service availability check
            if not list_client.wait_for_service(timeout_sec=0.5):
                return {}
            if not get_client.wait_for_service(timeout_sec=0.5):
                return {}
            if not describe_client.wait_for_service(timeout_sec=0.5):
                return {}

            # List parameters with shorter timeout
            list_request = ListParameters.Request()
            list_future = list_client.call_async(list_request)
            while not list_future.done():
                time.sleep(0.01)

            list_result = list_future.result()
            if not list_result:
                return {}

            param_names = list_result.result.names
            if not param_names:
                return {}

            # Get parameter values with shorter timeout
            get_request = GetParameters.Request()
            get_request.names = param_names
            get_future = get_client.call_async(get_request)
            while not get_future.done():
                time.sleep(0.01)

            get_result = get_future.result()
            if not get_result:
                return {}

            param_values = get_result.values

            # Get parameter descriptors for read-only status
            describe_request = DescribeParameters.Request()
            describe_request.names = param_names
            describe_future = describe_client.call_async(describe_request)
            while not describe_future.done():
                time.sleep(0.01)

            descriptors = []
            describe_result = describe_future.result()
            if describe_result:
                descriptors = describe_result.descriptors

            # Build parameter dictionary with descriptors
            parameters = {}
            for i, (name, value) in enumerate(zip(param_names, param_values)):
                descriptor = descriptors[i] if i < len(descriptors) else None
                param_info = {
                    'value': self._parameter_value_to_python(value),
                    'type': value.type,
                    'descriptor': descriptor,
                    'read_only': descriptor.read_only if descriptor else False
                }
                parameters[name] = param_info

            return parameters

        except Exception as e:
            print(f"Error getting parameters for {node_name}: {e}")
            return {}

    def _parameter_value_to_python(self, param_value: ParameterValue):
        """Convert ROS2 ParameterValue to Python value"""
        if param_value.type == ParameterType.PARAMETER_BOOL:
            return param_value.bool_value
        elif param_value.type == ParameterType.PARAMETER_INTEGER:
            return param_value.integer_value
        elif param_value.type == ParameterType.PARAMETER_DOUBLE:
            return param_value.double_value
        elif param_value.type == ParameterType.PARAMETER_STRING:
            return param_value.string_value
        elif param_value.type == ParameterType.PARAMETER_BYTE_ARRAY:
            return list(param_value.byte_array_value)
        elif param_value.type == ParameterType.PARAMETER_BOOL_ARRAY:
            return list(param_value.bool_array_value)
        elif param_value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
            return list(param_value.integer_array_value)
        elif param_value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return list(param_value.double_array_value)
        elif param_value.type == ParameterType.PARAMETER_STRING_ARRAY:
            return list(param_value.string_array_value)
        else:
            return None


class ParameterWidget(QWidget):
    """Widget for editing a single parameter"""

    # node_name, param_name, value
    parameter_changed = pyqtSignal(str, str, object)

    def __init__(self, param_name: str, param_info: Dict, node_name: str, gui_parent, config_bounds: Optional[Dict] = None, display_names: Optional[Dict] = None, parent=None):
        super().__init__(parent)
        self.param_name = param_name
        self.param_info = param_info
        self.param_type = param_info['type']
        self.original_value = param_info['value']
        self.current_value = param_info['value']
        self.is_read_only = param_info.get('read_only', False)
        self.has_been_modified = False
        self.node_name = node_name
        self.config_bounds = config_bounds
        self.display_names = display_names
        self.gui_parent = gui_parent  # Reference to ParameterManagerGUI for pattern matching

        # Timer for real-time updates
        self.update_timer = QTimer()
        self.update_timer.setSingleShot(True)
        self.update_timer.timeout.connect(self._emit_parameter_change)
        self.update_delay = 500  # 500ms delay

        self.setup_ui()

    def _get_bounds(self):
        """Get bounds for this parameter from config"""
        if not self.config_bounds:
            return None, None

        # Use pattern matching to find bounds
        bounds_config = self.gui_parent._get_pattern_config(
            self.node_name, self.param_name, self.config_bounds)

        if bounds_config and isinstance(bounds_config, dict):
            return bounds_config.get('min'), bounds_config.get('max')

        return None, None

    def _enforce_bounds(self, value):
        """Enforce bounds on parameter value"""
        min_val, max_val = self._get_bounds()

        if min_val is not None and value < min_val:
            return min_val
        if max_val is not None and value > max_val:
            return max_val

        return value

    def _get_display_name(self) -> str:
        """Get display name for this parameter, with override from config"""
        if self.display_names:
            # Use pattern matching to find display name
            display_name = self.gui_parent._get_pattern_config(
                self.node_name, self.param_name, self.display_names)
            if display_name:
                return display_name
        return self.param_name

    def setup_ui(self):
        layout = QHBoxLayout()
        layout.setContentsMargins(5, 1, 5, 1)

        # Set fixed height for tighter spacing
        self.setFixedHeight(35)

        # Parameter name label with read-only indicator and custom display name
        display_name = self._get_display_name()
        param_display = display_name + \
            (" (READ ONLY)" if self.is_read_only else "")
        name_label = QLabel(param_display)
        name_label.setMinimumWidth(200)
        name_label.setFont(QFont("Arial", 9))

        # Style read-only parameters
        if self.is_read_only:
            name_label.setStyleSheet(
                "QLabel { color: #888; font-style: italic; }")

        layout.addWidget(name_label)

        # Create appropriate input widget based on parameter type
        self.input_widgets = []

        if self.param_type == ParameterType.PARAMETER_BOOL:
            # Add stretch to push checkbox to the right
            layout.addStretch()

            widget = QCheckBox()
            widget.setChecked(self.original_value)
            widget.stateChanged.connect(self._on_bool_changed)
            if self.is_read_only:
                widget.setEnabled(False)
                widget.setStyleSheet("QCheckBox { color: #888; }")
            self.input_widgets.append(widget)
            layout.addWidget(widget)

        elif self.param_type == ParameterType.PARAMETER_INTEGER:
            # Get bounds from config
            min_bound, max_bound = self._get_bounds()

            # Slider with bounds-aware range
            slider = QSlider(Qt.Orientation.Horizontal)

            if min_bound is not None and max_bound is not None:
                slider_min = int(min_bound)
                slider_max = int(max_bound)
            else:
                # Fallback to adaptive range
                range_factor = abs(self.original_value) * 10
                slider_min = 0
                slider_max = min(
                    2147483647, self.original_value + range_factor)

            slider.setRange(slider_min, slider_max)
            slider.setValue(self.original_value)
            slider.valueChanged.connect(self._on_int_slider_changed)

            # Spin box with bounds
            spinbox = QSpinBox()
            if min_bound is not None and max_bound is not None:
                spinbox.setRange(int(min_bound), int(max_bound))
            else:
                spinbox.setRange(-2147483648, 2147483647)
            spinbox.setValue(self.original_value)
            spinbox.valueChanged.connect(self._on_int_spinbox_changed)

            if self.is_read_only:
                slider.setEnabled(False)
                spinbox.setEnabled(False)
                slider.setStyleSheet("QSlider { color: #888; }")
                spinbox.setStyleSheet(
                    "QSpinBox { color: #888; background-color: #f0f0f0; }")

            self.input_widgets = [slider, spinbox]
            layout.addWidget(slider)
            layout.addWidget(spinbox)

        elif self.param_type == ParameterType.PARAMETER_DOUBLE:
            # Get bounds from config
            min_bound, max_bound = self._get_bounds()

            # Slider (scaled) with bounds-aware range
            slider = QSlider(Qt.Orientation.Horizontal)

            if min_bound is not None and max_bound is not None:
                self.double_min = float(min_bound)
                self.double_max = float(max_bound)
            else:
                # Fallback to adaptive range
                if abs(self.original_value) > 1000:
                    range_factor = max(100.0, abs(self.original_value) * 0.1)
                    self.double_min = float(
                        max(-1e10, self.original_value - range_factor))
                    self.double_max = float(
                        min(1e10, self.original_value + range_factor))
                elif abs(self.original_value) > 10:
                    range_factor = max(10.0, abs(self.original_value) * 0.5)
                    self.double_min = float(
                        max(-10000.0, self.original_value - range_factor))
                    self.double_max = float(
                        min(10000.0, self.original_value + range_factor))
                else:
                    self.double_min = float(
                        max(-100.0, self.original_value - 10.0))
                    self.double_max = float(
                        min(1000.0, self.original_value + 10.0))

            # Ensure min != max to avoid division by zero
            if abs(self.double_max - self.double_min) < 1e-10:
                self.double_max = self.double_min + 1.0

            # Calculate slider steps based on spinbox precision
            # This minimizes floating point errors by using integer arithmetic
            self.slider_range = 10000  # Use larger range for better precision
            slider.setRange(0, self.slider_range)

            # Calculate initial slider position using better precision
            range_span = self.double_max - self.double_min
            if range_span > 0:
                normalized_pos = (self.original_value -
                                  self.double_min) / range_span
                # Round to nearest
                slider_val = int(normalized_pos * self.slider_range + 0.5)
            else:
                slider_val = 0

            slider.setValue(max(0, min(self.slider_range, slider_val)))
            slider.valueChanged.connect(self._on_double_slider_changed)

            # Double spin box with bounds
            spinbox = QDoubleSpinBox()
            if min_bound is not None and max_bound is not None:
                spinbox.setRange(float(min_bound), float(max_bound))
            else:
                spinbox.setRange(-1e10, 1e10)

            # Set decimals to a reasonable default
            # 4 decimals should be enough for most use cases
            spinbox.setDecimals(4)

            # Set step size based on range for better usability
            range_span = self.double_max - self.double_min
            if range_span > 0:
                # Step size is 1/1000th of the range, rounded to a nice value
                step = range_span / 1000.0
                # Round step to a nice value (1, 2, 5 pattern)
                import math
                magnitude = 10 ** (int(math.log10(step)))
                normalized = step / magnitude
                if normalized <= 1:
                    step = magnitude
                elif normalized <= 2:
                    step = 2 * magnitude
                elif normalized <= 5:
                    step = 5 * magnitude
                else:
                    step = 10 * magnitude
                spinbox.setSingleStep(step)
            else:
                spinbox.setSingleStep(0.1)

            spinbox.setValue(self.original_value)
            spinbox.valueChanged.connect(self._on_double_spinbox_changed)

            if self.is_read_only:
                slider.setEnabled(False)
                spinbox.setEnabled(False)
                slider.setStyleSheet("QSlider { color: #888; }")
                spinbox.setStyleSheet(
                    "QDoubleSpinBox { color: #888; background-color: #f0f0f0; }")

            self.input_widgets = [slider, spinbox]
            layout.addWidget(slider)
            layout.addWidget(spinbox)

        elif self.param_type == ParameterType.PARAMETER_STRING:
            widget = QLineEdit()
            widget.setText(str(self.original_value))
            widget.textChanged.connect(self._on_string_changed)
            if self.is_read_only:
                widget.setEnabled(False)
                widget.setStyleSheet(
                    "QLineEdit { color: #888; background-color: #f0f0f0; }")
            self.input_widgets.append(widget)
            layout.addWidget(widget)

        elif self.param_type in [ParameterType.PARAMETER_INTEGER_ARRAY,
                                 ParameterType.PARAMETER_DOUBLE_ARRAY,
                                 ParameterType.PARAMETER_BOOL_ARRAY,
                                 ParameterType.PARAMETER_STRING_ARRAY]:
            self._create_array_widgets(layout)

        self.setLayout(layout)

    def _create_array_widgets(self, layout):
        """Create widgets for array parameters"""
        array_widget = QWidget()
        array_layout = QHBoxLayout()
        array_layout.setContentsMargins(0, 0, 0, 0)

        for i, value in enumerate(self.original_value):
            if self.param_type == ParameterType.PARAMETER_INTEGER_ARRAY:
                widget = QSpinBox()
                # Full 32-bit integer range
                widget.setRange(-2147483648, 2147483647)
                widget.setValue(value)
                widget.valueChanged.connect(
                    lambda v, idx=i: self._on_array_changed(idx, v))
            elif self.param_type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                widget = QDoubleSpinBox()
                widget.setRange(-1e10, 1e10)  # Much larger range
                widget.setDecimals(6)
                widget.setValue(value)
                widget.valueChanged.connect(
                    lambda v, idx=i: self._on_array_changed(idx, v))
            elif self.param_type == ParameterType.PARAMETER_BOOL_ARRAY:
                widget = QCheckBox()
                widget.setChecked(value)
                widget.stateChanged.connect(
                    lambda s, idx=i: self._on_array_changed(idx, s == Qt.CheckState.Checked))
            else:  # STRING_ARRAY
                widget = QLineEdit()
                widget.setText(str(value))
                widget.textChanged.connect(
                    lambda t, idx=i: self._on_array_changed(idx, t))

            # Apply read-only styling to array widgets
            if self.is_read_only:
                widget.setEnabled(False)
                if hasattr(widget, 'setStyleSheet'):
                    if isinstance(widget, (QSpinBox, QDoubleSpinBox)):
                        widget.setStyleSheet(
                            "QSpinBox, QDoubleSpinBox { color: #888; background-color: #f0f0f0; }")
                    elif isinstance(widget, QLineEdit):
                        widget.setStyleSheet(
                            "QLineEdit { color: #888; background-color: #f0f0f0; }")
                    elif isinstance(widget, QCheckBox):
                        widget.setStyleSheet("QCheckBox { color: #888; }")

            self.input_widgets.append(widget)
            array_layout.addWidget(widget)

        array_widget.setLayout(array_layout)
        layout.addWidget(array_widget)

    def _on_bool_changed(self, state):
        self.current_value = state == Qt.CheckState.Checked
        self.has_been_modified = True
        self._schedule_update()

    def _on_int_slider_changed(self, value):
        value = self._enforce_bounds(value)
        self.current_value = value
        self.has_been_modified = True
        if len(self.input_widgets) > 1:
            self.input_widgets[1].setValue(value)
        self._schedule_update()

    def _on_int_spinbox_changed(self, value):
        value = self._enforce_bounds(value)
        self.current_value = value
        self.has_been_modified = True
        if len(self.input_widgets) > 1:
            self.input_widgets[0].setValue(value)
        self._schedule_update()

    def _on_double_slider_changed(self, value):
        # Use better calculation order to minimize floating point errors
        # 1. First calculate the exact fraction using integer arithmetic
        # 2. Then apply to range, minimizing operations on floating points

        range_span = self.double_max - self.double_min

        # Use integer arithmetic as much as possible
        # Calculate value as: min + (slider_pos * range) / slider_max
        # This order minimizes compounding errors
        if hasattr(self, 'slider_range'):
            # More accurate: multiply first, then divide
            double_val = self.double_min + \
                (value * range_span) / self.slider_range
        else:
            # Fallback for compatibility
            double_val = self.double_min + (value / 1000.0) * range_span

        double_val = self._enforce_bounds(double_val)

        # Round to appropriate precision based on spinbox decimals
        if len(self.input_widgets) > 1:
            decimals = self.input_widgets[1].decimals()
            # Use decimal quantization for exact representation
            factor = 10 ** decimals
            double_val = round(double_val * factor) / factor

        self.current_value = double_val
        self.has_been_modified = True
        if len(self.input_widgets) > 1:
            self.input_widgets[1].setValue(double_val)
        self._schedule_update()

    def _on_double_spinbox_changed(self, value):
        value = self._enforce_bounds(value)
        self.current_value = value
        self.has_been_modified = True
        if len(self.input_widgets) > 1:
            # Calculate slider position with minimal floating point operations
            range_span = self.double_max - self.double_min
            if range_span > 0:
                # Normalize position first, then scale to slider range
                normalized_pos = (value - self.double_min) / range_span
                if hasattr(self, 'slider_range'):
                    # Round to nearest
                    slider_val = int(normalized_pos * self.slider_range + 0.5)
                    max_val = self.slider_range
                else:
                    slider_val = int(normalized_pos * 1000 +
                                     0.5)  # Round to nearest
                    max_val = 1000
                self.input_widgets[0].setValue(
                    max(0, min(max_val, slider_val)))
        self._schedule_update()

    def _on_string_changed(self, text):
        self.current_value = text
        self.has_been_modified = True
        self._schedule_update()

    def _on_array_changed(self, index, value):
        if not isinstance(self.current_value, list):
            self.current_value = list(self.original_value)
        self.current_value[index] = value
        self.has_been_modified = True
        self._schedule_update()

    def _schedule_update(self):
        """Schedule parameter update with delay"""
        if not self.is_read_only:
            self.update_timer.start(self.update_delay)

    def _emit_parameter_change(self):
        """Emit parameter change signal"""
        if self.has_been_modified:
            self.parameter_changed.emit(
                self.node_name, self.param_name, self.current_value)

    def get_current_value(self):
        return self.current_value

    def has_changed(self):
        return not self.is_read_only and self.has_been_modified

    def reset_to_original(self):
        """Reset parameter to its original value"""
        if self.is_read_only:
            return

        self.current_value = self.original_value
        self.has_been_modified = False  # Reset modification flag

        # Update UI widgets
        if self.param_type == ParameterType.PARAMETER_BOOL:
            self.input_widgets[0].setChecked(self.original_value)
        elif self.param_type == ParameterType.PARAMETER_INTEGER:
            self.input_widgets[0].setValue(self.original_value)  # slider
            self.input_widgets[1].setValue(self.original_value)  # spinbox
        elif self.param_type == ParameterType.PARAMETER_DOUBLE:
            # spinbox triggers slider update
            self.input_widgets[1].setValue(self.original_value)
        elif self.param_type == ParameterType.PARAMETER_STRING:
            self.input_widgets[0].setText(str(self.original_value))
        elif self.param_type in [ParameterType.PARAMETER_INTEGER_ARRAY,
                                 ParameterType.PARAMETER_DOUBLE_ARRAY,
                                 ParameterType.PARAMETER_BOOL_ARRAY,
                                 ParameterType.PARAMETER_STRING_ARRAY]:
            for widget, original_val in zip(self.input_widgets, self.original_value):
                if self.param_type == ParameterType.PARAMETER_INTEGER_ARRAY:
                    widget.setValue(original_val)
                elif self.param_type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                    widget.setValue(original_val)
                elif self.param_type == ParameterType.PARAMETER_BOOL_ARRAY:
                    widget.setChecked(original_val)
                else:  # STRING_ARRAY
                    widget.setText(str(original_val))


class ParameterManagerGUI(QMainWindow):

    def __init__(self):
        super().__init__()
        self.ros_node = None
        self.parameter_widgets = {}  # {node_name: {param_name: widget}}
        self.pending_changes = {}  # {node_name: {param_name: value}}
        self.config_bounds = {}  # Config bounds from JSON
        self.config = {}  # Full config from JSON
        self.display_names = {}  # Display name overrides from JSON

        self.load_config()
        self.init_ros()
        self.init_ui()
        self.setup_discovery()

    def load_config(self):
        """Load configuration from JSON file"""
        config_path = os.path.join(os.path.dirname(__file__), 'config.json')
        try:
            if os.path.exists(config_path):
                with open(config_path, 'r') as f:
                    self.config = json.load(f)
                    self.config_bounds = self.config.get(
                        'parameter_bounds', {})
                    self.display_names = self.config.get(
                        'parameterDisplayNames', {})
                    print(
                        f"Loaded config with {len(self.config_bounds)} node bounds")
                    print(
                        f"Display name overrides: {len(self.display_names)} nodes")
                    print(
                        f"Multiple tabs: {self.config.get('multipleTabs', True)}")
                    print(
                        f"Whitelist enabled: {self.config.get('whitelist', False)}")
                    print(
                        f"Hide read-only: {self.config.get('hideReadOnlyParameters', False)}")
            else:
                print(f"Config file not found at {config_path}")
                self.config = {}
        except Exception as e:
            print(f"Error loading config: {e}")
            self.config = {}
            self.config_bounds = {}
            self.display_names = {}

    def _match_parameter_pattern(self, param_name: str, pattern: str) -> Optional[re.Match]:
        """Check if parameter name matches a pattern (supports regex and channel patterns)"""
        # Handle channel pattern: ch*_paramname -> ch[1-8]_paramname
        if '*' in pattern:
            regex_pattern = pattern.replace('*', r'(\d+)')
            return re.match(regex_pattern, param_name)
        # Direct match
        return re.match(f"^{pattern}$", param_name)

    def _get_pattern_config(self, node_name: str, param_name: str, config_dict: Dict) -> Any:
        """Get configuration value using pattern matching"""
        node_config = config_dict.get(node_name, {})

        # First try direct match
        if param_name in node_config:
            return node_config[param_name]

        # Then try pattern matching
        for pattern, value in node_config.items():
            if self._match_parameter_pattern(param_name, pattern):
                return value

        return None

    def init_ros(self):
        """Initialize ROS2 node"""
        rclpy.init()
        self.ros_node = Node('parameter_manager_gui')

        # Start ROS spinning in separate thread
        self.ros_thread = threading.Thread(
            target=self.ros_spin_thread, daemon=True)
        self.ros_thread.start()

    def ros_spin_thread(self):
        """ROS spinning thread"""
        while rclpy.ok():
            try:
                rclpy.spin_once(self.ros_node, timeout_sec=0.1)  # type: ignore
            except KeyboardInterrupt:
                print("ROS spin thread interrupted")
                break
            except Exception as e:
                if "executor was shutdown" not in str(e).lower():
                    print(f"ROS spin error: {e}")

    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle("ROS2 Parameter Manager")
        self.setGeometry(100, 100, 1400, 800)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)

        # Control buttons
        button_layout = QHBoxLayout()

        self.refresh_btn = QPushButton("Refresh Nodes")
        self.refresh_btn.clicked.connect(self.manual_refresh)
        button_layout.addWidget(self.refresh_btn)

        button_layout.addStretch()

        self.status_label = QLabel("Discovering nodes...")
        button_layout.addWidget(self.status_label)

        main_layout.addLayout(button_layout)

        # Progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        main_layout.addWidget(self.progress_bar)

        # Tab widget for nodes (conditional based on config)
        self.use_multiple_tabs = self.config.get('multipleTabs', True)
        if self.use_multiple_tabs:
            self.tab_widget = QTabWidget()
            main_layout.addWidget(self.tab_widget)
        else:
            # Single scrollable area for all nodes
            self.single_scroll_area = QScrollArea()
            self.single_scroll_widget = QWidget()
            self.single_scroll_layout = QVBoxLayout()
            self.single_scroll_widget.setLayout(self.single_scroll_layout)
            self.single_scroll_area.setWidget(self.single_scroll_widget)
            self.single_scroll_area.setWidgetResizable(True)
            main_layout.addWidget(self.single_scroll_area)

    def setup_discovery(self):
        """Setup optimized parameter discovery"""
        if self.ros_node is not None:
            self.discovery = ParameterDiscovery(self.ros_node)
            self.discovery.nodes_updated.connect(self.update_node_parameters)
            self.discovery.start()  # Start discovery thread

    def manual_refresh(self):
        """Manually trigger node discovery"""
        # Clear cache to force fresh discovery
        self.discovery.service_clients.clear()
        self.discovery.last_nodes.clear()

    def update_node_parameters(self, node_params: Dict[str, Dict]):
        """Update GUI with discovered node parameters"""
        # Apply whitelist filtering
        filtered_params = self._apply_whitelist_filter(node_params)

        self.status_label.setText(f"Found {len(filtered_params)} nodes")

        # Clear existing display
        if self.use_multiple_tabs:
            self.tab_widget.clear()
        else:
            # Clear single scroll layout completely (widgets and spacers)
            while self.single_scroll_layout.count():
                item = self.single_scroll_layout.takeAt(0)
                if item:
                    widget = item.widget()
                    if widget:
                        widget.setParent(None)
                    # This also removes spacer items

        self.parameter_widgets.clear()

        # Organize parameters by channel for processor, keep publisher parameters separate
        if self.use_multiple_tabs:
            self.create_channel_tabs(filtered_params)
        else:
            for node_name, params in filtered_params.items():
                self.create_node_section(node_name, params)

        # Add stretch at the end in single page mode to prevent unnecessary spacing
        if not self.use_multiple_tabs:
            self.single_scroll_layout.addStretch()

    def _apply_whitelist_filter(self, node_params: Dict[str, Dict]) -> Dict[str, Dict]:
        """Apply whitelist filtering based on config"""
        if not self.config.get('whitelist', False):
            return node_params

        filtered_params = {}
        whitelist_nodes = self.config.get('whitelistNodes', [])
        whitelist_parameters = self.config.get('whitelistParameters', {})

        for node_name, params in node_params.items():
            # Check if node is whitelisted
            if whitelist_nodes and node_name not in whitelist_nodes:
                continue

            # Filter parameters for this node
            node_whitelist_params = whitelist_parameters.get(node_name, [])
            if node_whitelist_params:
                filtered_node_params = {}
                for param_name, param_info in params.items():
                    # Check if parameter matches any whitelist pattern
                    matched = False
                    for pattern in node_whitelist_params:
                        if self._match_parameter_pattern(param_name, pattern):
                            matched = True
                            break
                    if matched:
                        filtered_node_params[param_name] = param_info
                if filtered_node_params:
                    filtered_params[node_name] = filtered_node_params
            else:
                filtered_params[node_name] = params

        return filtered_params

    def _should_show_parameter(self, param_info: Dict, param_name: Optional[str] = None) -> bool:
        """Check if parameter should be shown based on config"""
        # Check if parameter is in alwaysShowParameters list
        if param_name and param_name in self.config.get('alwaysShowParameters', []):
            return True

        if self.config.get('hideReadOnlyParameters', False):
            if param_info.get('read_only', False):
                return False
        return True

    def _get_active_channels_from_publisher(self) -> Optional[list]:
        """Get active channels directly from publisher node"""
        try:
            if not self.ros_node:
                return None

            # Create service client for getting publisher parameters
            client = self.ros_node.create_client(
                GetParameters, '/ascan_publisher/get_parameters')

            if not client.wait_for_service(timeout_sec=1.0):
                print("Publisher service not available")
                return None

            # Request channelsOnArray parameter
            request = GetParameters.Request()
            request.names = ['channelsOnReceive']

            future = client.call_async(request)
            if self.ros_node:
                while not future.done():
                    time.sleep(0.01)

            result = future.result()
            if result and result.values:
                pv0 = result.values[0]
                if pv0.type != ParameterType.PARAMETER_INTEGER_ARRAY:
                    print('Publisher channelsOnReceive parameter is not an integer array (maybe not set?)')
                    return None
                channels_on_array = pv0.integer_array_value
                # Get indices of active channels (where value is 1) - 1-based
                active_indices = [i + 1 for i, is_active in enumerate(
                    channels_on_array) if is_active]
                print(
                    f"Got channelsOnReceive from publisher: {channels_on_array}")
                print(f"Active channel indices: {active_indices}")
                return active_indices

        except Exception as e:
            print(f"Error getting active channels from publisher: {e}")

        return None

    def create_channel_tabs(self, node_params: Dict[str, Dict]):
        """Create tabs organized by channel for processor parameters, plus publisher tab"""
        # Extract channel parameters from processor
        processor_params = node_params.get('ascan_processor', {})
        publisher_params = node_params.get('ascan_publisher', {})

        # Get active channels by directly accessing the publisher node
        # This bypasses the filtering to get read-only parameters
        active_channel_indices = self._get_active_channels_from_publisher()

        print(
            f"Active channel indices from publisher: {active_channel_indices}")

        # Group processor parameters by channel
        channel_params = {}
        for param_name, param_info in processor_params.items():
            if param_name.startswith('ch') and '_' in param_name:
                # Extract 'ch0', 'ch1', etc.
                channel_num = param_name.split('_')[0]
                channel_index = int(channel_num[2:])

                # Skip if this channel is not active
                if active_channel_indices is not None and channel_index not in active_channel_indices:
                    continue

                if channel_num not in channel_params:
                    channel_params[channel_num] = {}
                channel_params[channel_num][param_name] = param_info

        # Create tab for each active channel
        for channel_num in sorted(channel_params.keys()):
            if channel_params[channel_num]:  # Only create tab if channel has parameters
                # Extract number from 'ch1', 'ch2', etc.
                channel_index = int(channel_num[2:])
                tab_name = f"Channel {channel_index}"
                self.create_channel_tab(
                    tab_name, 'ascan_processor', channel_params[channel_num])

        # Create publisher tab with all publisher parameters
        if publisher_params:
            self.create_channel_tab(
                "Publisher", 'ascan_publisher', publisher_params)

    def create_channel_tab(self, tab_name: str, node_name: str, params: Dict[str, Dict]):
        """Create a tab for channel-specific or publisher parameters"""
        scroll_area = QScrollArea()
        scroll_widget = QWidget()
        layout = QVBoxLayout()
        scroll_widget.setLayout(layout)

        # Tab info
        visible_params = {
            k: v for k, v in params.items() if self._should_show_parameter(v)}

        info_label = QLabel(
            f"{tab_name} ({len(visible_params)} parameters)")
        info_label.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(info_label)

        # Initialize parameter widgets for this node if not exists
        if node_name not in self.parameter_widgets:
            self.parameter_widgets[node_name] = {}

        # Parameters
        for param_name, param_info in visible_params.items():
            try:
                param_widget = ParameterWidget(
                    param_name, param_info, node_name, self, self.config_bounds, self.display_names)
                param_widget.parameter_changed.connect(
                    self._on_parameter_changed)
                layout.addWidget(param_widget)
                self.parameter_widgets[node_name][param_name] = param_widget
            except Exception as e:
                print(
                    f"Error creating widget for {node_name}.{param_name}: {e}")

        layout.addStretch()

        scroll_area.setWidget(scroll_widget)
        scroll_area.setWidgetResizable(True)

        self.tab_widget.addTab(scroll_area, tab_name)

    def create_node_section(self, node_name: str, params: Dict[str, Dict]):
        """Create a section for a specific node's parameters in single page mode"""
        # Node info
        visible_params = {
            k: v for k, v in params.items() if self._should_show_parameter(v)}

        # Parameters
        self.parameter_widgets[node_name] = {}

        for param_name, param_info in visible_params.items():
            try:
                param_widget = ParameterWidget(
                    param_name, param_info, node_name, self, self.config_bounds, self.display_names)
                param_widget.parameter_changed.connect(
                    self._on_parameter_changed)
                self.single_scroll_layout.addWidget(param_widget)
                self.parameter_widgets[node_name][param_name] = param_widget
            except Exception as e:
                print(
                    f"Error creating widget for {node_name}.{param_name}: {e}")

    def _on_parameter_changed(self, node_name: str, param_name: str, value):
        """Handle real-time parameter changes"""
        try:
            success = self.set_node_parameters(node_name, {param_name: value})
            if success:
                self.status_label.setText(
                    f"Updated {node_name}.{param_name} = {value}")
            else:
                self.status_label.setText(
                    f"Failed to update {node_name}.{param_name}")
        except Exception as e:
            print(f"Error updating parameter {node_name}.{param_name}: {e}")
            self.status_label.setText(
                f"Error updating {node_name}.{param_name}")

    def set_node_parameters(self, node_name: str, changes: Dict[str, Any]) -> bool:
        """Set parameters for a specific node"""
        try:
            if not self.ros_node:
                return False
            client = self.ros_node.create_client(
                SetParameters, f'/{node_name}/set_parameters')

            if not client.wait_for_service(timeout_sec=2.0):
                raise Exception(f"Service not available for {node_name}")

            request = SetParameters.Request()

            for param_name, value in changes.items():
                param_msg = ParameterMsg()
                param_msg.name = param_name
                param_msg.value = self._python_to_parameter_value(value)
                request.parameters.append(param_msg)  # type: ignore

            future = client.call_async(request)
            if self.ros_node:
                while not future.done():
                    time.sleep(0.01)

            result = future.result()
            if result:
                for res in result.results:
                    if not res.successful:
                        raise Exception(
                            f"Parameter set failed: {res.reason}")
                return True
            else:
                raise Exception("Service call failed")

        except Exception as e:
            print(f"Error setting parameters for {node_name}: {e}")
            return False

    def _python_to_parameter_value(self, value) -> ParameterValue:
        """Convert Python value to ROS2 ParameterValue"""
        param_value = ParameterValue()

        if isinstance(value, bool):
            param_value.type = ParameterType.PARAMETER_BOOL
            param_value.bool_value = value
        elif isinstance(value, int):
            param_value.type = ParameterType.PARAMETER_INTEGER
            param_value.integer_value = value
        elif isinstance(value, float):
            param_value.type = ParameterType.PARAMETER_DOUBLE
            param_value.double_value = value
        elif isinstance(value, str):
            param_value.type = ParameterType.PARAMETER_STRING
            param_value.string_value = value
        elif isinstance(value, list):
            if all(isinstance(x, bool) for x in value):
                param_value.type = ParameterType.PARAMETER_BOOL_ARRAY
                param_value.bool_array_value = value
            elif all(isinstance(x, int) for x in value):
                param_value.type = ParameterType.PARAMETER_INTEGER_ARRAY
                param_value.integer_array_value = value
            elif all(isinstance(x, float) for x in value):
                param_value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                param_value.double_array_value = value
            elif all(isinstance(x, str) for x in value):
                param_value.type = ParameterType.PARAMETER_STRING_ARRAY
                param_value.string_array_value = value

        return param_value

    def closeEvent(self, a0):
        """Handle application close"""
        if hasattr(self, 'discovery'):
            self.discovery.stop()
        if self.ros_node:
            self.ros_node.destroy_node()
        rclpy.shutdown()
        if a0:
            a0.accept()


def main():
    app = QApplication(sys.argv)

    # Set application properties for proprietary use with dynamic linking
    app.setApplicationName("ROS2 Parameter Manager")
    app.setApplicationVersion("1.0")
    app.setOrganizationName("Proprietary Software")

    window = ParameterManagerGUI()
    window.show()

    # Handle keyboard interrupt (Ctrl+C) gracefully
    def signal_handler(*_):
        print("\nKeyboard interrupt received. Shutting down gracefully...")
        window.close()
        app.quit()

    # Install signal handler
    signal.signal(signal.SIGINT, signal_handler)

    # Create a timer to allow Python to process signals
    # Qt blocks signal handling, so we need to periodically yield control
    timer = QTimer()
    timer.timeout.connect(lambda: None)  # Empty callback to process events
    timer.start(100)  # Check every 100ms

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Shutting down gracefully...")
        window.close()
        sys.exit(0)


if __name__ == '__main__':
    main()
