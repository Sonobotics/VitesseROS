import signal
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from . import motoron
import time


class BackwardNode(Node):

    def __init__(self):
        super().__init__('backward_node')
        ## initialise actuator
        self.mc = motoron.MotoronSerial()
        self.mc.set_port("/dev/ttyS2")
        self.mc.reinitialize()  # Bytes: 0x96 0x74
        self.mc.disable_crc()   # Bytes: 0x8B 0x04 0x7B 0x43
        self.mc.clear_reset_flag()  # Bytes: 0xA9 0x00 0x04
        self.mc.set_command_timeout_milliseconds(2000)
        self.mc.set_max_acceleration(1, 1000)
        self.mc.set_max_deceleration(1, 1000)
        # advertise a Trigger service that executes the motor-on logic
        self.service = self.create_service(Trigger, 'actuate_backward', self.handle_actuate_backward)
        self.get_logger().info('Motor service /actuate_backward ready')

    def handle_actuate_backward(self, request, response):
        # Drives the actuator backward (positive speed in this system)
        self.get_logger().info('Actuate backward command received')

        self.mc.set_speed(1, 800)
        time.sleep(2)

        response.success = True
        response.message = 'Actuated backward'
        return response


def backward_main(args=None):
    rclpy.init(args=args)
    node = BackwardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    backward_main()