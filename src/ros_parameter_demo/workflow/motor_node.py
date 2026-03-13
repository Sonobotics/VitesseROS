import signal
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import motoron
import time


class MotorNode(Node):

    def __init__(self):
        super().__init__('motor_controller')
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
        self.service = self.create_service(Trigger, 'run_motor', self.handle_run_motor)
        self.get_logger().info('Motor service /run_motor ready')

    def handle_run_motor(self, request, response):
        # Placeholder for actual motor-on code.  Users should replace this
        # with whatever API is required to drive their hardware.
        self.get_logger().info('Run motor command received')

        self.mc.set_speed(1, 800)
        time.sleep(1)
        self.mc.set_speed(1, -800)
        time.sleep(1)

        response.success = True
        response.message = 'Motor executed'
        return response


def main(args=None):
    rclpy.init(args=args)
    motor_node = MotorNode()
    try:
        rclpy.spin(motor_node)
    except KeyboardInterrupt:
        pass
    finally:
        motor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
