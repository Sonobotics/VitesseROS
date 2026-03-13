# Simple smoke test for motor_node
# verifies that the module can be imported and the service is created without error

import rclpy
from ros_parameter_demo.workflow import motor_node
from std_srvs.srv import Trigger


def test_motor_node_service():
    rclpy.init()
    node = motor_node.MotorNode()
    # node should have a service attribute
    assert hasattr(node, 'service')
    # service type should be Trigger
    assert node.service.srv_type == Trigger
    node.destroy_node()
    rclpy.shutdown()
