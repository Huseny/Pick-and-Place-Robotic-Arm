import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers, SwitchController
from controller_manager_msgs.srv._list_controllers import ListControllers_Request as ListControllersRequest
from controller_manager_msgs.srv._switch_controller import SwitchController_Request as SwitchControllerRequest

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.controller_manager = self.create_client(ListControllers, 'list_controllers')
        self.switch_controller = self.create_client(SwitchController, 'switch_controllers')

    def list_controllers(self):
        request = ListControllersRequest()
        future = self.controller_manager.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def switch_controllers(self, start_controllers, stop_controllers):
        request = SwitchControllerRequest()
        request.start_controllers = start_controllers
        request.stop_controllers = stop_controllers
        request.strictness = SwitchControllerRequest.STRICT

        future = self.switch_controller.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    controller_node = RobotController()

    # Example: Move joint_1 to position 1.0
    controller_node.switch_controllers(["joint_2"], [])
    controller_node.list_controllers()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
