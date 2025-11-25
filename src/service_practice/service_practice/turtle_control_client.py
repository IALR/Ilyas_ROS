import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool


class TurtleControlClient(Node):
    """
    Client to enable/disable turtle movement.
    """
    
    def __init__(self):
        super().__init__('turtle_control_client')
        
        self.client = self.create_client(SetBool, 'validate_movement')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for validator service...')
    
    def send_request(self, enable):
        """
        Send request to enable or disable movement.
        """
        request = SetBool.Request()
        request.data = enable
        
        action = "enable" if enable else "disable"
        self.get_logger().info(f'Requesting to {action} movement')
        
        future = self.client.call_async(request)
        return future


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) != 2:
        print('Usage: ros2 run service_practice turtle_control_client <enable/disable>')
        return
    
    command = sys.argv[1].lower()
    
    if command not in ['enable', 'disable']:
        print('Error: Use "enable" or "disable"')
        return
    
    enable = (command == 'enable')
    
    client = TurtleControlClient()
    future = client.send_request(enable)
    
    rclpy.spin_until_future_complete(client, future)
    
    try:
        response = future.result()
        client.get_logger().info(f'Response: {response.message}')
    except Exception as e:
        client.get_logger().error(f'Service call failed: {e}')
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()