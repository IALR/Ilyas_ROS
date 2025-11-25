import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsClient(Node):
    """
    A service client that sends two numbers to be added.
    """
    
    def __init__(self):
        super().__init__('add_two_ints_client')
        
        # Create client
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        
        # Wait for service to be available
        self.get_logger().info('Waiting for Add Two Ints service...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.get_logger().info('Service found!')
    
    def send_request(self, a, b):
        """
        Send a request to add two numbers.
        """
        # Create request
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        self.get_logger().info(f'Sending request: {a} + {b}')
        
        # Send request asynchronously
        future = self.client.call_async(request)
        
        return future


def main(args=None):
    rclpy.init(args=args)
    
    # Check if user provided two numbers
    if len(sys.argv) != 3:
        print('Usage: ros2 run service_practice add_client <num1> <num2>')
        return
    
    # Get numbers from command line
    try:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
    except ValueError:
        print('Error: Please provide valid integers')
        return
    
    # Create client node
    client = AddTwoIntsClient()
    
    # Send request
    future = client.send_request(a, b)
    
    # Wait for response
    rclpy.spin_until_future_complete(client, future)
    
    # Get and display result
    try:
        response = future.result()
        client.get_logger().info(f'Result: {a} + {b} = {response.sum}')
    except Exception as e:
        client.get_logger().error(f'Service call failed: {e}')
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()