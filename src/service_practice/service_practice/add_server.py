import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):
    """
    A service server that adds two integers.
    """
    
    def __init__(self):
        super().__init__('add_two_ints_server')
        
        # Create service
        self.srv = self.create_service(
            AddTwoInts,              # Service type
            'add_two_ints',          # Service name
            self.add_two_ints_callback  # Callback function
        )
        
        self.get_logger().info('Add Two Ints Server is ready!')
    
    def add_two_ints_callback(self, request, response):
        """
        This function is called when a client sends a request.
        
        Args:
            request: Contains request.a and request.b
            response: Object we fill and return
        """
        # Perform the addition
        response.sum = request.a + request.b
        
        # Log what happened
        self.get_logger().info(
            f'Request: {request.a} + {request.b} = {response.sum}'
        )
        
        # Return the response
        return response


def main(args=None):
    rclpy.init(args=args)
    
    server = AddTwoIntsServer()
    
    server.get_logger().info('Add Two Ints server is running...')
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    
    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()