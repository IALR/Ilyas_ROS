import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
from geometry_msgs.msg import Twist


class TurtleValidatorServer(Node):
    """
    Service that validates if turtle can move safely.
    Also publishes movement commands if valid.
    """
    
    def __init__(self):
        super().__init__('turtle_validator_server')
        
        # Create service
        self.srv = self.create_service(
            SetBool,
            'validate_movement',
            self.validate_callback
        )
        
        # Create publisher for turtle movement
        self.cmd_pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )
        
        self.movement_enabled = True
        
        self.get_logger().info('Turtle Validator Server ready!')
    
    def validate_callback(self, request, response):
        """
        Enable or disable turtle movement.
        
        Request: bool data (True = enable, False = disable)
        Response: bool success, string message
        """
        self.movement_enabled = request.data
        
        if self.movement_enabled:
            response.success = True
            response.message = 'Turtle movement ENABLED'
            self.get_logger().info('Movement enabled')
        else:
            response.success = True
            response.message = 'Turtle movement DISABLED'
            self.get_logger().info('Movement disabled')
            
            # Stop turtle
            stop_msg = Twist()
            self.cmd_pub.publish(stop_msg)
        
        return response


def main(args=None):
    rclpy.init(args=args)
    server = TurtleValidatorServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    
    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()