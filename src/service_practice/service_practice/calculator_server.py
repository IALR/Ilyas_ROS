import rclpy
from rclpy.node import Node
from service_practice.srv import Calculator


class CalculatorServer(Node):
    """
    A calculator service server that can add, subtract, multiply, or divide two numbers.
    """

    def __init__(self):
        super().__init__('calculator_server')

        # Create the service
        self.srv = self.create_service(
            Calculator,               # Custom service type
            'calculator',             # Service name
            self.calculator_callback  # Callback
        )

        self.get_logger().info('Calculator Service is ready!')

    def calculator_callback(self, request, response):
        """
        Handles calculator operations: add, sub, mul, div.
        """
        a = request.a
        b = request.b
        op = request.operation.lower()

        # Determine operation
        if op == "add":
            response.result = a + b
            response.message = "Success: addition performed."

        elif op == "sub":
            response.result = a - b
            response.message = "Success: subtraction performed."

        elif op == "mul":
            response.result = a * b
            response.message = "Success: multiplication performed."

        elif op == "div":
            if b == 0:
                response.result = 0.0
                response.message = "Error: Division by zero."
            else:
                response.result = a / b
                response.message = "Success: division performed."

        else:
            response.result = 0.0
            response.message = "Error: Unknown operation. Use add, sub, mul, div."

        # Log operation
        self.get_logger().info(
            f"Request: {a} {op} {b} -> {response.result} ({response.message})"
        )

        return response


def main(args=None):
    rclpy.init(args=args)

    server = CalculatorServer()

    server.get_logger().info('Calculator server is running...')

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass

    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
