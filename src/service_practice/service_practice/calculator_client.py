import sys
import rclpy
from rclpy.node import Node
from service_practice.srv import Calculator


class CalculatorClient(Node):
    def __init__(self):
        super().__init__('calculator_client')

        self.client = self.create_client(Calculator, 'calculator')

        # Wait for the service
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

    def send_request(self, a, b, operation):
        req = Calculator.Request()
        req.a = a
        req.b = b
        req.operation = operation
        return self.client.call_async(req)


def main():
    rclpy.init()

    if len(sys.argv) != 4:
        print("Usage: ros2 run service_practice calculator_client <a> <b> <operation>")
        return

    a = int(sys.argv[1])
    b = int(sys.argv[2])
    op = sys.argv[3]

    client = CalculatorClient()
    future = client.send_request(a, b, op)

    rclpy.spin_until_future_complete(client, future)

    response = future.result()

    print(f"Result: {response.result}")
    print(f"Message: {response.message}")

    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
