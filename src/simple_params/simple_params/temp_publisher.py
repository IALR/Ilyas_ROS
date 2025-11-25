import rclpy
from rclpy.node import Node
from simple_interfaces.msg import TemperatureReading
from rcl_interfaces.msg import ParameterDescriptor
import random
from rcl_interfaces.msg import SetParametersResult


class TempPublisher(Node):
    """Simple temperature publisher with configurable parameters."""

    def __init__(self):
        super().__init__('temp_publisher')

        # Declare parameters
        self.declare_parameter(
            'sensor_id', 'sensor_01',
            ParameterDescriptor(description='Sensor identifier')
        )

        self.declare_parameter(
            'publish_rate', 1.0,
            ParameterDescriptor(description='Publishing rate in Hz')
        )

        self.declare_parameter(
            'base_temperature', 25.0,
            ParameterDescriptor(description='Base temperature in Celsius')
        )

        # Get parameter values
        self.sensor_id = self.get_parameter('sensor_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.base_temp = self.get_parameter('base_temperature').value

        # Add callback for runtime parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Create publisher
        self.publisher = self.create_publisher(TemperatureReading, 'temperature', 10)

        # Create timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_temperature)

        self.get_logger().info(f'🌡️  Temperature Publisher started!')
        self.get_logger().info(f'Sensor ID: {self.sensor_id}')
        self.get_logger().info(f'Base Temperature: {self.base_temp}°C')
        self.get_logger().info(f'Publish Rate: {self.publish_rate} Hz')

    def parameter_callback(self, params):
        """Handle parameter changes at runtime."""
        for param in params:
            if param.name == 'sensor_id':
                self.sensor_id = param.value
                self.get_logger().info(f'✅ Updated sensor_id to: {self.sensor_id}')

            elif param.name == 'base_temperature':
                self.base_temp = param.value
                self.get_logger().info(f'✅ Updated base_temperature to: {self.base_temp}°C')

            elif param.name == 'publish_rate':
                # Recreate timer with new rate
                self.publish_rate = param.value
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_temperature)
                self.get_logger().info(f'✅ Updated publish_rate to: {self.publish_rate} Hz')

        return SetParametersResult(successful=True)

    def publish_temperature(self):
        """Publish a temperature reading."""
        msg = TemperatureReading()
        msg.sensor_id = self.sensor_id
        # Add some random variation
        msg.temperature = self.base_temp + random.uniform(-2.0, 2.0)

        self.publisher.publish(msg)
        self.get_logger().info(f'📊 {msg.sensor_id}: {msg.temperature:.2f}°C')


def main(args=None):
    rclpy.init(args=args)
    node = TempPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
