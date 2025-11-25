import rclpy
from rclpy.node import Node
from fleet_interfaces.msg import RobotStatus, FleetCommand
from geometry_msgs.msg import Pose, Twist, Point
from std_msgs.msg import Header
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, SetParametersResult
import random
import math


class RobotSimulator(Node):
    """
    Simulates a robot with configurable parameters.
    
    This node demonstrates:
    - Declaring parameters with validation
    - Publishing custom messages
    - Subscribing to commands
    - Runtime parameter changes
    """
    
    def __init__(self):
        super().__init__('robot_simulator')
        
        # ============================================
        # PARAMETER DECLARATIONS
        # ============================================
        # Parameters are declared with descriptors that:
        # 1. Document what the parameter does
        # 2. Set allowed ranges
        # 3. Provide default values
        
        self.declare_parameter('robot_id', 'robot_unknown',
            ParameterDescriptor(description='Unique robot identifier'))
        
        self.declare_parameter('robot_type', 'scout',
            ParameterDescriptor(description='Robot type: scout, transporter, cleaner'))
        
        # FloatingPointRange enforces min/max values
        self.declare_parameter('initial_battery', 100.0,
            ParameterDescriptor(
                description='Initial battery percentage (0-100)',
                floating_point_range=[FloatingPointRange(from_value=0.0, to_value=100.0)]))
        
        self.declare_parameter('max_speed', 2.0,
            ParameterDescriptor(
                description='Maximum velocity in m/s',
                floating_point_range=[FloatingPointRange(from_value=0.1, to_value=5.0)]))
        
        self.declare_parameter('battery_drain_rate', 0.5,
            ParameterDescriptor(description='Battery drain percentage per minute'))
        
        self.declare_parameter('publish_rate', 1.0,
            ParameterDescriptor(description='Status publish rate in Hz'))
        
        self.declare_parameter('low_battery_threshold', 30.0,
            ParameterDescriptor(description='Battery % to trigger low battery warning'))
        
        self.declare_parameter('critical_battery_threshold', 15.0,
            ParameterDescriptor(description='Battery % to trigger critical warning'))
        
        # ============================================
        # GET PARAMETER VALUES
        # ============================================
        # After declaring, we retrieve the actual values
        # These could come from YAML config or command line
        
        self.robot_id = self.get_parameter('robot_id').value
        self.robot_type = self.get_parameter('robot_type').value
        self.battery = self.get_parameter('initial_battery').value
        self.max_speed = self.get_parameter('max_speed').value
        self.drain_rate = self.get_parameter('battery_drain_rate').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.low_threshold = self.get_parameter('low_battery_threshold').value
        self.critical_threshold = self.get_parameter('critical_battery_threshold').value
        
        # ============================================
        # PARAMETER CHANGE CALLBACK
        # ============================================
        # This allows runtime parameter changes via:
        # ros2 param set /robot_simulator max_speed 5.0
        
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # ============================================
        # PUBLISHERS
        # ============================================
        self.status_pub = self.create_publisher(RobotStatus, 'robot_status', 10)
        
        # ============================================
        # SUBSCRIBERS
        # ============================================
        self.command_sub = self.create_subscription(
            FleetCommand, 'fleet_command', self.command_callback, 10)
        
        # ============================================
        # TIMER FOR STATUS PUBLISHING
        # ============================================
        timer_period = 1.0 / self.publish_rate  # Convert Hz to seconds
        self.timer = self.create_timer(timer_period, self.publish_status)
        
        # ============================================
        # ROBOT STATE VARIABLES
        # ============================================
        self.position = Point(x=0.0, y=0.0, z=0.0)
        self.velocity = Twist()
        self.is_charging = False
        self.current_task = "idle"
        self.task_progress = 0
        self.is_operational = True
        self.distance_traveled = 0.0
        self.tasks_completed = 0
        self.warnings = []
        self.errors = []
        
        self.get_logger().info(f'🤖 Robot {self.robot_id} ({self.robot_type}) initialized!')
        self.log_configuration()
    
    def parameter_callback(self, params):
        """
        Validate and apply parameter changes at runtime.
        
        This is called when someone runs:
        ros2 param set /robot_simulator max_speed 5.0
        
        Returns:
            SetParametersResult: Success or failure
        """
        for param in params:
            if param.name == 'max_speed':
                # Validate the new value
                if param.value <= 0.0 or param.value > 5.0:
                    self.get_logger().error(f'❌ Invalid max_speed: {param.value}')
                    return SetParametersResult(successful=False)
                self.max_speed = param.value
                self.get_logger().info(f'✅ max_speed updated to {param.value} m/s')
                
            elif param.name == 'publish_rate':
                if param.value <= 0.0:
                    return SetParametersResult(successful=False)
                self.publish_rate = param.value
                # Need to recreate timer with new rate!
                self.timer.cancel()
                timer_period = 1.0 / self.publish_rate
                self.timer = self.create_timer(timer_period, self.publish_status)
                self.get_logger().info(f'✅ publish_rate updated to {param.value} Hz')
                
            elif param.name == 'low_battery_threshold':
                # Validation: low threshold must be above critical
                if param.value <= self.critical_threshold:
                    self.get_logger().error(
                        '❌ low_threshold must be > critical_threshold')
                    return SetParametersResult(successful=False)
                self.low_threshold = param.value
        
        self.log_configuration()
        return SetParametersResult(successful=True)
    
    def log_configuration(self):
        """Log current robot configuration."""
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'📋 {self.robot_id} Configuration')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Type: {self.robot_type}')
        self.get_logger().info(f'Max Speed: {self.max_speed} m/s')
        self.get_logger().info(f'Battery: {self.battery:.1f}%')
        self.get_logger().info(f'Publish Rate: {self.publish_rate} Hz')
        self.get_logger().info(f'Drain Rate: {self.drain_rate}%/min')
        self.get_logger().info('=' * 50)
    
    def command_callback(self, msg):
        """
        Process fleet commands.
        
        Commands can target:
        - Specific robots (by ID)
        - Robot types (all scouts)
        - All robots (empty lists)
        """
        # Check if this command is for us
        if msg.target_robot_ids and self.robot_id not in msg.target_robot_ids:
            return  # Not for this robot
        
        if msg.robot_type_filter and msg.robot_type_filter != self.robot_type:
            return  # Wrong robot type
        
        self.get_logger().info(f'📨 Received command: {msg.command_type}')
        
        # Process different command types
        if msg.command_type == 'stop':
            self.velocity = Twist()
            self.current_task = 'idle'
            self.get_logger().info('🛑 Stopping all motion')
            
        elif msg.command_type == 'charge':
            self.is_charging = True
            self.current_task = 'charging'
            self.get_logger().info('🔌 Starting to charge')
            
        elif msg.command_type == 'goto':
            self.current_task = 'navigating'
            self.task_progress = 0
            self.get_logger().info(f'🎯 Navigating to target position')
            
        elif msg.command_type == 'task':
            self.current_task = msg.task_description
            self.task_progress = 0
            self.get_logger().info(f'📋 Starting task: {msg.task_description}')
    
    def publish_status(self):
        """
        Publish robot status at configured rate.
        
        This is called by the timer every 1/publish_rate seconds.
        """
        msg = RobotStatus()
        
        # ============================================
        # HEADER
        # ============================================
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{self.robot_id}_frame'
        
        # ============================================
        # IDENTIFICATION
        # ============================================
        msg.robot_id = self.robot_id
        msg.robot_type = self.robot_type
        msg.location = f'Zone_{random.randint(1, 5)}'
        
        # ============================================
        # MOTION STATE
        # ============================================
        msg.pose = Pose()
        msg.pose.position = self.position
        msg.velocity = self.velocity
        
        # ============================================
        # BATTERY SIMULATION
        # ============================================
        # If charging, increase battery
        if self.is_charging and self.battery < 100.0:
            self.battery = min(100.0, self.battery + 0.5)
        # Otherwise, drain based on drain_rate
        elif not self.is_charging:
            # drain_rate is per minute, so divide by 60 for per-second
            self.battery = max(0.0, self.battery - (self.drain_rate / 60.0))
        
        msg.battery_voltage = 12.0 + (self.battery / 100.0) * 0.8  # 12.0-12.8V range
        msg.battery_current = -5.0 if self.is_charging else 2.5  # Negative = charging
        msg.battery_percentage = int(self.battery)
        msg.is_charging = self.is_charging
        
        # Battery health assessment
        if self.battery > 80:
            msg.battery_health = "excellent"
        elif self.battery > 50:
            msg.battery_health = "good"
        elif self.battery > 20:
            msg.battery_health = "fair"
        else:
            msg.battery_health = "poor"
        
        # ============================================
        # TASK INFORMATION
        # ============================================
        msg.current_task_id = f'task_{random.randint(1000, 9999)}'
        msg.task_status = self.current_task
        msg.task_progress = self.task_progress
        
        # ============================================
        # SYSTEM HEALTH
        # ============================================
        msg.is_operational = self.is_operational
        msg.cpu_temperature = 45.0 + random.uniform(-5.0, 5.0)
        msg.network_signal = random.randint(70, 100)
        
        # ============================================
        # WARNINGS AND ERRORS
        # ============================================
        self.warnings = []
        self.errors = []
        
        # Check battery thresholds
        if self.battery < self.low_threshold:
            self.warnings.append(f'Low battery: {self.battery:.1f}%')
        if self.battery < self.critical_threshold:
            self.errors.append(f'Critical battery: {self.battery:.1f}%')
        
        # Check temperature
        if msg.cpu_temperature > 70:
            self.warnings.append(f'High CPU temp: {msg.cpu_temperature:.1f}°C')
        
        msg.active_warnings = self.warnings
        msg.active_errors = self.errors
        
        # ============================================
        # PERFORMANCE METRICS
        # ============================================
        msg.distance_traveled = self.distance_traveled
        msg.tasks_completed = self.tasks_completed
        
        # ============================================
        # PUBLISH
        # ============================================
        self.status_pub.publish(msg)
        
        # ============================================
        # SIMULATE TASK PROGRESS
        # ============================================
        if self.current_task not in ['idle', 'charging']:
            self.task_progress = min(100, self.task_progress + 5)
            if self.task_progress >= 100:
                self.tasks_completed += 1
                self.current_task = 'idle'
                self.task_progress = 0
                self.get_logger().info(f'✅ Task completed! Total: {self.tasks_completed}')


def main(args=None):
    rclpy.init(args=args)
    node = RobotSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()