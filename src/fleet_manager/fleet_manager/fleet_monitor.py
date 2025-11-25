import rclpy
from rclpy.node import Node
from fleet_interfaces.msg import RobotStatus, FleetCommand
from fleet_interfaces.srv import GetFleetStatus
from rcl_interfaces.msg import ParameterDescriptor


class FleetMonitor(Node):
    """
    Monitors all robots in the fleet and provides status aggregation.
    
    This node demonstrates:
    - Subscribing to multiple namespaced topics
    - Aggregating data from multiple sources
    - Providing services for queries
    - Configurable alerting behavior
    """
    
    def __init__(self):
        super().__init__('fleet_monitor')
        
        # ============================================
        # PARAMETERS
        # ============================================
        self.declare_parameter('monitor_rate', 2.0,
            ParameterDescriptor(description='How often to log fleet summary (Hz)'))
        
        self.declare_parameter('alert_on_low_battery', True,
            ParameterDescriptor(description='Log warnings for low battery'))
        
        self.declare_parameter('alert_on_errors', True,
            ParameterDescriptor(description='Log errors from robots'))
        
        self.declare_parameter('low_battery_fleet_threshold', 30.0,
            ParameterDescriptor(description='Battery % that triggers warning'))
        
        # NEW: Parameter for robot namespaces
        self.declare_parameter('robot_namespaces', ['robot_alpha', 'robot_beta', 'robot_gamma'],
            ParameterDescriptor(description='List of robot namespaces to monitor'))
        
        # Get parameter values
        self.monitor_rate = self.get_parameter('monitor_rate').value
        self.alert_battery = self.get_parameter('alert_on_low_battery').value
        self.alert_errors = self.get_parameter('alert_on_errors').value
        self.battery_threshold = self.get_parameter('low_battery_fleet_threshold').value
        self.robot_namespaces = self.get_parameter('robot_namespaces').value
        
        # ============================================
        # SUBSCRIBERS - One for each robot namespace
        # ============================================
        # Create a list to hold all subscribers
        self.status_subscribers = []
        
        for namespace in self.robot_namespaces:
            # Subscribe to each robot's namespaced topic
            topic_name = f'/{namespace}/robot_status'
            sub = self.create_subscription(
                RobotStatus, 
                topic_name, 
                self.status_callback, 
                10
            )
            self.status_subscribers.append(sub)
            self.get_logger().info(f'📡 Subscribed to {topic_name}')
        
        # ============================================
        # PUBLISHER
        # ============================================
        # Publish commands to global topic (all robots will hear)
        self.command_pub = self.create_publisher(FleetCommand, '/fleet_command', 10)
        
        # ============================================
        # SERVICE
        # ============================================
        self.status_srv = self.create_service(
            GetFleetStatus, 'get_fleet_status', self.get_fleet_status_callback)
        
        # ============================================
        # TIMER FOR PERIODIC MONITORING
        # ============================================
        self.timer = self.create_timer(1.0 / self.monitor_rate, self.monitor_callback)
        
        # ============================================
        # FLEET STATE STORAGE
        # ============================================
        # Dictionary to store latest status from each robot
        # Key: robot_id, Value: RobotStatus message
        self.robots = {}
        
        self.get_logger().info('👁️  Fleet Monitor started!')
        self.get_logger().info(f'Monitor Rate: {self.monitor_rate} Hz')
        self.get_logger().info(f'Battery Alert Threshold: {self.battery_threshold}%')
        self.get_logger().info(f'Monitoring {len(self.robot_namespaces)} robot namespaces')
    
    def status_callback(self, msg):
        """
        Receive status updates from any robot.
        
        This callback is triggered every time ANY robot publishes status.
        We store it in our dictionary for aggregation.
        """
        # Store or update this robot's status
        self.robots[msg.robot_id] = msg
        
        # ============================================
        # IMMEDIATE ALERTS
        # ============================================
        # Check for conditions that need immediate attention
        
        if self.alert_battery and msg.battery_percentage < self.battery_threshold:
            self.get_logger().warn(
                f'⚠️  {msg.robot_id}: Low battery ({msg.battery_percentage}%)')
        
        if self.alert_errors and msg.active_errors:
            self.get_logger().error(
                f'🚨 {msg.robot_id}: Errors: {", ".join(msg.active_errors)}')
    
    def monitor_callback(self):
        """
        Periodic fleet monitoring summary.
        
        Called by timer every 1/monitor_rate seconds.
        Logs aggregated fleet statistics.
        """
        if not self.robots:
            self.get_logger().info('⏳ Waiting for robots to report...')
            return
        
        # ============================================
        # CALCULATE FLEET STATISTICS
        # ============================================
        total = len(self.robots)
        operational = sum(1 for r in self.robots.values() if r.is_operational)
        on_task = sum(1 for r in self.robots.values() 
                     if r.task_status not in ['idle', 'charging'])
        charging = sum(1 for r in self.robots.values() if r.is_charging)
        with_errors = sum(1 for r in self.robots.values() if r.active_errors)
        
        avg_battery = sum(r.battery_percentage for r in self.robots.values()) / total
        
        # ============================================
        # LOG SUMMARY
        # ============================================
        self.get_logger().info(
            f'📊 Fleet: {total} robots | '
            f'✅ {operational} operational | '
            f'🔧 {on_task} on task | '
            f'🔌 {charging} charging | '
            f'🔋 {avg_battery:.1f}% avg battery'
        )
        
        if with_errors > 0:
            self.get_logger().warn(f'⚠️  {with_errors} robots have errors!')
    
    def get_fleet_status_callback(self, request, response):
        """
        Service handler for fleet status queries.
        
        Args:
            request: GetFleetStatus request (filters)
            response: GetFleetStatus response (to fill)
        
        Returns:
            Filled response with fleet statistics
        """
        # Start with all robots
        robots = list(self.robots.values())
        
        # ============================================
        # APPLY FILTERS
        # ============================================
        # Filter by robot type if specified
        if request.robot_type_filter:
            robots = [r for r in robots if r.robot_type == request.robot_type_filter]
        
        # Filter out offline robots if requested
        if not request.include_offline_robots:
            robots = [r for r in robots if r.is_operational]
        
        # ============================================
        # CALCULATE STATISTICS
        # ============================================
        response.total_robots = len(robots)
        response.operational_robots = sum(1 for r in robots if r.is_operational)
        response.robots_on_task = sum(1 for r in robots 
                                     if r.task_status not in ['idle', 'charging'])
        response.robots_charging = sum(1 for r in robots if r.is_charging)
        response.robots_with_errors = sum(1 for r in robots if r.active_errors)
        response.robot_ids = [r.robot_id for r in robots]
        
        # Calculate average battery (handle empty list)
        if robots:
            response.average_battery_level = (
                sum(r.battery_percentage for r in robots) / len(robots)
            )
        else:
            response.average_battery_level = 0.0
        
        self.get_logger().info(
            f'📞 Fleet status requested - {response.total_robots} robots match filters')
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FleetMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()