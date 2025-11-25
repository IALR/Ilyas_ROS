# RobotStatus.msg
# Comprehensive robot status for fleet management

# ============================================
# HEADER - Timestamp and reference frame
# ============================================
std_msgs/Header header

# ============================================
# IDENTIFICATION - Who is this robot?
# ============================================
string robot_id            # Unique identifier (e.g., "alpha", "robot_01")
string robot_type          # Type category: "scout", "transporter", "cleaner"
string location            # Current zone/area (e.g., "Zone_A", "Warehouse_2")

# ============================================
# MOTION STATE - Where and how fast?
# ============================================
geometry_msgs/Pose pose        # Position and orientation in space
geometry_msgs/Twist velocity   # Linear and angular velocity

# ============================================
# BATTERY INFORMATION - Power status
# ============================================
float32 battery_voltage        # Current voltage (e.g., 12.6V)
float32 battery_current        # Current draw/charge (negative = charging)
uint8 battery_percentage       # 0-100%
bool is_charging               # Currently charging?
string battery_health          # "excellent", "good", "fair", "poor"

# ============================================
# TASK INFORMATION - What's the robot doing?
# ============================================
string current_task_id         # Unique task identifier
string task_status             # "idle", "assigned", "executing", "completed", "failed"
uint8 task_progress            # 0-100% completion

# ============================================
# SYSTEM HEALTH - Is everything OK?
# ============================================
bool is_operational            # Can the robot operate?
float32 cpu_temperature        # CPU temp in Celsius
uint8 network_signal           # WiFi/cellular signal strength 0-100
string[] active_warnings       # Non-critical issues
string[] active_errors         # Critical problems

# ============================================
# PERFORMANCE METRICS - Historical data
# ============================================
float32 distance_traveled      # Total meters traveled
uint32 tasks_completed         # Total tasks completed successfully