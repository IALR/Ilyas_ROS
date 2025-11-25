# FleetCommand.msg
# Command to be sent to one or more robots in the fleet

# ============================================
# COMMAND IDENTIFICATION
# ============================================
string command_id                      # Unique command ID for tracking
builtin_interfaces/Time timestamp      # When command was issued

# ============================================
# TARGET SPECIFICATION - Which robots?
# ============================================
string[] target_robot_ids              # Specific robots (empty = all robots)
string robot_type_filter               # Filter by type (empty = all types)

# ============================================
# COMMAND DETAILS - What to do?
# ============================================
string command_type                    # "goto", "stop", "return_home", "charge", "task"
geometry_msgs/Point target_position    # Where to go (if applicable)
string task_description                # What task to perform
uint8 priority                         # 0-10, higher = more urgent

# ============================================
# EXECUTION PARAMETERS - How to do it?
# ============================================
float32 max_velocity                   # Speed limit for this command (m/s)
float32 timeout                        # Max time to complete (seconds)
bool allow_interruption                # Can this command be interrupted?