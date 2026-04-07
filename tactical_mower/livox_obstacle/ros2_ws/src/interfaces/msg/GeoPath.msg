# A list of GPS coordinates forming a path
geometry_msgs/Point[] waypoints

# Driving mode along the path
uint8 mode

# Available modes:
uint8 ONCE = 0          # Travel the path only one time
uint8 LOOP = 1          # Start again from the beginning after reaching the end
uint8 PING_PONG = 2     # Travel forward, then backward repeatedly
uint8 STOP = 4          # Stop Execution
