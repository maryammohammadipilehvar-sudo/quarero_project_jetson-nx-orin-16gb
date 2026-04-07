# Schedule configuration message for tactical control
string schedule_id              # Unique ID for this schedule

# Timing configuration
uint8[] weekdays               # Array of weekdays (0=Monday, 6=Sunday)
string start_time              # Format: "HH:MM"
string end_time                # Format: "HH:MM"

# Route configuration
string[] route_names           # List of route filenames (without .yaml)
uint8[] route_repetitions      # Number of repetitions for each route (same length as route_names)
uint8 route_mode               # 0=Sequential, 1=Random
bool loop_mode                 # True if entire schedule should loop

# Control flags
bool active                    # True if schedule is active
bool require_home_return       # True if robot should return to home after schedule completion

# Battery management (passed through)
uint8 battery_threshold        # Battery percentage (5-50)
float32 home_tolerance         # Distance tolerance in meters
bool auto_charge_return        # Enable automatic return on low battery

# Route execution modes
uint8 SEQUENTIAL = 0           # Execute routes in order
uint8 RANDOM = 1               # Execute routes in random order