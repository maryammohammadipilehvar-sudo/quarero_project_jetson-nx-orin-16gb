# Joystick controller input from PS5 controller via ESP32
# Analog stick values (typically -100 to 100 range)
int32 left_stick_forward   # Left stick forward/backward axis
int32 left_stick_right     # Left stick left/right axis
int32 right_stick_forward  # Right stick forward/backward axis
int32 right_stick_right    # Right stick left/right axis

# Button states (true when pressed)
bool select   # Select/Share button
bool start    # Start/Options button

# Face buttons
bool x        # X button
bool square   # Square button
bool triangle # Triangle button
bool circle   # Circle button

# Trigger and bumper buttons
bool l1       # Left bumper button
int32 l2      # Left trigger (analog, 0-255)
bool r1       # Right bumper button
int32 r2      # Right trigger (analog, 0-255)

# D-pad buttons
bool up       # D-pad up
bool down     # D-pad down
bool left     # D-pad left
bool right    # D-pad right