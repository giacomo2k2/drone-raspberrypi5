# Severity levels
uint8 SEVERITY_CRITICAL = 0  # Land NOW
uint8 SEVERITY_WARNING = 1   # Return to home
uint8 SEVERITY_CAUTION = 2   # Continue with restrictions

uint8 severity
string source          # "battery_monitor", "gps_monitor", etc.
string description     # Human-readable
string[] affected_capabilities  # ["autonomous_flight", "vision_guidance"]
builtin_interfaces/Time timestamp