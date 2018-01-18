#
# This message describes a generic 2D control command.
#

# Message header.
Header header

# If true, the listener should only expect message updates when the control
# command changes (e.g. when generated from joystick input).
# If false, the listener should messages to be published at a regular interval.
bool lazy_publishing

# X axis command. 
float64 x

# Y axis command.
float64 y
