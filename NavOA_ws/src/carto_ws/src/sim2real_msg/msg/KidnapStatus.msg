# Foot-contact / kidnap status published by the leg odometry system.
#
# on_ground: at least one foot is currently in ground contact (torque/FK based).
#   true  = robot's feet are touching the ground
#   false = robot is airborne (e.g. being lifted / carried)
#
# kidnapped: latched flag that flips true once the robot has been lifted off the
#   ground and stays true after it is placed down again. It is the SLAM "the robot
#   was moved somewhere else" signal; reset is the responsibility of the consumer
#   (e.g. PoseGraph2D after re-localization), not this publisher.
#
# Semantics (normal / lifted / placed elsewhere):
#   on_ground=true,  kidnapped=false  -> normal, feet on ground, never lifted
#   on_ground=false, kidnapped=true   -> robot currently being lifted
#   on_ground=true,  kidnapped=true   -> robot placed down at a new location
bool on_ground
bool kidnapped
