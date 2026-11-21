"""QoS profiles shared by the stack's nodes and tests."""
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# Latched topics (/tello/events, /tello/info, /tello/flight/segment, /sim/*/config): late joiners and the
# recorder still receive what was published before they started.
LATCHED = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
