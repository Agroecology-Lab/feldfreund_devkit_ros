from rclpy.qos import DurabilityPolicy, Duration, LivelinessPolicy, QoSProfile, ReliabilityPolicy

SAFETY_QOS = QoSProfile(depth=1,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        liveliness=LivelinessPolicy.AUTOMATIC,
                        liveliness_lease_duration=Duration(seconds=1))
