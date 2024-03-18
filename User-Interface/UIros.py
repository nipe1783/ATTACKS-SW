import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, LivelinessPolicy
from px4_msgs.msg import VehicleAttitude,BatteryStatus,VehicleGlobalPosition, VehicleLocalPosition, VehicleStatus  # Make sure this matches the actual message type
from std_msgs.msg import Float64MultiArray, String
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PyQt5.QtGui import QImage, QPixmap


class VehicleAttitudeSubscriber(Node):
    def __init__(self):
        super().__init__('vehicle_attitude_subscriber')

        custom_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            liveliness=LivelinessPolicy.AUTOMATIC
        )

        self.subscription = self.create_subscription(
            VehicleAttitude, 
            '/fmu/out/vehicle_attitude', 
            self.listener_callback, 
            custom_qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'{msg.q}')
        self.quat = msg.q
        
class BatteryStatusSubscriber(Node):
    def __init__(self):
        super().__init__('battery_status_subscriber')

        self.battery_remaining = 0

        custom_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            liveliness=LivelinessPolicy.AUTOMATIC
        )

        self.subscription = self.create_subscription(
            BatteryStatus, 
            '/fmu/out/battery_status', 
            self.listener_callback, 
            custom_qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # self.get_logger().info(f'Voltage: {msg.voltage_v} V\n')
        # self.get_logger().info(f'Current: {msg.current_a} A\n')
        self.get_logger().info(f'{msg.remaining*100}')
        self.battery_remaining = msg.remaining*100
                
class VehicleGlobalPositionSubscriber(Node):
    def __init__(self):
        super().__init__('vehicle_global_position_subscriber')

        custom_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            liveliness=LivelinessPolicy.AUTOMATIC
        )

        self.subscription = self.create_subscription(
            VehicleGlobalPosition, 
            '/fmu/out/vehicle_global_position', 
            self.listener_callback, 
            custom_qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Lattitude: {msg.lat}\n')
        self.get_logger().info(f'Longitude: {msg.lon}\n')
        self.get_logger().info(f'Altitude: {msg.alt}\n')


class VehicleLocalPositionSubscriber(Node):
    def __init__(self):
        super().__init__('vehicle_local_position_subscriber')

        custom_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            liveliness=LivelinessPolicy.AUTOMATIC
        )

        self.subscription = self.create_subscription(
            VehicleLocalPosition, 
            '/fmu/out/vehicle_local_position', 
            self.listener_callback, 
            custom_qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'X: {msg.x}\n')
        self.get_logger().info(f'Y: {msg.y}\n')
        self.get_logger().info(f'Z: {msg.z}\n')
        self.x_pos = msg.x
        self.y_pos = msg.y
        self.z_pos = msg.z
        self.x_vel = msg.vx
        self.y_vel = msg.vy
        self.z_vel = msg.vz

class RGV1TruthDataSubscriber(Node):
    def __init__(self):
        super().__init__('rgv1_truth_subscriber')

        custom_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            liveliness=LivelinessPolicy.AUTOMATIC
        )

        self.subscription = self.create_subscription(
            Float64MultiArray, 
            '/rgv1_truth/pose/uas_i_frame', 
            self.listener_callback, 
            custom_qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
       if len(msg.data) >= 3:
            x, y, z = msg.data[:3]
            print("LOCALIZING")
            self.get_logger().info(f'X: {x}\n')
            self.get_logger().info(f'Y: {y}\n')
            self.get_logger().info(f'Z: {z}\n')
            self.rgv_x = x
            self.rgv_y = y
            self.rgv_z = z
            self.rgv_type = "1 true"


class RGV2TruthDataSubscriber(Node):
    def __init__(self):
        super().__init__('rgv2_truth_subscriber')

        print("RGV2 NODE MADE")

        custom_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            liveliness=LivelinessPolicy.AUTOMATIC
        )

        self.subscription = self.create_subscription(
            Float64MultiArray,  
            '/rgv2_truth/pose/uas_i_frame', 
            self.listener_callback, 
            custom_qos_profile)
        self.subscription  # prevent unused variable warning
    def listener_callback(self, msg):
       if len(msg.data) >= 3:
            x, y, z = msg.data[:3]
            print("LOCALIZING")
            self.get_logger().info(f'X: {x}\n')
            self.get_logger().info(f'Y: {y}\n')
            self.get_logger().info(f'Z: {z}\n')
            self.rgv_x = x
            self.rgv_y = y
            self.rgv_z = z
            self.rgv_type = "2 true"

class RGV2CoarseLocalizationSubscriber(Node):
    def __init__(self):
        super().__init__('rgv2_coarse_localization_subscriber')


        custom_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            liveliness=LivelinessPolicy.AUTOMATIC
        )

        self.subscription = self.create_subscription(
            Float64MultiArray,  
            '/rgv2/state', 
            self.listener_callback, 
            custom_qos_profile)
        self.subscription  # prevent unused variable warning
    def listener_callback(self, msg):
       if len(msg.data) >= 3:
            x, y, z = msg.data[:3]
            print("LOCALIZING")
            self.get_logger().info(f'X: {x}\n')
            self.get_logger().info(f'Y: {y}\n')
            self.get_logger().info(f'Z: {z}\n')
            self.rgv_x = x
            self.rgv_y = y
            self.rgv_z = z
            self.rgv_type = "2 estimate"

class ClockSubscriber(Node):
    def __init__(self):
        super().__init__('clock_subscriber')


        custom_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            liveliness=LivelinessPolicy.AUTOMATIC
        )

        self.subscription = self.create_subscription(
            Clock,  
            '/clock', 
            self.listener_callback, 
            custom_qos_profile)
        self.subscription  # prevent unused variable warning
    def listener_callback(self, msg):
        self.get_logger().info(f'clock: {msg.clock.sec}\n')
        self.time_sec = msg.clock.sec


class MissionPhaseSubscriber(Node):
    def __init__(self):
        super().__init__('mission_phase_subscriber')


        custom_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            liveliness=LivelinessPolicy.AUTOMATIC
        )

        self.subscription = self.create_subscription(
            String,  
            '/mission/phase', 
            self.listener_callback, 
            custom_qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'PHASE: {msg.data}\n')
        self.mission_phase = msg.data

class ModeSubscriber(Node):
    def __init__(self):
        super().__init__('mode_subscriber')


        custom_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            liveliness=LivelinessPolicy.AUTOMATIC
        )

        self.subscription = self.create_subscription(
            VehicleStatus,  
            '/fmu/out/vehicle_status', 
            self.listener_callback, 
            custom_qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        mode_names = [
            'Manual', 'Altitude Control', 'Position Control',
            'Auto Mission', 'Auto Loiter', 'Auto Return to Launch', 'Position Slow',
            'Free5', 'Free4', 'Free3', 'Acro', 'Free2', 'Descend', 'Termination', 'Offboard'
        ]
        mode_index = msg.nav_state
        if mode_index < len(mode_names):
            mode_name = mode_names[mode_index]
            self.get_logger().info(f'MODE: {mode_name}')
            self.flight_mode = mode_name


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.bridge = CvBridge()

        custom_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            liveliness=LivelinessPolicy.AUTOMATIC
        )

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            custom_qos_profile)
        self.subscription

    def listener_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.image_data = cv_image
        print('Received an image message')
   
       


# def main(args=None):
#     rclpy.init(args=args)
    
#     attitude_node = VehicleAttitudeSubscriber()
#     battery_node = BatteryStatusSubscriber()
#     vehicle_global_pos_node = VehicleGlobalPositionSubscriber()
#     vehicle_local_pos_node = VehicleLocalPositionSubscriber()
#     rgv1_truth_node = RGV1TruthDataSubscriber()
#     rgv2_truth_node = RGV2TruthDataSubscriber()
#     rgv2_coarse_node = RGV2CoarseLocalizationSubscriber()
#     clock_node = ClockSubscriber()
#     phase_node = MissionPhaseSubscriber()
#     mode_node = ModeSubscriber()
#     image_node = ImageSubscriber()

#     while True: 
#         try:
#             rclpy.spin_once(attitude_node)
#             rclpy.spin_once(battery_node)
#             rclpy.spin_once(vehicle_global_pos_node)
#             rclpy.spin_once(vehicle_local_pos_node)
#             rclpy.spin_once(rgv1_truth_node)
#             rclpy.spin_once(rgv2_truth_node)
#             rclpy.spin_once(rgv2_coarse_node)
#             rclpy.spin_once(clock_node)
#             rclpy.spin_once(phase_node)
#             rclpy.spin_once(mode_node)
#             rclpy.spin_once(image_node)
#         except KeyboardInterrupt:
            
#             print("Exiting UI ROS Test...")
#             attitude_node.destroy_node()
#             battery_node.destroy_node()
#             vehicle_global_pos_node.destroy_node()
#             vehicle_local_pos_node.destroy_node()
#             rgv1_truth_node.destroy_node()
#             rgv2_truth_node.destroy_node()
#             rgv2_coarse_node.destroy_node()
#             clock_node.destroy_node()
#             phase_node.destroy_node()
#             mode_node.destroy_node()
#             image_node.destroy_node()
#             rclpy.shutdown()

# if __name__ == '__main__':
#     main()