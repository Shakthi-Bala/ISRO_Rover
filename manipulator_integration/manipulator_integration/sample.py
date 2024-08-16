import math
import rclpy
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String
from rclpy.qos import QoSProfile
from isro_msgs.srv import Object_Location
from isro_msgs.msg import ObjectData

class RoboticArmMotionPlanner(Node):
    def __init__(self):
        super().__init__('robotic_arm_motion_planner')
        self.joint_publisher = self.create_publisher(Int32MultiArray, 'arm_sub', QoSProfile(depth=10))
        self.result_publisher = self.create_publisher(String, 'result', QoSProfile(depth=10))
        self.srv = self.create_service(Object_Location, 'task_3', self.start_service_callback)
        self.feedback_subscription = self.create_subscription(Int32MultiArray, 'arm_state', self.feedback_callback, QoSProfile(depth=10))
        self.service_started = False

        self.name = ''
        self.target_x = 0
        self.target_y = 0
        self.target_z = 0.0
        self.current_joint_angles = [0, 0, 0, 0, 0]

        # Initialize link lengths
        self.L1 = 1.0
        self.L2 = 1.0
        self.L3 = 1.0
        self.L4 = 1.0
        self.L5 = 1.0

        # Create the ObjectInfo message object
        self.object_info = ObjectData()

        # Timer callback to periodically update target positions from the object_info
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        if self.service_started:
            self.name = self.object_info.name
            self.target_x = self.object_info.centre_x
            self.target_y = self.object_info.centre_y
            self.target_z = self.object_info.depth
            self.get_logger().info(f'Updated targets from object info: ({self.target_x}, {self.target_y}, {self.target_z})')
            if self.name == "Pickup":
                self.execute_inverse_kinematics(self.target_x, self.target_y, self.target_z)
            elif self.name == "Drop":
                self.execute_hardcoded_position()
            else:
                self.get_logger().info("Invalid name given")            
            self.service_started = False
            self.reset_variables()

    def feedback_callback(self, msg):
        self.current_joint_angles = msg.data

    def start_service_callback(self, request, response):
        self.service_started = True
        self.object_info.name = request.name
        response.result = "task_3_comp"
        return response



    def reset_variables(self):
        self.name = ''
        self.target_x = 0
        self.target_y = 0
        self.target_z = 0.0

    def inverse_kinematics(self, x, y, z):
        d1 = self.L1
        a2 = self.L2
        a3 = self.L3
        d4 = self.L4

        # Calculate r
        r = math.sqrt(x**2 + y**2)

        # Calculate theta_0
        theta0 = 0.0

        # Calculate theta_1
        theta1 = math.atan2(y, x)

        # Check reachability
        if r > a2 + a3 or r < abs(a2 - a3):
            raise ValueError("The target position is out of reach")

        # Calculate the projection of the target point onto the plane of the first joint
        D = (r**2 + (z - d1)**2 - a2**2 - a3**2) / (2 * a2 * a3)

        # Ensure D is within the range [-1, 1] to avoid math domain errors
        D = min(1, max(-1, D))

        # Calculate theta_3
        theta3 = math.atan2(math.sqrt(1 - D**2), D)

        # Calculate theta_2
        theta2 = math.atan2(z - d1, r) - math.atan2(a3 * math.sin(theta3), a2 + a3 * math.cos(theta3))

        # Calculate theta_4 (based on orientation constraints, we can assume it is zero for simplification)
        theta4 = 0.0

        return theta0, theta1, theta2, theta3, theta4

    def map_value(self, x, in_min, in_max, out_min, out_max):
        if x == 0:
            return 1035
        elif x == -90:
            return 2058
        elif x < 0:
            y = 90 + x
            mapped_value = ((y - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min
            return int(mapped_value)
        else:
            return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

    def execute_inverse_kinematics(self, x, y, z):
        try:
            theta0, theta1, theta2, theta3, theta4 = self.inverse_kinematics(x, y, z)
        except ValueError as e:
            self.get_logger().error(str(e))
            return

        # Convert joint angles to degrees
        theta0_degree = math.degrees(theta0)
        theta1_degree = math.degrees(theta1)
        theta2_degree = math.degrees(theta2)
        theta3_degree = math.degrees(theta3)
        theta4_degree = math.degrees(theta4)

        # Convert degrees to encoder values and map to the range [1030, 3065]
        theta0_enc = self.map_value(theta0_degree, 0, 180, 1030, 3065)
        theta1_enc = self.map_value(theta1_degree, 0, 180, 1030, 3065)
        theta2_enc = self.map_value(theta2_degree, 0, 180, 1030, 3065)
        theta3_enc = self.map_value(theta3_degree, 0, 180, 1030, 3065)
        theta4_enc = self.map_value(theta4_degree, 0, 180, 1030, 3065)

        # Ensure all encoder values are integers
        target_joint_angles = [int(theta0_enc), int(theta1_enc), int(theta2_enc), int(theta3_enc), int(theta4_enc)]

        self.move_joints_step_by_step(target_joint_angles)

    def move_joints_step_by_step(self, target_joint_angles):
        step_size = 100
        while self.current_joint_angles != target_joint_angles:
            for i in range(len(target_joint_angles)):
                if self.current_joint_angles[i] < target_joint_angles[i]:
                    self.current_joint_angles[i] += step_size
                    if self.current_joint_angles[i] > target_joint_angles[i]:
                        self.current_joint_angles[i] = target_joint_angles[i]
                elif self.current_joint_angles[i] > target_joint_angles[i]:
                    self.current_joint_angles[i] -= step_size
                    if self.current_joint_angles[i] < target_joint_angles[i]:
                        self.current_joint_angles[i] = target_joint_angles[i]

            # Publish joint angles as an Int32MultiArray
            joint_angles_msg = Int32MultiArray()
            joint_angles_msg.data = self.current_joint_angles
            self.joint_publisher.publish(joint_angles_msg)
            self.get_logger().info(f'Moving to: {self.current_joint_angles}')
            rclpy.spin_once(self, timeout_sec=0.1)

    def execute_hardcoded_position(self):
        joint_angles = [1500, 1500, 1500, 1500, 1500]

        # Move joints step by step
        self.move_joints_step_by_step(joint_angles)

def main(args=None):
    rclpy.init(args=args)
    node = RoboticArmMotionPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
