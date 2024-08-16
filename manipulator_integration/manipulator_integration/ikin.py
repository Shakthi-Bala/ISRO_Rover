import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String
from rclpy.qos import QoSProfile
from isro_msgs.srv import Task5
from isro_msgs.msg import ObjectData
import time


class RoboticArmMotionPlanner(Node):
    def __init__(self):
        super().__init__('robotic_arm_motion_planner')

        # Publishers
        self.joint_publisher = self.create_publisher(Int32MultiArray, 'arm_sub', QoSProfile(depth=10))
        self.result_publisher = self.create_publisher(String, 'task_5_comp', QoSProfile(depth=10))

        # Service
        self.srv = self.create_service(Task5, 'task_5_srv', self.handle_service_request)

        # Subscribers
        self.feedback_subscription = self.create_subscription(Int32MultiArray, 'arm_state', self.feedback_callback, QoSProfile(depth=10))
        self.create_subscription(ObjectData, '/object_info', self.object_info_callback, 10)

        # Variables
        self.name = ''
        #self.target_x = 0
        #self.target_y = 0
        #self.target_z = 0.0
        self.current_joint_angles = [0, 0, 0, 0, 0]
        self.service_started = False
        self.request_name = ""
        self.request_number = 0

        # Initialize link lengths
        #self.L1 = 1.0
        #self.L2 = 1.0
        #self.L3 = 1.0
        #self.L4 = 1.0
        #self.L5 = 1.0

        # Hardcoded positions-Tuning
        self.pos1 = [1500,3050,1386,2647,3000]
        self.pos1_intermediate = [0,2610,1775,2645,3000]
        self.pos1_close= [0,3050,1386,2647,0]
        self.pos2 = [1000,3160,1419,2720,3763]
        self.pos2_intermediate = [2344,2570,1707,2727,3763]
        self.pos2_close =[1000,3160,1419,2720,0]
        self.pos3 = [1600,3046, 1869,1334, 3763]
        self.pos3_intermediate = [1900,2941,2104,1205,3763]
        self.pos3_close =[1200,3046,1869,1334,50]
        self.home_position = [1000, 1000, 2932, 2005, 700]
        self.home_container_position = [1000,1000,2932,2005,3000]
        self.drop_position = [1200,2600,1922,1655,20]
        self.drop_open = [1200,2600,1922,1655,3000]
        #self.drop_position_intermediate = [1900,2485,2309,1491,0]
        
    

        # Timer to periodically check the service status
        self.timer = self.create_timer(0.01, self.timer_callback)

    def handle_service_request(self, request, response):
        self.request_name = request.name
        self.request_number = request.samplenum
        if request.name in ["sample","container"]:
            self.service_started = True
            response.result = True
        else:
            response.result = False
        return response

    def timer_callback(self):
        if self.service_started:
            if self.request_name == "sample":
                if self.request_number == 1:
                    self.execute_pos1()
                    #self.execute_inverse_kinematics(self.target_x, self.target_y, self.target_z)
                elif self.request_number == 2:
                    self.execute_pos2()
                elif self.request_number == 3:
                    self.execute_pos3()
            elif self.request_name == "container":
                self.execute_sample_drop()

            self.service_started = False
            self.reset_variables()
            pub_data = "Task 5 Completed " + self.request_name
            self.result_publisher.publish(String(data=pub_data))

    def object_info_callback(self, msg):
        self.name = msg.name
        #self.target_x = msg.center_x
        #self.target_y = msg.center_y
        #self.target_z = msg.depth
        self.get_logger().info(f'Received object info: name={self.name}')

    def feedback_callback(self, msg):
        self.current_joint_angles = msg.data

    def reset_variables(self):
        self.name = ''
        #self.target_x = 0Object_LocationObject_Location
        #self.target_y = 0
        #self.target_z = 0.0

    
    # def map_value(self, x, in_min, in_max, out_min, out_max):
    #     if x == 0:
    #         return 1035
    #     elif x == -90:
    #         return 2058
    #     elif x < 0:
    #         y = 90 + x
    #         mapped_value = ((y - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min
    #         return int(mapped_value)
    #     else:
    #         return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

    # def execute_inverse_kinematics(self, x, y, z):
    #     try:
    #         theta0, theta1, theta2, theta3, theta4 = self.inverse_kinematics(x, y, z)
    #     except ValueError as e:
    #         self.get_logger().error(str(e))
    #         return

    #     # Convert joint angles to degrees
    #     theta0_degree = math.degrees(theta0)
    #     theta1_degree = math.degrees(theta1)
    #     theta2_degree = math.degrees(theta2)
    #     theta3_degree = math.degrees(theta3)
    #     theta4_degree = math.degrees(theta4)

    #     # Convert degrees to encoder values and map to the range [1030, 3065]
    #     theta0_enc = self.map_value(theta0_degree, 0, 180, 1030, 3065)
    #     theta1_enc = self.map_value(theta1_degree, 0, 180, 1030, 3065)
    #     theta2_enc = self.map_value(theta2_degree, 0, 180, 1030, 3065)
    #     theta3_enc = self.map_value(theta3_degree, 0, 180, 1030, 3065)
    #     theta4_enc = self.map_value(theta4_degree, 0, 180, 1030, 3065)

    #     # Ensure all encoder values are integers
    #     theta0_enc = int(theta0_enc)
    #     theta1_enc = int(theta1_enc)
    #     theta2_enc = int(theta2_enc)
    #     theta3_enc = int(theta3_enc)
    #     theta4_enc = int(theta4_enc)

    #     # Move to the initial position with calculated angles
    #     self.move_joints_step_by_step([theta0_enc, theta1_enc, theta2_enc, theta3_enc, theta4_enc])


        # self.joint_publisher.publish(Int32MultiArray(data=[theta4_enc]))

        # self.move_joints_step_by_step(self.home_position)

        # self.move_joints_step_by_step(self.hardcoded_position)

        # self.joint_publisher.publish(Int32MultiArray(data=[theta0_enc, theta1_enc, theta2_enc, theta3_enc, theta4_enc]))

        # # Return to home position
        # self.move_joints_step_by_step(self.home_position)

    def move_joints_step_by_step(self, target_joint_angles):
        # step_size = 100
        # while self.current_joint_angles != target_joint_angles:
        #     for i in range(len(target_joint_angles)):
        #         if self.current_joint_angles[i] < target_joint_angles[i]:
        #             self.current_joint_angles[i] += step_size
        #             if self.current_joint_angles[i] > target_joint_angles[i]:
        #                 self.current_joint_angles[i] = target_joint_angles[i]
        #         elif self.current_joint_angles[i] > target_joint_angles[i]:
        #             self.current_joint_angles[i] -= step_size
        #             if self.current_joint_angles[i] < target_joint_angles[i]:
        #                 self.current_joint_angles[i] = target_joint_angles[i]

            # Publish joint angles as an Int32MultiArray
            joint_angles_msg = Int32MultiArray()
            joint_angles_msg.data = target_joint_angles
            self.joint_publisher.publish(joint_angles_msg)
            self.get_logger().info(f'Moving to: {target_joint_angles}')
            rclpy.spin_once(self, timeout_sec=0.1)

    def execute_pos1(self):
        self.move_joints_step_by_step(self.pos1_intermediate)
        time.sleep(5)
        self.move_joints_step_by_step(self.pos1)
        time.sleep(5)
        self.move_joints_step_by_step(self.pos1_close)
        time.sleep(5)
        self.move_joints_step_by_step(self.home_position)

    def execute_pos2(self):
        # Move joints step by step to hardcoded position
        self.move_joints_step_by_step(self.pos2_intermediate)
        time.sleep(5)
        self.move_joints_step_by_step(self.pos2)
        time.sleep(5)
        self.move_joints_step_by_step(self.pos2_close)
        time.sleep(5)
        self.move_joints_step_by_step(self.home_position)

    def execute_pos3(self):
        self.move_joints_step_by_step(self.pos3_intermediate)
        time.sleep(5)
        self.move_joints_step_by_step(self.pos3)
        time.sleep(5)
        self.move_joints_step_by_step(self.pos3_close)
        time.sleep(5)
        self.move_joints_step_by_step(self.home_position)

    def execute_sample_drop(self):
        #self.move_joints_step_by_step(self.drop_position_intermediate)
        #  time.sleep(5)
        self.move_joints_step_by_step(self.drop_position)
        time.sleep(5)
        self.move_joints_step_by_step(self.drop_open)
        time.sleep(5)
        self.move_joints_step_by_step(self.home_container_position)


def main(args=None):
    rclpy.init(args=args)
    node = RoboticArmMotionPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
