import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import time
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor

from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose










from geometry_msgs.msg import Twist
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from pymoveit2 import MoveIt2
from pymoveit2.robots import tiago as robot
from rclpy.callback_groups import ReentrantCallbackGroup
import time
from geometry_msgs.msg import Pose
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration


from tf2_ros import TransformException
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from teleop_tools_msgs.action import Increment
from pymoveit2 import GripperInterface, MoveIt2
from pymoveit2.robots import tiago as robot
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SpawnEntity


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initialpose_publisher')
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # Create a timer to call publish_pose every 2 seconds
        self.timer = self.create_timer(2.0, self.publish_pose)  # Adjust the interval as needed
        self.counter = 0  # Counter to track the number of publications
        self.get_logger().info('InitialPosePublisher node initialized.')

    def publish_pose(self):
        if self.counter >= 2:
            self.get_logger().info('Stopping InitialPosePublisher after 2 publications.')
            self.timer.cancel()  # Stop the timer
            self.destroy_node()  # Destroy the node
            return

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
      
        pose_msg.pose.pose.position.x = -7.2952106530065
        pose_msg.pose.pose.position.y = 1.2578334448505635
        pose_msg.pose.pose.orientation.z = 0.0469387441221786
        pose_msg.pose.pose.orientation.w = 0.9988977696942929

        self.publisher.publish(pose_msg)
        self.counter += 1  # Increment the counter
        
        self.get_logger().info(f'Published initial pose {self.counter} at x: {pose_msg.pose.pose.position.x}, y: {pose_msg.pose.pose.position.y}')
       
class DoorDetection(Node):
    def __init__(self):
        super().__init__('door_detection')
        
        # Laser scan distance threshold to detect door/opening
        self.min_distance_threshold = 0.5  # Minimum distance to consider the door open

        # Subscriber for laser scan data
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_raw',  # Modify to match your laser scan topic
            self.scan_callback,
            10
        )
        
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.goal_published = False  # Flag to track if the goal has been published
        self.door_open_detected = False  # Flag to track if the door is detected as open
        self.get_logger().info('Goal Publisher Node Started!')

    def scan_callback(self, scan: LaserScan):
        """
        Callback function to handle LaserScan data.
        Detects if there's an obstacle (door) in front of the robot based on range data.
        """
        if self.door_open_detected:
            return  # Exit if the door is already detected as open

        if not scan.ranges:
            self.get_logger().warn("No valid data in LaserScan ranges.")
            return

        # Assume the front of the robot is at the middle index of the ranges array
        front_index = len(scan.ranges) // 2

        # Check if an obstacle is detected in the front of the robot (e.g., door)
        if scan.ranges[front_index] < self.min_distance_threshold:
            self.get_logger().info("Obstacle (door) detected in front. Door is closed.")
        else:
            self.get_logger().info("No obstacle detected. Door is open.")
            if not self.goal_published:
                self.publish_goal()
                self.goal_published = True  # Set the flag to True after publishing the goal
                self.door_open_detected = True  # Set the flag to True after detecting the door as open
                self.destroy_node()  # Stop and destroy the node

    def publish_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = -0.43322  # Target position (x)
        goal.pose.position.y = 5.4614   # Target position (y)
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = -0.09
        goal.pose.orientation.w = 0.9986
        self.publisher_.publish(goal)
        self.get_logger().info(f'Published goal: x={goal.pose.position.x}, y={goal.pose.position.y}')
        
class SpawnObjectNode(Node):
    def __init__(self):
        super().__init__('spawn_object_node')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')
        
        self.get_logger().info("Ready to spawn objects")
        #self.spawn_object()

    def spawn_marker(self,  name, marker, pose):
        #april_tag_path = "/home/gokul/iki_workspace/ros2_ws/src/gg-246842/models/april_tags"
        april_tag_path = "april_tags"
        with open(april_tag_path+ "/" + marker + '/model.sdf') as f:
            xml = f.read()
        
        request = SpawnEntity.Request()
        request.name = name
        request.xml = xml
        request.initial_pose = pose

        future = self.cli.call_async(request)
        future.add_done_callback(self.callback)

    def spawn_object(self, name, model_name, width, depth, height, pose, mass=0.1):
        request = SpawnEntity.Request()

        request.name = name
        request.xml = self.__get_box_sdf(model_name, width, depth, height, mass)
        request.initial_pose = pose

        future = self.cli.call_async(request)
        future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Successfully spawned object!')
            else:
                self.get_logger().error(f'Failed to spawn object: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'Exception: {str(e)}')

    def __get_box_sdf(self, model_name, x, y, z, mass, static=0):
        # Calculate inertia for a box
        inertia_xx = (mass / 12.0) * (y**2 + z**2)
        inertia_yy = (mass / 12.0) * (x**2 + z**2)
        inertia_zz = (mass / 12.0) * (x**2 + y**2)

        friction_coefficient = 2.0

        return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name="{model_name}">
    <pose>0 0 0.0 0 0 0</pose>
    <link name="{model_name}_link">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{inertia_xx}</ixx>
          <iyy>{inertia_yy}</iyy>
          <izz>{inertia_zz}</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>{x} {y} {z}</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>{friction_coefficient}</mu>
              <mu2>{friction_coefficient}</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="{model_name}_visual">
        <geometry>
          <box>
            <size>{x} {y} {z}</size>
          </box>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
"""
class PositionReachedSubscriber(Node):
    def __init__(self):
        super().__init__('position_reached_subscriber')
        
        # Target position
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',  # This topic should be the current position of your robot
            self.position_callback,
            10
        )
        self.client = ActionClient(self, Increment, "/head_controller/increment")
        self.source_frame = "tag36h11:1"
        self.target_frame = "base_footprint"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.poseOfTag = None
        
        # Wait for the action server to be available
        self.client.wait_for_server()
        self.get_logger().info("Action server available. Sending goal...")
        self.target_position = PoseStamped()
        self.target_position.pose.position.x = -0.73322 
        self.target_position.pose.position.y = 5.4614
        self.target_position.pose.position.z = 0.0  # Keeping z constant

        self.target_position.pose.orientation.z = -0.09
        self.target_position.pose.orientation.w = 0.9986
        self.get_logger().info('PositionReachedSubscriber node initialized.')
        

    def position_callback(self, msg):
        """
        Callback function to handle the current position of the robot.
        Checks if the robot has reached the target position.
        """
        # Check if the robot's current position is close to the target position
        if (abs(msg.pose.pose.position.x - self.target_position.pose.position.x) < 0.5 and
            abs(msg.pose.pose.position.y - self.target_position.pose.position.y) < 0.5 and
            abs(msg.pose.pose.position.z - self.target_position.pose.position.z) < 0.5 and
            abs(msg.pose.pose.orientation.z - self.target_position.pose.orientation.z) < 0.5 and
            abs(msg.pose.pose.orientation.w - self.target_position.pose.orientation.w) < 0.5):
            
            self.get_logger().info('Target position reached!')
            
            # Your custom code when the target position is reached
            self.handle_position_reached()
        else:
            self.get_logger().info('not reached')

    def handle_position_reached(self):
        """
        Custom function to handle actions when the target position is reached.
        """
        self.send_goal()
        self.timer = self.create_timer(1.0, self.on_timer)  # Create the timer once
        self.get_logger().info('Custom actions executed after reaching target position.')

    def send_goal(self):
        goal_msg = Increment.Goal()
        goal_msg.increment_by = [0.0, -0.5, 0.1]  # Yaw, Pitch, Roll adjustments

        self.get_logger().info(f"Sending goal: {goal_msg.increment_by}")

        # Send goal and set up a callback for result
        self.future = self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return

        self.get_logger().info("Goal accepted!")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Feedback received: {feedback_msg.feedback}")

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result received: {result}")
        #self.destroy_node()  # Stop the node after completing the task

    def on_timer(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time())
            marker_position = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]
            marker_orientation = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
            self.get_logger().info(f"Marker position: {marker_position}, orientation: {marker_orientation}")
            self.marker_detected = True  # Set the flag to True after detecting the marker
            self.timer.cancel()  # Stop the timer
            #self.destroy_node()  # Destroy the node after detecting the marker
        except TransformException as ex:
            self.get_logger().warn(f"Could not transform: {ex}")
        
        
        
        
        
        
        
        
        
        
        
class MarkerReach(Node):
    def __init__(self):
        super().__init__('node_name')
        self.source_frame = "tag36h11:1"
        self.target_frame = "base_footprint"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.poseOfTag = None

       
        self.timer = self.create_timer(1.0, self.on_timer1)
        self.__callback_group = ReentrantCallbackGroup()
        self.__synchronous = True
        self.__cartesian = False
        self.__cartesian_max_step = 0.0025
        self.__cartesian_fraction_threshold = 0.0
        # Initialize MoveIt2 for arm control
        self.__moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=self.__callback_group,
        )

        self.__moveit2.planner_id = "RRTConnectkConfigDefault" 
        self.__moveit2.max_velocity = 0.5
        self.__moveit2.max_acceleration = 0.5
        self.__moveit2.cartesian_avoid_collisions = False
        self.__moveit2.cartesian_jump_threshold = 0.0
        
        self.goal_reached = False
        self.timer_counter = 0  # Counter to track the number of calls
        self.max_calls = 1
        self._timer = None
        # self.tagPose = position
        # self.tagorient=orientation
        self.goal_reached = False
        self.client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        
        
        #
    def on_timer1(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time())
            marker_position = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]
            marker_orientation = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
            
            self.create_timer(1.0, self.on_timer(marker_position,marker_orientation))
            
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.source_frame} to {self.target_frame}: {ex}')
            #self.poseOfTag = (marker_position, marker_orientation)

    def qtoe(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        self.get_logger().info(f"roll={roll_deg}, pitch={pitch_deg}, yaw={yaw_deg}")
        
        return (roll_deg), (pitch_deg), (yaw_deg)
    def etoq(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr
        
        return [cy * cp * sr - sy * sp * cr,
            sy * cp * sr + cy * sp * cr,
            sy * cp * cr - cy * sp * sr,
            cy * cp * cr + sy * sp * sr]
    
    def on_timer(self, marker_position,marker_orientation):
        if self.goal_reached or self.timer_counter >= self.max_calls:
            self.get_logger().info("Stopping timer after 5 executions.50")
            if self._timer:  # Check if the timer exists
                self._timer.cancel()# Stop the timer
            self.destroy_node()
             
            return
        try:
            
            # Lookup the transform between the target frame (base_link) and the source frame (AprilTag
            self.get_logger().info(f"NEW_Marker position: {marker_position}")
            self.get_logger().info(f"NEW_Marker orientation: {marker_orientation}")
            
            # Calculate arm pose based on the marker position2
            base_arm_pos=marker_position
            arm_pos,arm_ori = self.__calculate_arm_pose(marker_position, marker_orientation)
            self.send_goal(0.10, 0.10)
            # Move the arm to the calculated pose
            self.__move_arm_to_pose(position=arm_pos, orientation=arm_ori)
            self.get_logger().info("arm_movement_begins")

            self.get_logger().info("arm_movement_ended")
            self.goal_reached = True
            time.sleep(10)
            self.send_goal(0.018, 0.018)
            time.sleep(7)
            # arm_position2 = [base_arm_pos[0]-0.19997513773, base_arm_pos[1]+0.00299394435, base_arm_pos[2]+0.10089152938]
            
            # self.__move_arm_to_pose(position=arm_position2, orientation=arm_ori)
            # time.sleep(5)
            # arm_position3 = [base_arm_pos[0]-0.19997513773, base_arm_pos[1]+0.00299394435, base_arm_pos[2]+0.080889152938]
            
            # self.__move_arm_to_pose(position=arm_position3, orientation=arm_ori)
            
            
        except TransformException as ex:
            self.get_logger().warn(f"Could not transform: {ex}")
    def __calculate_arm_pose(self, marker_position, marker_orientation):
        #count=1
        
        arm_position = [marker_position[0]-0.14947513773, marker_position[1]+0.03109394435, marker_position[2]+0.08089152938]
        
        #arm_orientation = [marker_orientation[0]-0.51898111897, marker_orientation[1]-0.42460217324, marker_orientation[2]-0.50794648603, marker_orientation[3]+0.53610456406]  
        #arm_orientation=[0,0,0,1]
        arm_orientation = [marker_orientation[0]-0.50794648603, marker_orientation[1]-0.42460217324, marker_orientation[2]-0.51898111897, marker_orientation[3]+0.53610456406]  
        
        
        
        #marker_orientation[0]=-0.5
        #marker_orientation[1]=0.0005
        #marker_orientation[2]=0.770
        #marker_orientation[3]=1.2
        #arm_orientation = [marker_orientation[0], marker_orientation[1], marker_orientation[2], marker_orientation[3]]  
        #arm_position = [
        #marker_position.x,
        #marker_position.y,
        #marker_position.z
    #]
    
    # Access the x, y, z, w attributes of the Quaternion object
        #arm_orientation = marker_orientation
        
        #self.timer_counter = 0  # Counter to track the nucd ros2_ws/src/gg-246842/stage_6/pick_and_place/pick_and_place/mber of calls
        
        #self._timer = Nonessss
        roll, pitch, yaw=self.qtoe(arm_orientation)
        neww,newz,newy,newx =self.etoq(roll, pitch+0.3, yaw)
        self.get_logger().info(f"massagedRoll={roll}, massagedPitch={pitch}, massagedYaw={yaw}")
        orientation_new=[newx,newy,newz,neww]
        self.tagPose=arm_position
        self.tagorient=orientation_new
        #self.get_logger(self.send_goal(0.04, 0.04).info(f"neworientation={orientation_new}")
        
        
        #arm_position = [marker_position[0]-marker_position[0]-0.01, marker_position[1]-marker_position[1]-0.015, marker_position[2]-marker_position[2]-0.4]

        #arm_orientation = [marker_pos[0]-marker_pos[0]-0.01, marker_pos[1]-marker_pos[1]-0.017, marker_pos[2]-marker_pos[2]-0.9, marker_pos[3]-marker_pos[3]-0.019] 


        self.get_logger().info(f"Calculated arm pose: position {arm_position}, orientation {orientation_new}")
        
        
        return arm_position, orientation_new
    
        
        
    def __move_arm_to_pose(self, position: list[float, float, float], orientation: list[float, float, float, float]):

        self.get_logger().info(f"Starting movement to position: {position}, orientation: {orientation}")

        try:
            # Start the move command
            self.__moveit2.move_to_pose(
                position=position,
                quat_xyzw=orientation,
                cartesian=self.__cartesian,
                cartesian_max_step=self.__cartesian_max_step,
                cartesian_fraction_threshold=self.__cartesian_fraction_threshold,
            )

            # Log before waiting
            self.get_logger().info("Waiting for execution to complete...")
            
            if self.__synchronous:
                # Wait for the movement to finish
                self.__moveit2.wait_until_executed()
                self.get_logger().info("Movement execution completed successfully.")
            else:
                self.get_logger().info("Asynchronous execution initiated.")
        
        
        except Exception as e:
            
            self.get_logger().error(f"Error during movement execution: {str(e)}")
    def send_goal(self, left_position: float, right_position: float):
        self.get_logger().info(f"Sending goal: Left={left_position}, Right={right_position}")

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]

        point = JointTrajectoryPoint()
        point.positions = [left_position, right_position]
        point.time_from_start = Duration(sec=1, nanosec=0)  # Move within 1 second

        goal_msg.trajectory.points.append(point)

        self.client.wait_for_server()
        future = self.client.send_goal_async(goal_msg)

        future.add_done_callback(self.goal_response_callback)
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected!")
            return

        self.get_logger().info("Goal accepted!")
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Goal finished with result: {result}")

class marker_detect(Node):

    def __init__(self):
        super().__init__('detect')
        self.source_frame = "tag36h11:1"
        self.target_frame = "base_footprint"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.poseOfTag = None

       
        self.timer = self.create_timer(1.0, self.on_timer)
    def on_timer(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time())
            marker_position = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]
            marker_orientation = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
        
            self.poseOfTag = (marker_position, marker_orientation)

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.source_frame} to {self.target_frame}: {ex}')
            return

    def getPoseOfTag(self):
        return self.poseOfTag




# class CollisionObjectExample(Node):

#     def __init__(self):
#         super().__init__("collision_object")
#         self.__callback_group = ReentrantCallbackGroup()

#         self.__moveit2 = MoveIt2(
#             node=self,
#             joint_names=robot.joint_names(),
#             base_link_name=robot.base_link_name(),
#             end_effector_name=robot.end_effector_name(),
#             group_name=robot.MOVE_GROUP_ARM,
#             callback_group=self.__callback_group,
#         )
        
#         #self.timer = self.create_timer(1.0, self.timer_callback)
#         self.subscription = self.create_subscription(
#             PoseWithCovarianceStamped,
#             '/amcl_pose',  # This topic should be the current position of your robot
#             self.timer_callback,
#             10
#         )
        
#         self.target_position = PoseStamped()
#         self.target_position.pose.position.x = -0.73322 
#         self.target_position.pose.position.y = 5.4614
#         self.target_position.pose.position.z = 0.0  # Keeping z constant

#         self.target_position.pose.orientation.z = -0.09
#         self.target_position.pose.orientation.w = 0.9986
#         self.Mpos=[-0.7332,5.4614,0.0]
#         self.Mori=[0.0,0.0,-0.09,0.9986]
#         self.get_logger().info("adding collision object")
#     def timer_callback(self, msg):
#         if (abs(msg.pose.pose.position.x - self.target_position.pose.position.x) < 0.5 and
#             abs(msg.pose.pose.position.y - self.target_position.pose.position.y) < 0.5 and
#             abs(msg.pose.pose.position.z - self.target_position.pose.position.z) < 0.5 and
#             abs(msg.pose.pose.orientation.z - self.target_position.pose.orientation.z) < 0.5 and
#             abs(msg.pose.pose.orientation.w - self.target_position.pose.orientation.w) < 0.5):
#             self.get_logger().info('Add Collisionobject')

#             # Sleeps are just for showcasing, not needed in production
            
#             arm_position = [self.Mpos[0]+0.255, self.Mpos[1], self.Mpos[2]-0.23]
#             self.get_logger().info(f"Calculated arm position: {arm_position}")
#             self.__add_collision_box(object_id="table", 
#                                             position=arm_position,
#                                             orientation=self.tagorient,
#                                             dimensions=[0.63, 0.5, 0.45])


#     def __add_collision_box(self, object_id:str, position:list[float, float, float], orientation:list[float, float, float, float], dimensions:list[float, float, float]):
 
#         self.get_logger().info(
#         f"Adding collision object: id={object_id}, position={position}, orientation={orientation}, dimensions={dimensions}"
#         )
#         self.__moveit2.add_collision_box(
#                 id=object_id,
#                 position=position,
#                 quat_xyzw=orientation,
#                 size=dimensions
#         )

class CollisionObjectExample(Node):

    def __init__(self):
        super().__init__("collision_object")
        self.__callback_group = ReentrantCallbackGroup()

        self.__moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=self.__callback_group,
        )
        Mposition=[-0.7332,5.4614,0.0]
        Morientation=[0.0,0.0,-0.09,0.9986]
        self.tagPose = Mposition
        self.tagorient=Morientation
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Add Collisionobject')

        # Sleeps are just for showcasing, not needed in production
        
        arm_position = [self.tagPose[0]+0.255, self.tagPose[1], self.tagPose[2]-0.23]
        self.get_logger().info(f"Calculated arm position: {arm_position}")
        self.__add_collision_box(object_id="table", 
                                        position=arm_position,
                                        orientation=self.tagorient,
                                        dimensions=[0.63, 0.5, 0.45])


    def __add_collision_box(self, object_id:str, position:list[float, float, float], orientation:list[float, float, float, float], dimensions:list[float, float, float]):
 
        self.get_logger().info(
        f"Adding collision object: id={object_id}, position={position}, orientation={orientation}, dimensions={dimensions}"
        )
        self.__moveit2.add_collision_box(
                id=object_id,
                position=position,
                quat_xyzw=orientation,
                size=dimensions
        )

def main(args=None):
    rclpy.init(args=args)
    
    # Create instances of your nodes
    initialpose = InitialPosePublisher()
    doordetect = DoorDetection()
    spawn = SpawnObjectNode()
    #collision = CollisionObjectExample()  # Instantiate the class
    reach = MarkerReach()  # Instantiate the class
    positionalreach = PositionReachedSubscriber()
    
    # Create a MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    
    # Add nodes to the executor
    executor.add_node(initialpose)
    executor.add_node(spawn)
    
    try:
        # Spawn objects
        pose = Pose()
        pose.orientation.w = 1.0
        
        id = 1
        pose.position.x = -4.348470
        pose.position.y = 3.1
        pose.position.z = 0.6
        spawn.spawn_object("box"+str(id), "box", 0.04, 0.06, 0.15, pose)
        pose.position.y -= 0.07
        pose.position.z -= 0.09
        spawn.spawn_marker("tag"+str(id), "Apriltag36_11_0000"+str(id), pose)

        id = 2
        pose.position.x = -4.291854
        pose.position.y = 3.204903
        pose.position.z = 0.6
        spawn.spawn_object("box"+str(id), "box", 0.04, 0.06, 0.15, pose)
        pose.position.y -= 0.07
        pose.position.z -= 0.09
        spawn.spawn_marker("tag"+str(id), "Apriltag36_11_0000"+str(id), pose)

        id = 3
        pose.position.x = -4.131414
        pose.position.y = 3.130812
        pose.position.z = 0.6
        spawn.spawn_object("box"+str(id), "box", 0.04, 0.06, 0.15, pose)
        pose.position.y -= 0.07
        pose.position.z -= 0.09
        spawn.spawn_marker("tag"+str(id), "Apriltag36_11_0000"+str(id), pose)

        id = 10
        pose.position.x = -2.761000
        pose.position.y = 4.128580
        pose.position.z = 0.510000
        spawn.spawn_marker("tag"+str(id), "Apriltag36_11_000"+str(id), pose)
        
        # Wait for spawning to complete
        start_time = time.time()
        timeout = 10
        while time.time() - start_time < timeout:
            executor.spin_once(timeout_sec=1.0)
        
        #executor.add_node(collision)
        start_time = time.time()
        timeout1 = 10 
        
        #rclpy.spin_once(collision)    
        executor.add_node(doordetect)
        executor.add_node(positionalreach)
        executor.add_node(reach)
        #executor.add_node(collision)
        
        while time.time() - start_time < timeout:
            executor.spin_once(timeout_sec=1.0)
            
        #executor.add_node(collision)
        #executor.add_node(reach)
        
        # Spin the executor to keep all nodes running
        executor.spin()
        
    except KeyboardInterrupt:
        initialpose.get_logger().info('Shutting down nodes...')
        
    finally:
        # Shutdown the executor and nodes
        executor.shutdown()
        initialpose.destroy_node()
        doordetect.destroy_node()
        spawn.destroy_node()
        positionalreach.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



#ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "name: 'door'"
#ros2 run teleop_twist_keyboard teleop_twist_keyboard







# def main(args=None):
#     rclpy.init(args=args)
    
#     # Create instances of your nodes
#     initialpose = InitialPosePublisher()
#     doordetect = DoorDetection()
#     spawn = SpawnObjectNode()
#     positionalreach = PositionReachedSubscriber()
    
#     # Create a MultiThreadedExecutor
#     executor = MultiThreadedExecutor()
    
#     # Add nodes to the executor
#     executor.add_node(initialpose)
#     executor.add_node(doordetect)
#     executor.add_node(spawn)
#     executor.add_node(positionalreach)
    
#     try:
#         # Spawn objects
#         pose = Pose()
#         pose.orientation.w = 1.0
        
#         id = 1
#         pose.position.x = -4.348470
#         pose.position.y = 3.1
#         pose.position.z = 0.6
#         spawn.spawn_object("box"+str(id), "box", 0.04, 0.06, 0.15, pose)
#         pose.position.y -= 0.07
#         pose.position.z -= 0.09
#         spawn.spawn_marker("tag"+str(id), "Apriltag36_11_0000"+str(id), pose)

#         id = 2
#         pose.position.x = -4.291854
#         pose.position.y = 3.204903
#         pose.position.z = 0.6
#         spawn.spawn_object("box"+str(id), "box", 0.04, 0.06, 0.15, pose)
#         pose.position.y -= 0.07
#         pose.position.z -= 0.09
#         spawn.spawn_marker("tag"+str(id), "Apriltag36_11_0000"+str(id), pose)

#         id = 3
#         pose.position.x = -4.131414
#         pose.position.y = 3.130812
#         pose.position.z = 0.6
#         spawn.spawn_object("box"+str(id), "box", 0.04, 0.06, 0.15, pose)
#         pose.position.y -= 0.07
#         pose.position.z -= 0.09
#         spawn.spawn_marker("tag"+str(id), "Apriltag36_11_0000"+str(id), pose)

#         id = 10
#         pose.position.x = -2.761000
#         pose.position.y = 4.128580
#         pose.position.z = 0.510000
#         spawn.spawn_marker("tag"+str(id), "Apriltag36_11_000"+str(id), pose)
        
#         # Spin the executor to keep all nodes running
#         executor.spin()
        
#     except KeyboardInterrupt:
#         initialpose.get_logger().info('Shutting down nodes...')
        
#     finally:
#         # Shutdown the executor and nodes
#         executor.shutdown()
#         initialpose.destroy_node()
#         doordetect.destroy_node()
#         spawn.destroy_node()
#         positionalreach.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#      main()