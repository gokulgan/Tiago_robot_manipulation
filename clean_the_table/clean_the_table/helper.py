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

class MarkerReach(Node):
    def __init__(self,position, orientation, node_name='reach_marker'):
        super().__init__(node_name)
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
        self.tagPose = position
        self.tagorient=orientation
        self.goal_reached = False
        self.client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.create_timer(1.0, self.on_timer)
        
        #
    
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
    
    def on_timer(self):
        if self.goal_reached or self.timer_counter >= self.max_calls:
            self.get_logger().info("Stopping timer after 5 executions.50")
            if self._timer:  # Check if the timer exists
                self._timer.cancel()# Stop the timer
            self.destroy_node()
             
            return
        try:
            
            # Lookup the transform between the target frame (base_link) and the source frame (AprilTag
            self.get_logger().info(f"NEW_Marker position: {self.tagPose}")
            self.get_logger().info(f"NEW_Marker orientation: {self.tagorient}")
            
            # Calculate arm pose based on the marker position2
            base_arm_pos=self.tagPose
            arm_pos,arm_ori = self.__calculate_arm_pose(self.tagPose, self.tagorient)
            self.send_goal(0.10, 0.10)
            # Move the arm to the calculated pose
            self.__move_arm_to_pose(position=arm_pos, orientation=arm_ori)
            self.get_logger().info("arm_movement_begins")

            self.get_logger().info("arm_movement_ended")
            self.goal_reached = True
            time.sleep(10)
            self.send_goal(0.018, 0.018)
            time.sleep(7)
            arm_position2 = [base_arm_pos[0]-0.19997513773, base_arm_pos[1]+0.00299394435, base_arm_pos[2]+0.10089152938]
            
            self.__move_arm_to_pose(position=arm_position2, orientation=arm_ori)
            time.sleep(5)
            arm_position3 = [base_arm_pos[0]-0.19997513773, base_arm_pos[1]+0.00299394435, base_arm_pos[2]+0.080889152938]
            
            self.__move_arm_to_pose(position=arm_position3, orientation=arm_ori)
            
            
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



class CollisionObjectExample(Node):

    def __init__(self, Mposition, Morientation):
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





def main(args=None):
    rclpy.init(args=args)
    try:
        executor = MultiThreadedExecutor()
        detector = marker_detect()
        #collision=CollisionObjectExample()
        #executor.add_node(detector)
        start_time = time.time()
        timeout = 10 

        while time.time() - start_time < timeout:
            rclpy.spin_once(detector)
            pose = detector.getPoseOfTag()
            if pose is not None:
                break  # Exit the loop if pose is found

        if pose is not None:
            position, orientation = pose
            reach = MarkerReach(position, orientation)
            
            position, orientation = pose
            time.sleep(5)
            collision = CollisionObjectExample(position, orientation)
            executor.add_node(collision)
            start_time = time.time()
            timeout1 = 10 
            while time.time() - start_time < timeout1:
                rclpy.spin_once(collision, timeout_sec=1)
            executor.add_node(reach)
            executor.spin()
            
            reach.destroy_node()

    finally:
        rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()


#node = GripperActionClient()
#cd ros2_ws/src/gg-246842/stage_6/pick_and_place/pick_and_place/