import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import time
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor

from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initialpose_publisher')
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # Create a timer to call publish_pose every 2 seconds
        self.timer = self.create_timer(2.0, self.publish_pose)  # Adjust the interval as needed
        self.get_logger().info('InitialPosePublisher node initialized.')

    def publish_pose(self):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
      
        pose_msg.pose.pose.position.x = -7.5952106530065
        pose_msg.pose.pose.position.y = 1.2578334448505635
        pose_msg.pose.pose.orientation.z = 0.0469387441221786
        pose_msg.pose.pose.orientation.w = 0.9988977696942929


        self.publisher.publish(pose_msg)
        
        self.get_logger().info('Published initial pose at x: %f, y: %f' %
                               (pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y))
       
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
          # Publish every 5 seconds
        self.get_logger().info('Goal Publisher Node Started!')

        self.subscription = self.create_subscription(
            PoseStamped,
            '/robot_position',  # This topic should be the current position of your robot
            self.position_callback1,
            10
        )

        

    def scan_callback(self, scan: LaserScan):
        """
        Callback function to handle LaserScan data.
        Detects if there's an obstacle (door) in front of the robot based on range data.
        """
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
            self.timer = self.create_timer(5.0, self.publish_goal)
    

    def publish_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        # goal.pose.position.x = -4.15  # Target position (x)
        # goal.pose.position.y = 2.26# Target position (y)
        # goal.pose.position.z = 1.61
        # goal.pose.orientation.w = 0.69
        # goal.pose.position.x = -0.53322  # Target position (x)
        # goal.pose.position.y = 5.2614# Target position (y)
        # goal.pose.position.z = 0.0
        # goal.pose.orientation.w = 0.9986
        goal.pose.position.x = -0.43322  # Target position (x)
        goal.pose.position.y = 5.4614# Target position (y)
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = -0.09
        goal.pose.orientation.w = 0.9986
        self.publisher_.publish(goal)
        self.get_logger().info(f'Published goal: x={goal.pose.position.x}, y={goal.pose.position.y}')
        
        
        
        self.target_position1 = PoseStamped()
        self.target_position1.pose.position.x = -0.43322
        self.target_position1.pose.position.y = 5.4614
        self.target_position1.pose.position.z = 0.0
        self.target_position1.pose.orientation.z = -0.09
        self.target_position1.pose.orientation.w = 0.9986
        # if (abs(msg.pose.pose.position.x - self.target_position1.pose.position.x) < 0.01 and
        #     abs(goal.pose.position.y - self.target_position1.pose.position.y) < 0.01 and
        #     abs(goal.pose.position.z - self.target_position1.pose.position.z) < 0.01 and
        #     abs(goal.pose.orientation.z - self.target_position1.pose.orientation.z) < 0.01 and
        #     abs(goal.pose.orientation.w - self.target_position1.pose.orientation.w) < 0.01):
        #   self.position_callback1()
        if (abs(goal.pose.position.x - self.target_position1.pose.position.x) < 0.01 and
            abs(goal.pose.position.y - self.target_position1.pose.position.y) < 0.01 and
            abs(goal.pose.position.z - self.target_position1.pose.position.z) < 0.01 and
            abs(goal.pose.orientation.z - self.target_position1.pose.orientation.z) < 0.01 and
            abs(goal.pose.orientation.w - self.target_position1.pose.orientation.w) < 0.01):
          self.position_callback1()
          
    def position_callback1(self):
      self.destroy_node()
      self.get_logger().info('destroyed__reach')
      
      
        
class SpawnObjectNode(Node):
    def __init__(self):
        super().__init__('spawn_object_node')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')
        
        self.get_logger().info("Ready to spawn objects")
        #self.spawn_object()

    def spawn_marker(self,  name, marker, pose):
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
        self.target_position = PoseStamped()
        self.target_position.pose.position.x = -0.43322 
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
        if (abs(msg.pose.pose.position.x - self.target_position.pose.position.x) < 0.01 and
            abs(msg.pose.pose.position.y - self.target_position.pose.position.y) < 0.01 and
            abs(msg.pose.pose.position.z - self.target_position.pose.position.z) < 0.01 and
            abs(msg.pose.pose.orientation.z - self.target_position.pose.orientation.z) < 0.01 and
            abs(msg.pose.pose.orientation.w - self.target_position.pose.orientation.w) < 0.01):
            
            self.get_logger().info('Target position reached!')

            # Your custom code when the target position is reached
            self.handle_position_reached()
        else:
          self.get_logger().info('not reached')
    def handle_position_reached(self):
        """
        Custom function to handle actions when the target position is reached.
        """
        # Your own logic here
        self.get_logger().info('Custom actions executed after reaching target position.')


def main(args=None):
    rclpy.init(args=args)
    initialpose = InitialPosePublisher()
    doordetect=DoorDetection()
    spawn = SpawnObjectNode()
    
    try:
        #rclpy.spin(initialpose)  # Keep the node running to publish periodically
        
        current_timestamp = int(time.time())
        #executor = MultiThreadedExecutor()
        positionalreach=PositionReachedSubscriber()
        #executor.add_node(positionalreach)
        

        pose = Pose()
        pose.orientation.w = 1.0
        
        id = 1
        pose.position.x = -4.348470
        pose.position.y = 3.1
        pose.position.z = 0.6
        spawn.spawn_object("box"+str(id),"box", 0.04, 0.06, 0.15 ,pose)
        pose.position.y -= 0.07
        pose.position.z -= 0.09
        spawn.spawn_marker("tag"+str(id),"Apriltag36_11_0000"+str(id),pose)

        id = 2
        pose.position.x = -4.291854
        pose.position.y = 3.204903
        pose.position.z = 0.6
        spawn.spawn_object("box"+str(id),"box", 0.04, 0.06, 0.15 ,pose)
        pose.position.y -= 0.07
        pose.position.z -= 0.09
        spawn.spawn_marker("tag"+str(id),"Apriltag36_11_0000"+str(id),pose)

        id = 3
        pose.position.x = -4.131414
        pose.position.y = 3.130812
        pose.position.z = 0.6
        spawn.spawn_object("box"+str(id),"box", 0.04, 0.06, 0.15 ,pose)
        pose.position.y -= 0.07
        pose.position.z -= 0.09
        spawn.spawn_marker("tag"+str(id),"Apriltag36_11_0000"+str(id),pose)

        id = 10
        pose.position.x = -2.761000
        pose.position.y = 4.128580
        pose.position.z = 0.510000
        spawn.spawn_marker("tag"+str(id),"Apriltag36_11_000"+str(id),pose)
        start_time = time.time()
        timeout = 10 

        while time.time() - start_time < timeout:
            rclpy.spin_once(initialpose)
        rclpy.spin(doordetect)
        rclpy.spin(spawn)
        rclpy.spin(positionalreach)
        #executor.spin()
        
    except KeyboardInterrupt:
        initialpose.get_logger().info('Shutting down node...')
        
    finally:
        rclpy.shutdown()
        #executor.shutdown()

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