import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float64MultiArray
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import random
import time
import math

class TerrenceAutonomy(Node):
    def __init__(self):
        super().__init__('terrence_autonomy')
        
        # Publishers for your custom controller
        self.mode_pub = self.create_publisher(String, '/set_mode', 10)
        self.dig_cmd_pub = self.create_publisher(Float64MultiArray, '/dig_cmd', 10)
        
        # Nav2 API
        self.navigator = BasicNavigator()
        
    def set_mode(self, mode_str):
        msg = String()
        msg.data = mode_str
        self.mode_pub.publish(msg)
        self.get_logger().info(f"Transitioned to {mode_str} mode.")
        time.sleep(0.5) # Give the controller time to latch the state
        
    def send_joint_cmds(self, loader_pos, hopper_pos, door_pos=0.0):
        # NOTE: You will need to update your C++ controller to accept a 3rd array element for the door
        msg = Float64MultiArray()
        msg.data = [loader_pos, hopper_pos] 
        self.dig_cmd_pub.publish(msg)
        
    def get_random_pose(self):
        """Generates a random pose within a bounded area."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        
        # Adjust these bounds based on your specific Gazebo world size
        pose.pose.position.x = random.uniform(-5.0, 5.0)
        pose.pose.position.y = random.uniform(-5.0, 5.0)
        
        # Random yaw rotation
        yaw = random.uniform(-math.pi, math.pi)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def navigate_to_random_point(self):
        self.set_mode('DRIVE')
        
        while True:
            goal_pose = self.get_random_pose()
            self.get_logger().info(f"Navigating to X: {goal_pose.pose.position.x:.2f}, Y: {goal_pose.pose.position.y:.2f}")
            
            self.navigator.goToPose(goal_pose)
            
            while not self.navigator.isTaskComplete():
                # Optional: Check feedback here if you want to abort early
                pass
                
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Successfully reached destination.')
                break
            elif result == TaskResult.CANCELED:
                self.get_logger().warn('Goal was canceled. Retrying...')
            elif result == TaskResult.FAILED:
                self.get_logger().warn('Goal failed (likely obstacle or unreachable). Retrying...')

    def run_cycle(self):
        # Wait for Nav2 to fully come online
        self.navigator.waitUntilNav2Active()
        
        for cycle in range(1, 11):
            self.get_logger().info(f"--- STARTING CYCLE {cycle}/10 ---")
            
            # 1. Drive to Dig Location
            self.navigate_to_random_point()
            
            # 2. Dig Routine
            self.set_mode('DIG')
            self.get_logger().info("Scooping sand...")
            self.send_joint_cmds(loader_pos=1.0, hopper_pos=0.0) # Move loader down
            time.sleep(2.0)
            self.send_joint_cmds(loader_pos=0.0, hopper_pos=0.0) # Move loader back up
            time.sleep(2.0)
            
            # 3. Drive to Dump Location
            self.navigate_to_random_point()
            
            # 4. Dump Routine
            self.set_mode('DUMP')
            self.get_logger().info("Dumping payload...")
            # Assuming you add door_pos logic later
            self.send_joint_cmds(loader_pos=0.0, hopper_pos=0.7) # Raise hopper
            time.sleep(1.0)
            # self.send_joint_cmds(loader_pos=0.0, hopper_pos=0.7, door_pos=0.3) # Open door
            time.sleep(2.0)
            
            # Stash everything back
            self.send_joint_cmds(loader_pos=0.0, hopper_pos=0.0)
            time.sleep(2.0)

def main(args=None):
    rclpy.init(args=args)
    autonomy_node = TerrenceAutonomy()
    
    try:
        autonomy_node.run_cycle()
    except KeyboardInterrupt:
        autonomy_node.get_logger().info("Autonomy script interrupted.")
    finally:
        autonomy_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()