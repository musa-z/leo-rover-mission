"""
================================================================================
COMPONENT: Navigation Controller Node (Real Robot Version)
================================================================================
Design Philosophy:
    Acts as a smart actuator. Manages its own exploration loop and targeted 
    navigation, strictly using the topics defined by the Master FSM.
    
Interface Contract (Strictly adhering to Master FSM):
    - Sub: /nav/cmd_explore (Bool) -> True=Explore Loop, False=Stop/Brake
    - Sub: /nav/goal_point (PointStamped) -> Target destination from FSM
    - Pub: /nav/goal_reached (Bool) -> True when GOTO task completes
================================================================================
"""

import math
import random
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener, TransformException

class NavControllerNode(Node):
    def __init__(self):
        super().__init__('nav_controller_node', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, False)])
        
        self.explore_bounds = [-1.5, 1.5] 
        self.standoff_dist = 0.25         
        
        self.nav_mode = "IDLE"  # "IDLE", "EXPLORE", "GOTO"
        self.goal_handle = None 
        
        self.last_target_x = 999.0 
        self.last_target_y = 999.0 

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 is active! Ready for FSM commands.')

        self.sub_cmd_explore = self.create_subscription(Bool, '/nav/cmd_explore', self.explore_cb, 10)
        self.sub_goal_point  = self.create_subscription(PointStamped, '/nav/goal_point', self.point_cb, 10)
        self.pub_fb = self.create_publisher(Bool, '/nav/goal_reached', 10)

    # ==========================================================================
    # INPUT INTERFACES
    # ==========================================================================
    def explore_cb(self, msg: Bool):
        """Toggles exploration loop or forces an immediate stop."""
        if msg.data and self.nav_mode != "EXPLORE":
            self.nav_mode = "EXPLORE"
            self.dispatch_goal()  # Call without args -> Triggers random generation
            
        elif not msg.data and self.nav_mode == "EXPLORE":
            self.nav_mode = "IDLE"
            if self.goal_handle: self.goal_handle.cancel_goal_async()

    def point_cb(self, msg: PointStamped):
        """Calculates Standoff Pose and dispatches targeted navigation."""
        tx, ty = msg.point.x, msg.point.y
        
        # Anti-spam filter (20cm tolerance)
        if math.hypot(tx - self.last_target_x, ty - self.last_target_y) < 0.2:
            return 
            
        self.nav_mode = "GOTO"
        self.last_target_x, self.last_target_y = tx, ty
        
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rx, ry = t.transform.translation.x, t.transform.translation.y
            yaw = math.atan2(ty - ry, tx - rx)
            dist = math.hypot(tx - rx, ty - ry)
            
            # Synthesize standoff coordinates
            gx = tx - self.standoff_dist * math.cos(yaw) if dist > self.standoff_dist else rx
            gy = ty - self.standoff_dist * math.sin(yaw) if dist > self.standoff_dist else ry
            
            self.dispatch_goal(gx, gy, yaw) # Call with args -> Triggers precise GOTO
            
        except TransformException as ex:
            self.get_logger().warn(f"TF Error: {ex}")

    # ==========================================================================
    # CORE: ALL-IN-ONE DISPATCHER
    # ==========================================================================
    def dispatch_goal(self, x=None, y=None, yaw=None):
        """
        Cancels active goals, generates random coordinates if none provided, 
        and dispatches the new PoseStamped to Nav2.
        """
        # 1. Auto-cancel previous task
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

        # 2. Auto-generate random coordinates if running in EXPLORE mode
        if x is None or y is None or yaw is None:
            x = random.uniform(self.explore_bounds[0], self.explore_bounds[1])
            y = random.uniform(self.explore_bounds[0], self.explore_bounds[1])
            yaw = random.uniform(-math.pi, math.pi)

        # 3. Assemble and dispatch
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp.sec = 0  # Bypass TF buffer delays on real robot
        
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.nav_client.send_goal_async(NavigateToPose.Goal(pose=pose)).add_done_callback(self.goal_response_cb)

    # ==========================================================================
    # ASYNC FEEDBACK CHAIN
    # ==========================================================================
    def goal_response_cb(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            # Auto-retry if an exploration point was invalid
            if self.nav_mode == "EXPLORE": self.dispatch_goal()
            return
            
        self.goal_handle.get_result_async().add_done_callback(self.result_cb)

    def result_cb(self, future):
        status = future.result().status
        self.goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            if self.nav_mode == "GOTO":
                self.pub_fb.publish(Bool(data=True))
                self.nav_mode = "IDLE" 
            elif self.nav_mode == "EXPLORE":
                self.dispatch_goal() # Auto-loop exploration
                
        # Retry on failure to prevent robot freezing
        elif status != GoalStatus.STATUS_CANCELED and self.nav_mode == "EXPLORE":
            self.dispatch_goal()

def main():
    rclpy.init()
    node = NavControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()