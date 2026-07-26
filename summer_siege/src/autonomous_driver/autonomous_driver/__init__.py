#!/usr/bin/env python3
import time 
import rclpy 
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
import math 

# Checkpoint coordinate matrix
checkpoints = [
    [-2.0, 0.0], [-2.0, -1.0], [0.0, -2.0], [2.0, 0.0], 
    [0.0, 2.0], [-0.5, 0.5], [0.5, 0.5], [-2.5, -0.5]
]

def bot_trip(x_g, y_g, checkpoint_no, navigator):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()

    goal_pose.pose.position.x = x_g
    goal_pose.pose.position.y = y_g
    goal_pose.pose.orientation.w = 1.0
    
    print(f"\n[START] Commanding robot to Checkpoint {checkpoint_no}: [{x_g}, {y_g}]")
    navigator.goToPose(goal_pose)
    
    # 1. Reset feedback memory buffer to prevent reading old checkpoint values
    navigator.feedback = None

    # 2. Acknowledgment Gate: Wait for Nav2 to register the fresh task
    for _ in range(10):
        rclpy.spin_once(navigator, timeout_sec=0.05)
        if not navigator.isTaskComplete():
            break
        time.sleep(0.05)
    
    last_distance = None
    stall_timeout = 15.0 
    min_progress = 0.005 
    last_progress_time = navigator.get_clock().now()
    
    while not navigator.isTaskComplete():
        # 3. PUMP ROS 2 EVENTS: Keeps incoming feedback topics updated cleanly
        rclpy.spin_once(navigator, timeout_sec=0.05)
        
        feedback = navigator.getFeedback()
        if feedback:
            current_distance = feedback.distance_remaining

            curr_x = feedback.current_pose.pose.position.x
            curr_y = feedback.current_pose.pose.position.y
            q_z = feedback.current_pose.pose.orientation.z
            q_w = feedback.current_pose.pose.orientation.w

            # Reconstruct current robot yaw
            current_yaw = 2.0 * math.atan2(q_z, q_w)
            current_yaw = math.atan2(math.sin(current_yaw), math.cos(current_yaw))

            # 4. COMPUTE REAL TARGET YAW: Dynamic vector from current position to goal
            delta_x = x_g - curr_x
            delta_y = y_g - curr_y
            target_yaw = math.atan2(delta_y, delta_x)

            # Compute normalized heading error
            diff = target_yaw - current_yaw
            angle_error = abs(math.atan2(math.sin(diff), math.cos(diff)))

            print(f"Est Time: {feedback.estimated_time_remaining.sec}s | Dist: {current_distance:.2f}m | Heading: {math.degrees(current_yaw):.1f}° | Angle Error: {math.degrees(angle_error):.1f}°")

            # 5. SAFEGUARDED BUBBLE CHECK:
            # Ignore distance <= 0.001 to bypass the initial zero-distance frame
            if 0.001 < current_distance < 0.25 and angle_error < math.radians(30):
                print(f"[INFO] Checkpoint {checkpoint_no} bubble reached within +/-30° heading alignment. Advancing!")
                break

            current_time = navigator.get_clock().now()
            if last_distance is None:
                last_distance = current_distance
                last_progress_time = current_time
            elif (last_distance - current_distance) > min_progress:
                last_distance = current_distance
                last_progress_time = current_time
            else:
                elapsed_stall_time = (current_time - last_progress_time).nanoseconds / 1e9
                if elapsed_stall_time > stall_timeout:
                    print(f"[ALERT] Robot is stuck for {elapsed_stall_time:.1f}s! Aborting checkpoint {checkpoint_no}")
                    navigator.cancelTask()
                    navigator.clearLocalCostmap()
                    navigator.clearGlobalCostmap()
                    break       
        
        time.sleep(0.05)

    # Cooldown buffer to clear ROS 2 action client lifecycle states
    time.sleep(1.0) 
    return 

def main():
    rclpy.init()
    navigator = BasicNavigator()
    
    existing_pose = None

    def amcl_callback(msg):
        nonlocal existing_pose
        existing_pose = msg
    
    amcl_qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE
    )

    print("Localization alignment established. Starting mission routing.")
    pose_sub = navigator.create_subscription(
        PoseWithCovarianceStamped, 
        'amcl_pose',
        amcl_callback,
        amcl_qos
    )

    start_check_time = time.time()
    max_wait_seconds = 6.0
    while existing_pose is None: 
        elapsed = (time.time() - start_check_time)
        if elapsed > max_wait_seconds:
            break
        rclpy.spin_once(navigator, timeout_sec=0.1)
    
    navigator.destroy_subscription(pose_sub)

    if existing_pose is not None:
        print("[LIFECYCLE] Active robot localization detected out in the field")
        print(f"Current position: X:{existing_pose.pose.pose.position.x:.2f}, Y:{existing_pose.pose.pose.position.y:.2f}")
        print("-> Skipping initial spawn overwrite. Proceeding straight to mission routing.")
    else:
        print("[LIFECYCLE] AMCL silence detected. Executing fresh spawn localization ... ")
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = rclpy.time.Time().to_msg()

        initial_pose.pose.position.x = -2.0
        initial_pose.pose.position.y = -0.5
        initial_pose.pose.orientation.w = 1.0
    
        for i in range(5):
            print(f" Sending initial pose broadcast [{i+1}/5]...")
            navigator.setInitialPose(initial_pose)
            rclpy.spin_once(navigator, timeout_sec=0.1)
            time.sleep(1.0)
        
    print("Mission routing initialized.")    
    for i in range(len(checkpoints)):
        bot_trip(checkpoints[i][0], checkpoints[i][1], i + 1, navigator)
        time.sleep(1.0)
    
    rclpy.shutdown() 

if __name__ == '__main__':
    main()