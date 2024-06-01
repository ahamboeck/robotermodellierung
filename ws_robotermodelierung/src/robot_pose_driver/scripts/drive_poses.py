#!/usr/bin/env python
import rospy
import random
import moveit_commander
import sys

def drive_robot(move_group, joints):
    """Drive the robot to the given joint positions."""
    if joints:
        try:
            rospy.loginfo(f"Driving {move_group.get_name()} to pose: {joints}")
            move_group.go(joints, wait=True)
            move_group.stop()  # Ensures there is no residual movement
        except moveit_commander.MoveItCommanderException as e:
            rospy.logerr(f"Error moving {move_group.get_name()}: {str(e)}")

def generate_random_pose(num_joints):
    """Generate random joint values within the range -π to π, or a narrower range if needed."""
    # Assuming hypothetical joint limits if not known:
    joint_limits = [(-2.0, 2.0) for _ in range(num_joints)]
    return [random.uniform(*limit) for limit in joint_limits]

def random_test(move_group):
    """Perform random testing on a specified move group."""
    num_joints = len(move_group.get_active_joints())
    while not rospy.is_shutdown():
        random_pose = generate_random_pose(num_joints)
        rospy.loginfo(f"Testing {move_group.get_name()} with random joints: {random_pose}")
        drive_robot(move_group, random_pose)
        rospy.sleep(2)  # Pause for 2 seconds between tests

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_robot_driver', anonymous=True)
    
    rospy.loginfo("Starting moveit_robot_driver node...")
    rospy.sleep(2)
    
    move_group = moveit_commander.MoveGroupCommander("coco_robot_eef1")
    
    

    if len(sys.argv) > 1 and sys.argv[1] == 'test':
        random_test(move_group)
    else:
        rospy.loginfo("Running normal operation...")

if __name__ == '__main__':
    main()

