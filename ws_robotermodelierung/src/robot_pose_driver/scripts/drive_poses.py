#!/usr/bin/env python
import rospy
import yaml
import os
import sys
import moveit_commander
import rospkg

def load_yaml(file_path):
    """Load and return data from a YAML file."""
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def drive_robot(move_group, joints):
    """Drive the specified move_group to the given joint positions."""
    if joints:
        try:
            rospy.loginfo(f"Driving {move_group.get_name()} to pose: {joints}")
            move_group.go(joints, wait=True)
            move_group.stop()  # Ensure no residual movement
        except moveit_commander.MoveItCommanderException as e:
            rospy.logerr(f"Error moving {move_group.get_name()}: {str(e)}")
    # rospy.sleep(1)  # Wait a bit to ensure the robot has reached the pose

def main():
    rospack = rospkg.RosPack()
    path = rospack.get_path('robot_pose_driver')  # Adjust this to your package name

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_robot_driver', anonymous=True)

    rospy.sleep(2)
    rospy.loginfo("Starting moveit_robot_driver node...")
    
    # Initialize MoveGroupCommander for each robot
    move_groups = {
        "coco_robot_eef1": moveit_commander.MoveGroupCommander("coco_robot_eef1"),
        "coco_robot_eef2": moveit_commander.MoveGroupCommander("coco_robot_eef2"),
        "alex_robot": moveit_commander.MoveGroupCommander("alex_robot"),
        "yuzi_robot": moveit_commander.MoveGroupCommander("yuzi_robot")
    }

    # Load movement sequence                    /home/ahamboeck/local_git/robotermodelierung/ws_robotermodelierung/src/robot_pose_driver/robot_poses/joint/move_sequence.yaml
    move_sequence = load_yaml(os.path.join(path, 'robot_poses/joint/move_sequence.yaml'))

    # Execute each move in the sequence
    while not rospy.is_shutdown() and move_sequence:
        for move in move_sequence:
            group_name = move['move_group']
            pose_name = move['pose']
            if group_name in move_groups:
                pose_file = os.path.join(path, f'robot_poses/joint/robot_poses_{group_name}.yaml')
                all_poses = load_yaml(pose_file)
                if all_poses:
                    # Find the correct pose by iterating and matching pose_name
                    for key, details in all_poses.items():
                        if details.get('pose_name') == pose_name:
                            rospy.loginfo(f"Moving {group_name} to {pose_name}: {details['pose_name']}")
                            drive_robot(move_groups[group_name], details['joints'])
                            break
                    else:
                        rospy.logerr(f"Pose {pose_name} not found in file {pose_file}")
                else:
                    rospy.logerr(f"No poses found in file {pose_file}")
            else:
                rospy.logerr(f"Move group {group_name} not defined")

if __name__ == '__main__':
    main()
