#!/usr/bin/env python3
import argparse
import time
from typing import List

import rospy
import rospkg
import yaml
from std_srvs.srv import Trigger
from pca9685_servo_control.srv import SetAngle

def load_poses(poses_ns: str):
    # Try read from ROS param first (loaded by launch), else fallback to package file
    if rospy.has_param(f"{poses_ns}/poses") and rospy.has_param(f"{poses_ns}/sequence"):
        poses = rospy.get_param(f"{poses_ns}/poses")
        seq = rospy.get_param(f"{poses_ns}/sequence")
        return poses, seq
    # Fallback: load config/demo_poses.yaml directly
    rp = rospkg.RosPack()
    path = rp.get_path("pca9685_servo_control") + "/config/demo_poses.yaml"
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    return data["poses"], data["sequence"]

def go_pose(set_angle, channels: List[int], angles: List[float], dwell: float):
    for ch, a in zip(channels, angles):
        set_angle(ch, float(a))
        time.sleep(dwell)

def main():
    parser = argparse.ArgumentParser(description="Named pose sequence demo for PCA9685 servos")
    parser.add_argument("--ns", default="/pca9685_servo", help="Namespace for services (default: /pca9685_servo)")
    parser.add_argument("--poses_ns", default="/pca9685_demo", help="ROS param namespace for poses/sequence")
    parser.add_argument("--repeat", type=int, default=1, help="Repeat count for the sequence")
    parser.add_argument("--hold_scale", type=float, default=1.0, help="Scale timing for hold_sec in sequence")
    parser.add_argument("--settle", type=float, default=0.05, help="Settle time per joint when sending pose")
    args = parser.parse_args()

    rospy.init_node("pca9685_demo_sequence", anonymous=True)
    set_angle_srv = f"{args.ns}/set_angle"
    enable_srv = f"{args.ns}/enable"
    home_srv = f"{args.ns}/home"

    rospy.loginfo("Waiting for services %s, %s, %s", set_angle_srv, enable_srv, home_srv)
    rospy.wait_for_service(set_angle_srv)
    rospy.wait_for_service(enable_srv)
    rospy.wait_for_service(home_srv)

    set_angle = rospy.ServiceProxy(set_angle_srv, SetAngle)
    enable = rospy.ServiceProxy(enable_srv, Trigger)
    home = rospy.ServiceProxy(home_srv, Trigger)

    # Load poses and sequence
    poses, seq = load_poses(args.poses_ns)
    channels = [0, 1, 2, 3, 4]  # must match config/servos.yaml

    # Enable and go home
    enable()
    home()

    for r in range(args.repeat):
        rospy.loginfo("Starting sequence run %d/%d", r+1, args.repeat)
        for step in seq:
            pose_name = step["pose"]
            hold = float(step.get("hold_sec", 1.0)) * args.hold_scale
            if pose_name not in poses:
                rospy.logwarn("Pose '%s' not found, skipping", pose_name)
                continue
            angles = poses[pose_name]
            rospy.loginfo("Go to pose %s: %s (hold %.2fs)", pose_name, angles, hold)
            go_pose(set_angle, channels, angles, args.settle)
            t0 = time.time()
            while (time.time() - t0) < hold and not rospy.is_shutdown():
                time.sleep(0.02)

    rospy.loginfo("Sequence complete. Returning home.")
    home()

if __name__ == "__main__":
    main()