#!/usr/bin/env python3
import argparse
import time

import rospy
from std_srvs.srv import Trigger
from pca9685_servo_control.srv import SetAngle

def main():
    parser = argparse.ArgumentParser(description="Sweep demo for PCA9685 servos")
    parser.add_argument("--ns", default="/pca9685_servo", help="Namespace for services (default: /pca9685_servo)")
    parser.add_argument("--channels", type=int, nargs="*", default=[0,1,2,3,4], help="Channels to sweep")
    parser.add_argument("--min", type=float, default=30.0, help="Min angle deg")
    parser.add_argument("--max", type=float, default=150.0, help="Max angle deg")
    parser.add_argument("--step", type=float, default=10.0, help="Step deg")
    parser.add_argument("--dwell", type=float, default=0.2, help="Dwell time per step (s)")
    parser.add_argument("--rounds", type=int, default=2, help="Number of sweep rounds")
    args = parser.parse_args()

    rospy.init_node("pca9685_demo_sweep", anonymous=True)
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

    # Enable outputs and go home
    enable()
    home()

    for r in range(args.rounds):
        for ch in args.channels:
            rospy.loginfo("Sweeping channel %d (round %d)", ch, r+1)
            # Up
            a = args.min
            while a <= args.max + 1e-3 and not rospy.is_shutdown():
                set_angle(ch, a)
                time.sleep(args.dwell)
                a += args.step
            # Down
            a = args.max
            while a >= args.min - 1e-3 and not rospy.is_shutdown():
                set_angle(ch, a)
                time.sleep(args.dwell)
                a -= args.step

    rospy.loginfo("Sweep done. Returning home.")
    home()

if __name__ == "__main__":
    main()