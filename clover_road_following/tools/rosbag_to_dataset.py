#!/usr/bin/env python3
import os
import csv
import argparse
import cv2
from cv_bridge import CvBridge
import rosbag
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, TwistStamped

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out_dir", required=True)
    ap.add_argument("--image-topic", default="/main_camera/image_raw")
    ap.add_argument("--twist-topics", nargs="+", default=["/mavros/setpoint_velocity/cmd_vel_unstamped", "/cmd_vel"])
    ap.add_argument("--cmd-timeout", type=float, default=0.5)
    args = ap.parse_args()

    os.makedirs(os.path.join(args.out_dir, "images"), exist_ok=True)
    csv_path = os.path.join(args.out_dir, "data.csv")
    fcsv = open(csv_path, "w", newline="")
    wr = csv.writer(fcsv)
    wr.writerow(["file_path", "yaw"])

    bridge = CvBridge()
    last_yaw = None
    last_cmd_time = None

    def handle_cmd(topic, msg, t):
        nonlocal last_yaw, last_cmd_time
        if topic.endswith("cmd_vel_unstamped") and isinstance(msg, TwistStamped):
            last_yaw = float(msg.twist.angular.z)
            last_cmd_time = msg.header.stamp.to_sec() if msg.header.stamp else t.to_sec()
        elif isinstance(msg, Twist):
            last_yaw = float(msg.angular.z)
            last_cmd_time = t.to_sec()

    with rosbag.Bag(args.bag, "r") as bag:
        for topic, msg, t in bag.read_messages():
            if topic in args.twist_topics:
                try:
                    handle_cmd(topic, msg, t)
                except Exception:
                    pass
            elif topic == args.image_topic and isinstance(msg, Image):
                if last_yaw is None or last_cmd_time is None:
                    continue
                if abs(t.to_sec() - last_cmd_time) > args.cmd_timeout:
                    continue
                try:
                    cv_bgr = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                    ts = int((msg.header.stamp.to_sec() if msg.header.stamp else t.to_sec()) * 1e6)
                    fname = f"{ts}.jpg"
                    rel = os.path.join("images", fname)
                    cv2.imwrite(os.path.join(args.out_dir, rel), cv_bgr)
                    wr.writerow([rel, f"{last_yaw:.6f}"])
                except Exception:
                    pass

    fcsv.close()
    print(f"Wrote dataset to {args.out_dir}")

if __name__ == "__main__":
    main()