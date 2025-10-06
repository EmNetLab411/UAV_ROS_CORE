#!/usr/bin/env python
import rospy
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped

def set_offboard():
    rospy.init_node('offboard_node')
    
    # Chờ kết nối
    while not rospy.is_shutdown() and not rospy.get_param('/mavros/state/connected', False):
        rospy.sleep(0.1)
    
    # Set position setpoint
    setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2
    
    # Publish setpoint vài lần trước khi chuyển mode
    for i in range(100):
        setpoint_pub.publish(pose)
        rospy.sleep(0.01)
    
    # Chuyển sang OFFBOARD mode
    rospy.wait_for_service('/mavros/set_mode')
    try:
        set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        response = set_mode(custom_mode="OFFBOARD")
        rospy.loginfo("Offboard mode set: %s", response.mode_sent)
    except rospy.ServiceException as e:
        rospy.logerr("Set mode failed: %s", e)

if __name__ == '__main__':
    set_offboard()
