import rospy
from uavlab411.msg import data_sensor_msg
import random
rospy.init_node("sensor_node")

pub = rospy.Publisher("/uavlab411/sensor_data",data_sensor_msg,queue_size=1)
r = rospy.Rate(10) # 10hz
data = data_sensor_msg()
data.id = 1
data.lat = 20.922833
data.lon = 105.771217
data.hum = 52.36
data.temp = 42.36
data.dust = 89.23
while not rospy.is_shutdown():
   pub.publish(data)
   r.sleep()