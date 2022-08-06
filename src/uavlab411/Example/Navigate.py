import rospy
import math
from uavlab411 import srv
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
import time
rospy.init_node("navigate_node")

uavpose = 0
is_pose = False
wps = [[0, 1], [1, 1], [1, 2], [2, 2], [3, 4], [4, 4]]
x= [   0.49804688, 0.53125  ,  0.96289062, 0.99609375 ,1.36132812,
 1.42773438 ,1.7265625,  2.02539062 ,2.82226562, 2.85546875 ,2.88867188,
 3.41992188 ,3.61914062 ,3.88476562, 3.91796875, 3.95117188]
y= [ 1.4296875, 1.4296875 ,2.859375 , 2.859375,  4.2890625 ,4.2890625,
 4.2890625 ,5.71875,   7.1484375, 7.1484375, 7.1484375, 8.578125 , 8.578125,
 8.578125,  8.578125  ,8.578125 ]
get_telemetry = rospy.ServiceProxy('uavlab411/telemetry', srv.Telemetry)
navigate_to = rospy.ServiceProxy('uavlab411/navigate', srv.Navigate)
takeoff_srv = rospy.ServiceProxy('uavlab411/takeoff', srv.Takeoff)
land_srv = rospy.ServiceProxy('uavlab411/land', Trigger)

def navigate_wait(x, y, z, nav_mode, tolerance=0.15):
    res = navigate_to(x=x, y=y, z=z, nav_mode=nav_mode,tolerance = tolerance)
    if not res.success:
        return res

    time = rospy.get_rostime()
    while not rospy.is_shutdown():
        telemetry = get_telemetry("indoor")
        if(telemetry.mode != nav_mode):
            return res
        rospy.sleep(0.2)
        if (rospy.get_rostime() - time > rospy.Duration(20)):
            print("Can nav to wp!")
            return res


def takeoff(z):
    res = takeoff_srv(z=z,tolerance = 0.1)
    if not res.success:
        return res
    wait_for_telemetry()
    while not rospy.is_shutdown():
        telemetry = get_telemetry("indoor")
        if(telemetry.mode != 0):
            return res
        rospy.sleep(0.2)

def wait_for_telemetry():
    while not get_telemetry("indoor").isPose:
        rospy.sleep(0.5)

# takeoff
print("Take off now!")
takeoff(1.5)
rospy.sleep(2)
start = time.time()

# navigate
for i in range (len(x)):
    wait_for_telemetry()
    print("navigate to wp " + str(i))
    navigate_wait(x=x[i], y=y[i], z=1.5, nav_mode=3,
                  tolerance=0.2)
    # rospy.sleep(1)

land_srv()
print("Time to fly: %.2f"%(time.time()-start))