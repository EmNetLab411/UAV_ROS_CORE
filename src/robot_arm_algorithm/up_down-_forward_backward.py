from __future__ import division
import numpy as np
import math
from geometry_msgs.msg import PoseStamped 
import string
import time
import rospy

import Adafruit_PCA9685

pwm = Adafruit_PCA9685.PCA9685()


def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    print(pulse)
    pwm.set_pwm(channel, 0, pulse)


def set_servo_angle(step1, step2, step3, step4, step5):
    Pstep1 = 540 - 440 / 180 * step1
    Pstep2 = 100 + 450 / 180 * step2
    Pstep3 = 130 + 470 / 180 * step3
    Pstep4 = 150 + 450 / 180 * step4
    Pstep5 = 400 - step5*200
    pwm.set_pwm(0, 0, int(Pstep1))
    pwm.set_pwm(1, 0, int(Pstep2))
    pwm.set_pwm(2, 0, int(Pstep3))
    pwm.set_pwm(3, 0, int(Pstep4))
    pwm.set_pwm(4, 0, int(Pstep5))

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

def callback(msg_range):
    global p1,p1_base_uav
    p_object_in_uav_coordinate=((msg_range.pose.position.y - p_object[0])*1000,(msg_range.pose.position.z - p_object[1])*1000)
    p_object_in_robot_coordinate = (p_object_in_uav_coordinate[0] - p1_base_uav[0], p_object_in_uav_coordinate[1] - p1_base_uav[1])
    find_angle((p_object_in_robot_coordinate[0],p_object_in_robot_coordinate[1]))
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/uavlab411/uavpose', PoseStamped, callback)


L1 = 120
L2 = 120
L3 = 120
p1_base_uav = (150,40) #position of robot in uav coordinate
p1 = (0, 0)
p2 = (0, 0)
p3 = (0, 0)
a1 = 90
a2 = 160
a3 = 120
p_object=(0.65,0.1) #y=0.8, z=0.1


def find_angle(des):
    try: 
        global a1, a2, a3, p1, p2, p3, L1, L2, L3
        d = math.sqrt((p1[0]- des[0])**2 + (p1[1]-des[1])**2)
        a1_min = a1_max = 0
        # dieu kien de goc a3 tu
        cos_a1_min = (L1**2 + d**2 - L2**2 - L3**2)/(2*L1*d)
        # dieu kien de co tam giac p2 p3 des
        cos_a1_max = (L1**2 + d**2 - (L2+L3)**2) / (2*L1*d)
        if cos_a1_min < 1:
            a1_min = math.acos(cos_a1_min)
        if cos_a1_max < 1:
            a1_max = math.acos(cos_a1_max)
        a1_min = (a1_min + math.atan2(des[1]-p1[1], des[0]-p1[0]))
        a1_max = (a1_max + math.atan2(des[1]-p1[1], des[0]-p1[0]))
        while True:
            a1 = (a1_max + a1_min)/2
            # a1 = a1_max
            p2 = (p1[0]+int(L1 * math.cos(a1)),
                p1[1]+int(L1 * math.sin(a1)))
            y = des[0] - p2[0]
            x = des[1] - p2[1]
            a3 = math.acos((x**2 + y**2 - L2**2 - L3**2)/(2*L2*L3))
            a2 = math.atan2(y, x) - math.atan2(L3*math.sin(a3), L2+L3*math.cos(a3))
            p3 = (p2[0] + int(L2*math.sin(a2)), p2[1] + int(L2*math.cos(a2)))
            if (p3[0]- p1[0])**2 + (p3[1]-p1[1])**2 < L1**2 + L2**2:
                a1_max -= 0.1
                continue
            des1 = (p3[0] + int(L3*math.sin(a2+a3)),
                    p3[1] + int(L3*math.cos(a2+a3)))
            A1 = 180 - a1/math.pi * 180
            A2 = 180 - A1 + a2/math.pi * 180
            A3 = 90 + a3/math.pi*180
            set_servo_angle(int(A1),int(A2),90,int(A3),1)
            print(A1, A2, A3)
            break
    except:
        print("can't control robot because object is in blind spot")

if __name__ == '__main__':
    listener()
    rospy.spin()
    #x = input()
    #set_servo_angle(170,170,90,150,1)
