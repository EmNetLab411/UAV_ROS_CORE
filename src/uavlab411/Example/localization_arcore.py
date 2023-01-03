import bluetooth
import math
import rospy
from threading import Thread
import threading
import time
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
camera_point=None
object_point=None

def thread_listening(client_sock):
    try:
        global camera_point
        global object_point
        uav_pose_seq=0
        uav_path = Path()
        try:
            while True:
                data = client_sock.recv(1024)
                msg = data.decode('UTF-8').split(",")
                #print(msg[0])
                if msg[0]=="1":
                    camera_point = np.array([float(msg[1]),float(msg[2]),float(msg[3])])
                elif msg[0]=="2":
                    object_point = np.array([float(msg[1]),float(msg[2]),float(msg[3])])
                print("cam", camera_point)
                print("obj",object_point)
                if (camera_point is not None):
                    uav_pose_seq +=1
                    uav_pose = PoseStamped()
                    uav_pose.header.seq = uav_pose_seq
                    uav_pose.header.stamp = rospy.Time.now()
                    uav_pose.header.frame_id = "map"
                    uav_pose.pose.position.x = -camera_point[2]
                    uav_pose.pose.position.y = -camera_point[0]
                    uav_pose.pose.position.z = camera_point[1]
                    uav_pose.pose.orientation.x = -float(msg[6])
                    uav_pose.pose.orientation.y = -float(msg[4])
                    uav_pose.pose.orientation.z = float(msg[5])
                    uav_pose.pose.orientation.w = float(msg[7])
                    pub_pose.publish(uav_pose)

                    uav_path.header = uav_pose.header
                    uav_path.poses.append(uav_pose)
                    pub_path.publish(uav_path)
                if(camera_point is not None and object_point is not None):
                    print("distance to object", np.sqrt(np.sum((camera_point-object_point)**2, axis=0)))
                    print("\n\n") 
                    
        except:
            print("receiving connect failed")
            client_sock.close()
            server_sock.close()
    except KeyboardInterrupt:
        print("stop")
        client_sock.close()
        server_sock.close()
def thread_sending(client_sock):
    try:
        while True:
            client_sock.send("400,1200")
            time.sleep(30)
    except:
        print("sending connect fail")
   
rospy.init_node("arcore")
pub_pose = rospy.Publisher('/uav_lab411/pose', PoseStamped, queue_size=10)
pub_path = rospy.Publisher('/uav_lab411/path', Path, queue_size=10)
server_sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )

port = 20
server_sock.bind(("",port))
server_sock.listen(1)

client_sock,address = server_sock.accept()
print( "Accepted connection from ",address)
t1 = threading.Thread(target=thread_listening, args=(client_sock,))
t2 = threading.Thread(target=thread_sending, args=(client_sock,))
t1.start()
t2.start()
t1.join()
t2.join()
client_sock.close()
server_sock.close()
print("main failed")