#!/usr/bin/env python

import os
import rospy
import math
from geometry_msgs.msg import PoseStamped
from datetime import datetime


dirname, filename = os.path.split(os.path.abspath(__file__))
print(dirname)

data_e_hora_atuais = datetime.now()
data_e_hora_em_texto = data_e_hora_atuais.strftime("%Y.%m.%d_%H.%M.%S")

name_file = dirname + '/position_' + data_e_hora_em_texto + '.csv'

with open(name_file, 'w') as pose_data:
     pass



def callback(data):
    # print("callback")
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    # print(rospy.get_caller_id() + "I heard %s", data)
    
    x = data.pose.position.x
    y = data.pose.position.y
    z = data.pose.position.z
    qx = data.pose.orientation.x
    qy = data.pose.orientation.y
    qz = data.pose.orientation.z
    qw = data.pose.orientation.w

    #now = rospy.get_rostime()
    seconds = rospy.get_time()
    #rospy.loginfo("Current time %i %i", now.secs, now.nsecs)

    #calculating roll, pitch and yaw
    yaw = math.atan2(2*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz)
    pitch = math.asin(-2.0*(qx*qz - qw*qy))
    roll = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)

    # print("yaw = ", yaw)
    # print("pitch = ", pitch)
    # print("roll = ", roll)

    data_e_hora_atuais = datetime.now()
    data_e_hora_em_texto = data_e_hora_atuais.strftime("%Y/%m/%d %H:%M:%S")

    with open(name_file, 'a') as pose_data:
        pose_data.write(data_e_hora_em_texto+", {:7.4f}, {:7.4f}, {:7.4f}, {:7.4f}, {:7.4f}, {:7.4f}\n".format( x, y , z,yaw, pitch, roll))
        # pose_data.write("{}, {}, {}\n".format(yaw, pitch, roll))

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    
    # run simultaneously.
    rospy.init_node('get_information', anonymous=True)
    rospy.Subscriber("/lactec/geo/pose/aruco_marker_frame_to_map", PoseStamped, callback)
    # spin() simply keeps python from exiting until this node is stopped
    # print("estou aqui")
    rospy.spin()
    
if __name__ == '__main__':
    listener()