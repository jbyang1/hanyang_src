# -*- coding: utf-8 -*-
import rospy
import rospkg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from math import sqrt,pow



def findLocalPath(ref_path,status_msg): ## global_path와 차량의 status_msg를 이용해 현재waypoint와 local_path를 생성 ##
    out_path=Path()
    current_x=status_msg.position.x
    current_y=status_msg.position.y
    current_waypoint=0
    min_dis=float('inf')

    for i in range(len(ref_path.poses)) :
        dx=current_x - ref_path.poses[i].pose.position.x
        dy=current_y - ref_path.poses[i].pose.position.y
        dis=sqrt(pow(dx,2) + pow(dy,2))
        if dis < min_dis:
            min_dis=dis
            current_waypoint=i


    if current_waypoint+50 > len(ref_path.poses) :
        last_local_waypoint= len(ref_path.poses)
    else :
        last_local_waypoint=current_waypoint+50


    out_path.header.frame_id='map'
    for i in range(current_waypoint,last_local_waypoint) :
        tmp_pose=PoseStamped()
        tmp_pose.pose.position.x=ref_path.poses[i].pose.position.x
        tmp_pose.pose.position.y=ref_path.poses[i].pose.position.y
        tmp_pose.pose.position.z=ref_path.poses[i].pose.position.z
        tmp_pose.pose.orientation.x=0
        tmp_pose.pose.orientation.y=0
        tmp_pose.pose.orientation.z=0
        tmp_pose.pose.orientation.w=1
        out_path.poses.append(tmp_pose)

    return out_path,current_waypoint ## local_path와 waypoint를 반환 ##