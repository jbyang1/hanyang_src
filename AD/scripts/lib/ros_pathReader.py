# -*- coding: utf-8 -*-
import rospy
import rospkg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class pathReader :  ## 텍스트 파일에서 경로를 출력 ##
    def __init__(self,pkg_name):
        rospack=rospkg.RosPack()
        self.file_path=rospack.get_path(pkg_name)

    def read_txt(self,file_name):
        full_file_name=self.file_path+"/path/"+file_name+".txt"
        openFile = open(full_file_name, 'r')
        out_path=Path()
        
        out_path.header.frame_id='/map'
        line=openFile.readlines()
        for i in line :
            tmp=i.split()
            read_pose=PoseStamped()
            read_pose.pose.position.x=float(tmp[0])
            read_pose.pose.position.y=float(tmp[1])
            read_pose.pose.position.z=float(tmp[2])
            read_pose.pose.orientation.x=0
            read_pose.pose.orientation.y=0
            read_pose.pose.orientation.z=0
            read_pose.pose.orientation.w=1
            out_path.poses.append(read_pose)
        
        
        openFile.close()
        return out_path ## 읽어온 경로를 global_path로 반환 ##
      



