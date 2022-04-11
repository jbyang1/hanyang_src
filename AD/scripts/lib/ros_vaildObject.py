# -*- coding: utf-8 -*-
import rospy
import rospkg

import numpy as np
from math import cos,sin

class object_struct :
    def __init__(self):
        self.num_of_objects=0
        self.pose_x=[]
        self.pose_y=[]
        self.velocity=[]
        self.object_type=[]

class object_info:
    def __init__(self):
        self.object_num = 0
        self.object_type = []
        self.object_pose_x = []
        self.object_pose_y = []
        self.object_velocity = []
        self.object_index = []

class vaildObject : ## 장애물 유무 확인 (차량, 사람, 정지선 신호) ##

    def __init__(self,stop_line=[]):
        self.stop_line = stop_line

    def get_object(self,object_info):
        self.all_object = object_struct()
        self.all_object.num_of_objects = object_info[0]
        self.all_object.object_type = object_info[1]
        self.all_object.pose_x = object_info[2]
        self.all_object.pose_y = object_info[3]
        self.all_object.velocity = object_info[4]
        
        

    def calc_vaild_obj(self,ego_pose):  # ego_pose[x, y, heading]
        global_object_info = object_info()
        global_object_info.object_num = self.all_object.num_of_objects
        local_object_info = object_info()
        
        

        tmp_theta = ego_pose[2]
        tmp_translation = [ego_pose[0],ego_pose[1]]
        tmp_t = np.array([[cos(tmp_theta), -sin(tmp_theta),tmp_translation[0]],
                         [sin(tmp_theta),cos(tmp_theta),tmp_translation[1]],
                         [0,0,1]])
        tmp_det_t = np.array([[tmp_t[0][0],tmp_t[1][0],-(tmp_t[0][0]*tmp_translation[0]+tmp_t[1][0]*tmp_translation[1])],
                             [tmp_t[0][1],tmp_t[1][1],-(tmp_t[0][1]*tmp_translation[0]+tmp_t[1][1]*tmp_translation[1])],
                             [0,0,1]])

        if self.all_object.num_of_objects > 0:
            for num in range(self.all_object.num_of_objects):
                global_result = np.array([[self.all_object.pose_x[num]],[self.all_object.pose_y[num]],[1]])
                local_result = tmp_det_t.dot(global_result)
                if local_result[0][0]> 0 :
                    global_object_info.object_type.append(self.all_object.object_type[num])
                    global_object_info.object_pose_x.append(self.all_object.pose_x[num])
                    global_object_info.object_pose_y.append(self.all_object.pose_y[num])
                    global_object_info.object_velocity.append(self.all_object.velocity[num])

                    local_object_info.object_type.append(self.all_object.object_type[num])
                    local_object_info.object_pose_x.append(local_result[0][0])
                    local_object_info.object_pose_y.append(local_result[1][0])
                    local_object_info.object_velocity.append(self.all_object.velocity[num])
        
        for line in self.stop_line:
            global_result=np.array([[line[0]],[line[1]],[1]])
            local_result=tmp_det_t.dot(global_result)
            if local_result[0][0]> 0 :
                global_object_info.object_type.append(3)
                global_object_info.object_pose_x.append(line[0])
                global_object_info.object_pose_y.append(line[1])
                global_object_info.object_velocity.append(0)

                local_object_info.object_type.append(3)
                local_object_info.object_pose_x.append(local_result[0][0])
                local_object_info.object_pose_y.append(local_result[1][0])
                local_object_info.object_velocity.append(0)
                global_object_info.object_num += 1

        return global_object_info,local_object_info