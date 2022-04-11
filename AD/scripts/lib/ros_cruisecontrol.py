# -*- coding: utf-8 -*-
from threading import local
import rospy
import rospkg
from math import sqrt,pow

class cruiseControl: ## ACC(advanced cruise control) 적용 ##
    def __init__(self,object_vel_gain,object_dis_gain):
        self.object = [False,0]
        self.traffic = [False,0]
        self.Person = [False,0]
        self.object_vel_gain=object_vel_gain
        self.object_dis_gain=object_dis_gain


    def checkObject(self,local_path,global_obj,local_obj,tl=[]): ## 경로상의 장애물 유무 확인 (차량, 사람, 정지선 신호) ##
        self.object = [False,0]
        self.traffic = [False,0]
        self.Person = [False,0]
        if not len(tl) == 0 :
            self.tl_index = tl[0]
            self.tl_status = tl[1]
            

        
        min_rel_distance=float('inf')
        try:
            for i in range(global_obj.object_num):
                for path in local_path.poses :      
                    

                        dx = path.pose.position.x - global_obj.object_pose_x[i]
                        dy = path.pose.position.y - global_obj.object_pose_y[i]

                        dis = sqrt(pow(dx, 2) + pow(dy, 2))

                        dx = local_obj.object_pose_x[i]
                        dy = local_obj.object_pose_y[i]
                        rel_distance = sqrt(pow(dx, 2) + pow(dy, 2))

                        if global_obj.object_type[i] == 1 or global_obj.object_type[i] == 2:

                            if dis < 2.5 and rel_distance < min_rel_distance:                            
                                min_rel_distance = rel_distance
                                self.object = [True,i]
                            

                        if global_obj.object_type[i] == 0:

                            if dis<4.35 and rel_distance < min_rel_distance:
                                
                                min_rel_distance = rel_distance
                                self.Person = [True,i]


                        if global_obj.object_type[i] == 3 and not len(tl) == 0 :
                            traffic_sign='STOP'

                            if self.tl_status == 48 or self.tl_status == 16 :
                                traffic_sign ='GO'
                            else:                                 

                                if rel_distance < min_rel_distance:                                    
                                    min_rel_distance = rel_distance
                                    self.traffic = [True,i]
        except :
            pass
                    



    def acc(self,local_obj,ego_vel,target_vel,status_msg): ## advanced cruise control 를 이용한 속도 계획 ##
        out_vel=target_vel
        pre_out_vel = out_vel
        if self.object[0] == True :
            print("ACC ON_vehicle")   
            front_vehicle=[local_obj.object_pose_x[self.object[1]],local_obj.object_pose_y[self.object[1]],local_obj.object_velocity[self.object[1]]]
            time_gap=0.8
            default_space=8
            dis_safe=ego_vel* time_gap+default_space
            dis_rel=sqrt(pow(front_vehicle[0],2)+pow(front_vehicle[1],2))-3
            
            vel_rel=(front_vehicle[2]-ego_vel)  
            
            v_gain=self.object_vel_gain
            x_errgain=self.object_dis_gain
            acceleration=vel_rel*v_gain - x_errgain*(dis_safe-dis_rel)

            acc_based_vel=ego_vel+acceleration
            
            if acc_based_vel > target_vel : 
                acc_based_vel=target_vel
            
            if dis_safe-dis_rel >0 :
                out_vel=acc_based_vel
            else :
                if acc_based_vel<target_vel :
                    out_vel=acc_based_vel


        if self.Person[0]==True:
            print("ACC ON_person")
            Pedestrian=[local_obj.object_pose_x[self.Person[1]],local_obj.object_pose_y[self.Person[1]],local_obj.object_velocity[self.Person[1]]]
            time_gap=0.8
            default_space=8
            dis_safe=ego_vel* time_gap+default_space
            dis_rel=sqrt(pow(Pedestrian[0],2)+pow(Pedestrian[1],2))-3
            
            vel_rel=(Pedestrian[2]-ego_vel)  
            
            v_gain=self.object_vel_gain
            x_errgain=self.object_dis_gain
            acceleration=vel_rel*v_gain - x_errgain*(dis_safe-dis_rel)    

            acc_based_vel=ego_vel+acceleration
            
            if acc_based_vel > target_vel : 
                acc_based_vel=target_vel
            
            if dis_safe-dis_rel >0 :
                out_vel=acc_based_vel - 5
            else :
                if acc_based_vel<target_vel :
                    out_vel=acc_based_vel

        if self.traffic[0] == True :
            front_vehicle=[local_obj.object_pose_x[self.traffic[1]],local_obj.object_pose_y[self.traffic[1]],-5]
            time_gap=0.8
            default_space=2
            dis_safe=ego_vel* time_gap+default_space
            dis_rel=sqrt(pow(front_vehicle[0],2)+pow(front_vehicle[1],2))-3
            
            vel_rel=(0-ego_vel)  
            
            v_gain=self.object_vel_gain
            x_errgain=self.object_dis_gain
            acceleration=vel_rel*v_gain - x_errgain*(dis_safe-dis_rel)    

            acc_based_vel=ego_vel+acceleration
            
            if acc_based_vel > target_vel : 
                acc_based_vel=target_vel
            
            if dis_safe-dis_rel >0 :
                out_vel=acc_based_vel
            else :
                if acc_based_vel<target_vel :
                    out_vel=acc_based_vel

            if dis_rel < 3 :
                out_vel = -5

        return out_vel