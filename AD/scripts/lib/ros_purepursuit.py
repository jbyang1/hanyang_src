# -*- coding: utf-8 -*-
import rospy
import rospkg
from geometry_msgs.msg import Point
from math import cos,sin,sqrt,pow,atan2,pi


class purePursuit : ## purePursuit 알고리즘 적용 ##
    def __init__(self):
        self.forward_point=Point()
        self.current_postion=Point()
        self.is_look_forward_point=False
        self.vehicle_length=2.8
        self.lfd=5
        self.min_lfd=5
        self.max_lfd=30
        self.steering=0
        
    def getPath(self,msg):
        self.path=msg  #nav_msgs/Path 
    
    def getEgoStatus(self,msg):

        self.current_vel=msg.velocity  #kph
        self.vehicle_yaw=(msg.heading)/180*pi   # rad
        self.current_postion.x=msg.position.x ## 차량의 현재x 좌표 ##
        self.current_postion.y=msg.position.y ## 차량의 현재y 좌표 ##
        self.current_postion.z=msg.position.z ## 차량의 현재z 좌표 ##

    def steering_angle(self): ## purePursuit 알고리즘을 이용한 Steering 계산 ## 
        vehicle_position=self.current_postion
        rotated_point=Point()
        self.is_look_forward_point= False

        for i in self.path.poses :
            path_point=i.pose.position
            dx= path_point.x - vehicle_position.x
            dy= path_point.y - vehicle_position.y
            rotated_point.x=cos(self.vehicle_yaw)*dx +sin(self.vehicle_yaw)*dy
            rotated_point.y=sin(self.vehicle_yaw)*dx - cos(self.vehicle_yaw)*dy
 
            if rotated_point.x>0 :
                dis=sqrt(pow(rotated_point.x,2)+pow(rotated_point.y,2))
                
                if dis>= self.lfd :
                    
                    self.lfd=self.current_vel.x * 0.65

                    if self.lfd < self.min_lfd : 
                        self.lfd=self.min_lfd
                    elif self.lfd > self.max_lfd :
                        self.lfd=self.max_lfd
                    self.forward_point=path_point
                    self.is_look_forward_point=True
                    
                    break
        
        theta=atan2(rotated_point.y,rotated_point.x)

        if self.is_look_forward_point :
            self.steering=atan2((2*self.vehicle_length*sin(theta)),self.lfd)# *180/pi deg
            return self.steering ## Steering 반환 ##
        else : 
            print("no found forward point")
            return 0
