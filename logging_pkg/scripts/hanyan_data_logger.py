#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
import time
import pyproj
from datetime import datetime

from morai_msgs.msg import EgoVehicleStatus,GPSMessage,ObjectStatusList
from sensor_msgs.msg import Imu
from mgeo_find import find_mgeo_data

from math import sqrt,pow


class make_result :

    def __init__(self):
        rospy.init_node('make_result', anonymous=True)

        rospy.Subscriber("/gps",GPSMessage, self.gpsCB)
        rospy.Subscriber("/Ego_topic",EgoVehicleStatus,self.egoCB)
        rospy.Subscriber("/Object_topic",ObjectStatusList, self.objectCB)
        rospy.Subscriber("/imu",Imu, self.imu_callback)

        rospack=rospkg.RosPack()
        pkg_path=rospack.get_path('logging_pkg')

        self.file_name = 'EGO_DATA'
        full_path=pkg_path+'/log/'+self.file_name+'.txt'

        self.f=open(full_path,'w')
        data = "TIME\tLatitude\tLongitude\tvelocity_x\tvelocity_y\tvelocity_z\tacceleration\tangular_speed\twheel\tLane ID\tbrake\taccel\tfront_distance\tbehind_distance\tfront_ttc\tbehind_ttc\n"
        self.f.write(data) 

        self.file_name = 'NPC_DATA'
        full_path=pkg_path+'/log/'+self.file_name+'.txt'

        self.f2=open(full_path,'w')        
        data = "TIME\tLatitude\tLongitude\tvelocity_x\tvelocity_y\tvelocity_z\tacceleration\tLane ID\tfront_distance\tbehind_distance\tfront_ttc\tbehind_ttc\n"        
        self.f2.write(data) 

        self.is_gps=False
        self.is_ego=False
        self.is_obj=False
        self.is_imu=False

        self.utmOffset_x =  345145.7657761079
        self.utmOffset_y =  4037545.494624729
        self.utmOffset_z =  53.57714711671258
         
        
        rate=rospy.Rate(1)
        self.mgeo = find_mgeo_data('R_KR_PR_SejongBRT0')
        
        while not rospy.is_shutdown():
            
            if self.is_obj and self.is_gps and self.is_ego and self.is_imu:                       
                self.save_data()
                # print(self.mgeo.find_link_idx(self.ego_data.position.x,self.ego_data.position.y))
            else:
                if not self.is_ego:
                    print("ego_topic check ..")
                if not self.is_obj:
                    print("object_topic check ..")
                if not self.is_gps:
                    print("gps sensor connetion check..")
                if not self.is_imu:
                    print("imu sensor connetion check ..")

            rate.sleep()

        self.f.close()
        self.f2.close()
        
    def save_data(self):   
        
        time=(datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f'))
        print(time)  
        lat = self.gps_data.latitude
        lon = self.gps_data.longitude
        vel_x = self.ego_data.velocity.x
        vel_y = self.ego_data.velocity.y
        vel_z = self.ego_data.velocity.z
        acc = self.ego_data.acceleration.x
        ang = self.imu_msg.angular_velocity.z
        wheel = self.ego_data.wheel_angle
        Lane_id = self.get_lane_id(self.ego_data.position.x,self.ego_data.position.y)
        brake = self.ego_data.brake
        accel = self.ego_data.accel
        idx =  self.get_lane_idx(self.ego_data.position.x,self.ego_data.position.y)
        
        f_vehicle,f_dist= self.find_front_vehicle(idx,self.ego_data.position.x,self.ego_data.position.y)
        b_vehicle,b_dist = self.find_behind_vehicle(idx,self.ego_data.position.x,self.ego_data.position.y)
        
        f_ttc,b_ttc = self.get_ttc(f_vehicle,f_dist,b_vehicle,b_dist,self.ego_data.velocity.x)
         
        
        ego_data = '{0}\t{1}\t{2}\t{3}\t{4}\t{5}\t{6}\t{7}\t{8}\t{9}\t{10}\t{11}\t{12}\t{13}\t{14}\t{15}\n'.\
            format(time,lat,lon,vel_x,vel_y,vel_z,acc,ang,wheel,Lane_id,brake,accel,f_dist,b_dist,f_ttc,b_ttc)
        
        print(ego_data)
        self.f.write(ego_data)    


        ###   ================NPC====================  ###

        if self.obj_data.num_of_npcs == 0 :
            npc_data = 'None\n'
            self.f2.write(npc_data)
        else:
            
            for npc_list in self.obj_data.npc_list:
                x = self.utmOffset_x + npc_list.position.x
                y = self.utmOffset_y + npc_list.position.y
                lat,lon = self.xy_to_lonlat(x,y)          
                vel_x = npc_list.velocity.x/3.6
                vel_y = npc_list.velocity.y/3.6
                vel_z = npc_list.velocity.z/3.6
                acc = npc_list.acceleration.x
                Lane_id = self.get_lane_id(npc_list.position.x,npc_list.position.y)
                idx =  self.get_lane_idx(npc_list.position.x,npc_list.position.y)

                f_vehicle,f_dist = self.find_front_vehicle(idx,npc_list.position.x,npc_list.position.y)
                b_vehicle,b_dist = self.find_behind_vehicle(idx,npc_list.position.x,npc_list.position.y)
                
                f_ttc,b_ttc = self.get_ttc(f_vehicle,f_dist,b_vehicle,b_dist,npc_list.velocity.x/3.6 )


                npc_data = '{0}\t{1}\t{2}\t{3}\t{4}\t{5}\t{6}\t{7}\t{8}\t{9}\t{10}\t{11}\n'.\
                    format(time,lat,lon,vel_x,vel_y,vel_z,acc,Lane_id,f_dist,b_dist,f_ttc,b_ttc)
                self.f2.write(npc_data)

            self.f2.write("----------------------\n")

    def get_ttc(self,f_v,f_d,b_v,b_d,velocity):
        if not f_v  == None:
            f_v = f_v.velocity.x/3.6

            vel = velocity - f_v
            try:
                f_ttc = f_d / vel
            except ZeroDivisionError:
                f_ttc = f_d / 0.0001
        else:
            f_ttc = None

        if not b_v == None:
            b_v = b_v.velocity.x/3.6

            vel = b_v - velocity
            try:
                b_ttc = b_d / vel
            except ZeroDivisionError:
                b_ttc = b_d / 0.0001
        else:
            b_ttc = None

        return f_ttc, b_ttc

    def find_front_vehicle(self,current_idx,x,y):
        distance = 0
        link = self.mgeo.get_link(current_idx)        
        waypoint = self.find_waypoint(current_idx,x,y)    
        
        front_vehicle = None

        while True:
            for point in link.points[waypoint:]:
                distance = distance + 0.5
                for npc in self.obj_data.npc_list:
                    dx = point[0] - npc.position.x
                    dy = point[1] - npc.position.y

                    dist = sqrt(pow(dx,2)+pow(dy,2))

                    if dist < 0.5: #find front vehicle
                        front_vehicle = npc
                        return front_vehicle,distance
            
            waypoint = 0 
            link = self.mgeo.get_next_link(link.idx)
            if link == None:
                return front_vehicle,None
        

    def find_behind_vehicle(self,current_idx,x,y):
        distance = 0
        link = self.mgeo.get_link(current_idx)        
        
        waypoint = self.find_waypoint(current_idx,x,y)
        behind_vehicle = None

        while True:
            for point in link.points[waypoint::-1]:
                distance = distance + 0.5
                for npc in self.obj_data.npc_list:
                    dx = point[0] - npc.position.x
                    dy = point[1] - npc.position.y

                    dist = sqrt(pow(dx,2)+pow(dy,2))
                    if dist < 0.5: #find behind vehicle
                        behind_vehicle = npc
                        return behind_vehicle, distance

            waypoint = -1 
            link = self.mgeo.get_prev_link(link.idx)
            if link == None:
                return behind_vehicle,None


    def find_waypoint(self,current_idx,x,y):
        link = self.mgeo.get_link(current_idx)
        min_dis = float('inf')
        waypoint_num = -1
        for index,point in enumerate(link.points):
            dx = x - point[0]
            dy = y - point[1]

            dist = sqrt(pow(dx,2)+pow(dy,2))
            if dist < min_dis :
                min_dis = dist
                waypoint_num = index

        return waypoint_num



    def get_lane_id(self,x,y):
        try:
            return self.mgeo.find_link_idx(x,y).ego_lane
        except :
            return None

    def get_lane_idx(self,x,y):
        try:
            return self.mgeo.find_link_idx(x,y).idx
        except:
            return None

    def xy_to_lonlat(self,x, y):
        proj_latlon = pyproj.Proj(proj='latlong',datum='WGS84')
        proj_xy = pyproj.Proj(proj="utm", zone='52s', datum='WGS84')
        lonlat = pyproj.transform(proj_xy, proj_latlon, x, y)
        return lonlat[1],lonlat[0]

                        
    def gpsCB(self,msg):
        self.is_gps = True
        self.gps_data = msg

    def egoCB(self,msg):
        self.is_ego = True
        self.ego_data = msg

    def objectCB(self,msg):
        self.is_obj = True
        self.obj_data = msg

    def imu_callback(self,msg):
        self.is_imu = True
        self.imu_msg = msg


if __name__ == '__main__':
    make_result()