#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy,rospkg
import sys,os
import tf
import numpy as np
from math import pi

#msg
from nav_msgs.msg import Path,Odometry
from morai_msgs.msg import EgoVehicleStatus,ObjectStatusList,CtrlCmd,GetTrafficLightStatus,SetTrafficLight

#lib
from lib.ros_cruisecontrol import cruiseControl
from lib.ros_findLocalPath import findLocalPath
from lib.ros_pathReader import pathReader
from lib.ros_pid import pidController
from lib.ros_purepursuit import purePursuit
from lib.ros_vaildObject import vaildObject
from lib.ros_velocityPlanning import velocityPlanning

class gen_planner():

    def __init__(self):
        rospy.init_node('gen_planner', anonymous=True)

        print(">>> SELECT PATH\t <<<")
        print(">>> 1.BLACK PATH <<<")
        print(">>> 2.RED PATH\t <<<")
        path_num = input(">>")

        if path_num == 1 :
            self.path_name = "brt0_path_1"
        elif path_num == 2 :
            self.path_name = "brt0_path_2"
        else:
            os.exit(0)

        #publisher
        global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        local_path_pub  = rospy.Publisher('/local_path', Path, queue_size=1)
        ctrl_pub        = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        
        #subscriber
        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.statusCB)
        rospy.Subscriber('/Object_topic', ObjectStatusList, self.objectInfoCB)
        rospy.Subscriber('/GetTrafficLightStatus', GetTrafficLightStatus, self.getTL_callback)
        
        
        #msg
        ctrl_msg = CtrlCmd()
        
        #def
        self.is_status=False ## 차량 상태 점검
        self.is_obj=False ## 장애물 상태 점검
        self.is_traffic=False ## 신호등 상태 점검
        self.traffic_info = [[-1036.56,5279.18,'C119AS300257'], ## TrafficLight information
                             [-1044.03,4897.28,'C119AS300194'],
                             [-1040.09,4862.13,'C119AS300198'],
                             [-999.49,4648.46,'C119AS300202'],
                             [-839.46,4243.78,'C119AS300183'],
                             [-844.14,3813.18,'C119AS300174'],
                             [-939.66,3576.03,'C119AS300160'],
                             [-991.98,3446.17,'C119AS300166'],
                             [-1132.50,3135.50,'C119AS300170'],
                             [-1312.90,2678.27,'C119AS300123'],
                             [-1325.45,2322.07,'C119AS300102'],
                             [-1306.24,2134.36,'C119AS300085'],
                             [-1241.62,1887.16,'C119AS300089'],
                             [-1229.95,1855.55,'C119AS300093'],
                             [-1124.79,1638.27,'C119AS300096'],
                             [-1099.27,1623.03,'C119AS300063'],
                             [-1209.09,1844.76,'C119AS300067'],
                             [-1221.31,1875.73,'C119AS300071'],
                             [-1286.82,2110.35,'C119AS300074'],
                             [-1306.87,2260.22,'C119AS300105'],
                             [-1306.14,2621.09,'C119AS300112'],
                             [-1162.55,3049.64,'C119AS300132'],
                             [-999.02,3382.53,'C119AS300136'],
                             [-944.47,3523.23,'C119AS300145'],
                             [-853.48,3749.72,'C119AS300177'],
                             [-810.67,4189.88,'C119AS300184'],
                             [-979.53,4634.29,'C119AS300187'],
                             [-1021.87,4844.29,'C119AS300192'],
                             [-1026.85,4879.04,'C119AS300211'],
                             [-1026.47,5229.07,'C119AS300271'],
                             [248.61,34.97,'C119AS300538'],
                             [560.70,247.00,'C119AS300748'],
                             [849.73,471.17,'C119AS300516'],
                             [1510.05,820.36,'C119AS300686'],
                             [1541.53,836.39,'C119AS300683'],
                             [1982.85,1057.68,'C119AS300678'],
                             [2305.22,1234.19,'C119AS300660'],
                             [2373.75,1285.93,'C119AS300657'],
                             [2534.85,1448.06,'C119AS300644'],
                             [2676.03,1652.58,'C119AS300639'],
                             [2747.41,1782.53,'C119AS300623'],
                             [2865.78,2094.86,'C119AS300617'],
                             [2950.01,2238.12,'C119AS300604'],
                             [2970.72,2274.62,'C119AS300596'],
                             [3192.11,2664.82,'C119AS300738'],
                             [3603.84,2911.73,'C119AS300702'],
                             [3656.85,2937.24,'C119AS300587'],
                             [3211.91,2729.85,'C119AS300595'],
                             [2962.44,2298.87,'C119AS300601'],
                             [2941.74,2262.80,'C119AS300605'],
                             [2877.72,2148.38,'C119AS300620'],
                             [2736.30,1810.70,'C119AS300627'],
                             [2693.69,1721.44,'C119AS300636'],
                             [2532.97,1471.05,'C119AS300647'],
                             [2375.66,1311.95,'C119AS300654'],
                             [2308.71,1258.47,'C119AS300665'],
                             [2014.12,1090.70,'C119AS300670'],
                             [1550.71,857.99,'C119AS300689'],
                             [1521.85,843.76,'C119AS300691'],
                             [886.59,517.39,'C119AS300508'],
                             [587.37,286.01,'C119AS300744'],
                             [296.81,76.12,'C119AS300531']]

        
        #class        
        path_reader = pathReader(pkg_name = 'AD')
        self.global_path = path_reader.read_txt(self.path_name) ## 출력할 경로의 이름(확장자 제외)       
        
        pure_pursuit = purePursuit() ## purePursuit import

        pid = pidController(p = 0.1, i = 0, d = 0.05)

        self.cc = cruiseControl(object_vel_gain = 0.5, object_dis_gain = 1) ## cruiseControl import (object_vel_gain, object_dis_gain)

        self.vo = vaildObject(self.traffic_info) ## 장애물 유무 확인 (TrafficLight)

        vel_planner = velocityPlanning(car_max_speed = 60/3.6, road_friction = 0.15) ## 속도 계획(최고속도 / 도로마찰계수)

        vel_profile = vel_planner.curveBasedVelocity(self.global_path, point_num = 100)



        #time var
        count = 0
        rate = rospy.Rate(30) # 30hz

        print("Start!!!")

        while not rospy.is_shutdown():
            
            

            if self.is_status and self.is_obj :
                
                ## global_path와 차량의 status_msg를 이용해 현제 waypoint와 local_path를 생성
                local_path,self.current_waypoint = findLocalPath(self.global_path,self.status_msg)

                self.vo.get_object(self.object_info)

                global_obj,local_obj = self.vo.calc_vaild_obj([self.status_msg.position.x,self.status_msg.position.y,(self.status_msg.heading)/180*pi])
                
                
                if self.is_traffic:                
                    self.cc.checkObject(local_path,global_obj,local_obj,[self.tl_msg.trafficLightIndex,self.tl_msg.trafficLightStatus])                     
                else :
                    self.cc.checkObject(local_path,global_obj,local_obj)

                # pure pursuit control
                pure_pursuit.getPath(local_path) ## pure_pursuit 알고리즘에 Local path 적용
                pure_pursuit.getEgoStatus(self.status_msg) ## pure_pursuit 알고리즘에 차량의 status 적용                
                ctrl_msg.steering=-pure_pursuit.steering_angle()
        

                cc_vel = self.cc.acc(local_obj,self.status_msg.velocity.x,vel_profile[self.current_waypoint],self.status_msg.position) ## advanced cruise control 적용한 속도 계획
                target_velocity = cc_vel
  

                control_input=pid.pid(target_velocity,self.status_msg.velocity) ## 속도 제어를 위한 PID 적용 (target Velocity, Status Velocity)

                if control_input > 0 :
                    ctrl_msg.accel= control_input
                    ctrl_msg.brake= 0
                else :
                    ctrl_msg.accel= 0
                    ctrl_msg.brake= -control_input


                local_path_pub.publish(local_path) ## Local Path publish
                ctrl_pub.publish(ctrl_msg) ## Vehicl Control publish
                os.system('clear')
                print("accel = ",ctrl_msg.accel)
                print("brake = ",ctrl_msg.brake)
                print("steer = ",ctrl_msg.steering)
                if self.is_traffic:  
                    print("tl",[self.tl_msg.trafficLightIndex,self.tl_msg.trafficLightStatus])

            
                if count==30 : ## global path 출력
                    global_path_pub.publish(self.global_path)
                    count=0
                count+=1
                rate.sleep()

    def statusCB(self,data): ## Vehicle Status Subscriber 
        self.status_msg=data        
        self.is_status=True

    def objectInfoCB(self,data): ## Object information Subscriber
        object_type=[]
        object_pose_x=[]
        object_pose_y=[]
        object_velocity=[]

        for num in range(data.num_of_npcs) :
            object_type.append(data.npc_list[num].type)
            object_pose_x.append(data.npc_list[num].position.x)
            object_pose_y.append(data.npc_list[num].position.y)
            object_velocity.append(data.npc_list[num].velocity.x)

        for num in range(data.num_of_obstacle) :
            object_type.append(data.obstacle_list[num].type)
            object_pose_x.append(data.obstacle_list[num].position.x)
            object_pose_y.append(data.obstacle_list[num].position.y)
            object_velocity.append(data.obstacle_list[num].velocity.x)

        for num in range(data.num_of_pedestrian) :
            object_type.append(data.pedestrian_list[num].type)
            object_pose_x.append(data.pedestrian_list[num].position.x)
            object_pose_y.append(data.pedestrian_list[num].position.y)
            object_velocity.append(data.pedestrian_list[num].velocity.x)

        self.object_num = data.num_of_npcs+data.num_of_obstacle+data.num_of_pedestrian
        self.object_info=[self.object_num,object_type,object_pose_x,object_pose_y,object_velocity]
        self.is_obj = True

    def getTL_callback(self,msg): ## TrafficLight information Subscriber
        self.is_traffic=True
        self.tl_msg=GetTrafficLightStatus()
        self.tl_msg=msg
    
if __name__ == '__main__':
    try:
        run=gen_planner()
    except rospy.ROSInterruptException:
        pass

