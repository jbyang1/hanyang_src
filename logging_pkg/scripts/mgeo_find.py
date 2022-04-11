#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy

from math import pi,pow,sqrt
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Point32,PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *

class find_mgeo_data :

    def __init__(self,mgeo_map):
        
        self.map_name = mgeo_map

        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/'+self.map_name))
        mgeo_planner_map = MGeoPlannerMap.create_instance_from_json(load_path)

        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set
        self.nodes=node_set.nodes
        self.links=link_set.lines
        self.link_msg=self.getAllLinks()
        self.node_msg=self.getAllNode()
        self.temp_link = Link()
        self.temp_node = Node()

    def getAllLinks(self):
        all_link=PointCloud()
        all_link.header.frame_id='map'
        
        for link_idx in self.links :
            for link_point in self.links[link_idx].points:
                tmp_point=Point32()
                tmp_point.x=link_point[0]
                tmp_point.y=link_point[1]
                tmp_point.z=link_point[2]
                all_link.points.append(tmp_point)

        return all_link
    
    def getAllNode(self):
        all_node=PointCloud()
        all_node.header.frame_id='map'
        for node_idx in self.nodes :
            tmp_point=Point32()
            tmp_point.x=self.nodes[node_idx].point[0]
            tmp_point.y=self.nodes[node_idx].point[1]
            tmp_point.z=self.nodes[node_idx].point[2]
            all_node.points.append(tmp_point)

        return all_node


    def find_link_idx(self,pose_x,pose_y):
        
        prev_dis = -1
        idx = None
        while True:
            min_dis  = float('inf')
            for node_idx in self.nodes:                                   
                dx = self.nodes[node_idx].point[0] - pose_x
                dy = self.nodes[node_idx].point[1] - pose_y
                dis = sqrt(pow(dx,2)+pow(dy,2))

                if prev_dis < dis and dis < min_dis :
                    min_dis = dis
                    near_node = self.nodes[node_idx]
            
            link_list =  near_node.to_links + near_node.from_links
            for link in link_list :
                for link_point in link.points:
                    x = link_point[0]
                    y = link_point[1]
                
                    dx = pose_x - x
                    dy = pose_y - y

                    dist = sqrt(pow(dx,2)+pow(dy,2))         
                    if dist < 1.5 :
                        idx = link
                        return idx #link
                    if prev_dis >= 500 :
                        return idx #None
            
            prev_dis = min_dis

    def get_link(self,idx):
        return self.links[idx]
        
    
    def get_next_link(self,idx):
        try:
            if len(self.links[idx].to_node.to_links)== 0:
                return self.links[idx].to_node.to_links[0]
            else:
                min = float('inf')
                min_index = -1
                for index,link in enumerate(self.links[idx].to_node.to_links):
                    if link.ego_lane < min :
                        min = link.ego_lane
                        min_index = index
                return self.links[idx].to_node.to_links[min_index]
        except:
            pass
        

    
    def get_prev_link(self,idx):

        try:
            if len(self.links[idx].from_node.from_links) == 0:
                return self.links[idx].from_node.from_links[0]
            else:
                min = float('inf')
                min_index = -1
                for index,link in enumerate(self.links[idx].from_node.from_links):
                    if link.ego_lane < min :
                        min = link.ego_lane
                        min_index = index
                return self.links[idx].from_node.from_links[min_index]
        except:
            pass
