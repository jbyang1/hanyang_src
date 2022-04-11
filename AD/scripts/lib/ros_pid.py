# -*- coding: utf-8 -*-
import rospy
import rospkg

class pidController : ## 속도 제어를 위한 PID 적용 ##
    def __init__(self, p, i, d):
        self.p_gain = p #0.1
        self.i_gain = i #0.0
        self.d_gain = d #0.05
        self.controlTime = 0.033
        self.prev_error = 0
        self.i_control = 0


    def pid(self,target_vel,current_vel):
        error = target_vel - current_vel.x
        
        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain *(error - self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error
        return output
