# -*- coding: utf-8 -*-
"""
Created on Mon Feb  6 12:16:30 2023

@author: Bijo Sebastian
"""
import robot_params
import numpy as np 

class Go_to_goal_controller:
    
    #Errors
    prev_heading_error = 0.0
    total_heading_error = 0.0
    
    #Controller parameters
    Kp = 0.00656
    Kd = 0.0001
    Ki = 0.0
    dt = 0.5

    def at_goal(self, robot_state, goal_state):    
    
        #check if we have reached goal point
        d = np.sqrt(((goal_state[0] - robot_state[0])**2) + ((goal_state[1] - robot_state[1])**2))
    
        if d <= robot_params.goal_threshold:
            return True
        else:
            return False

    def gtg(self, robot_state, goal_state):  
        #The Go to goal controller
        
        #determine how far to rotate to face the goal point
        #PS. ALL ANGLES ARE IN RADIANS
        delta_theta = (np.arctan2((goal_state[1] - robot_state[1]), (goal_state[0] - robot_state[0]))) - robot_state[2]
        #restrict angle to (-pi,pi)
        delta_theta = ((delta_theta + np.pi)%(2.0*np.pi)) - np.pi
        
        #Error is delta_theta in degrees
        e_new = ((delta_theta*180.0)/np.pi)
        e_dot = (e_new - self.prev_heading_error)/self.dt 
        self.total_heading_error = (self.total_heading_error + e_new)*self.dt
        #control input for angular velocity
        W = (self.Kp*e_new) + (self.Ki*self.total_heading_error) + (self.Kd*e_dot)
        self.prev_heading_error = e_new
      
        #find distance to goal
        d = np.sqrt(((goal_state[0] - robot_state[0])**2) + ((goal_state[1] - robot_state[1])**2))
        
        #velocity parameters
        distThresh = 0.1#mm
        
        #control input for linear velocity
        V = (0.12/1.5)*(np.arctan(d - distThresh))
        
        #request robot to execute velocity
        return[V,W]
                                           
                       
