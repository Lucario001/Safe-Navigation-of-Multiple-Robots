# -*- coding: utf-8 -*-
"""
Created on Mon Feb  6 12:16:30 2023

@author: Bijo Sebastian
"""
import time

import robot_params
import control
import numpy as np


try:
  import sim
except:
  print ('--------------------------------------------------------------')
  print ('"sim.py" could not be imported. This means very probably that')
  print ('either "sim.py" or the remoteApi library could not be found.')
  print ('Make sure both are in the same folder as this file,')
  print ('or appropriately adjust the file "sim.py"')
  print ('--------------------------------------------------------------')
  print ('')

client_ID = []


def sim_init():
  global sim
  global client_ID
  
  #Initialize sim interface
  sim.simxFinish(-1) # just in case, close all opened connections
  client_ID = sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim    
  if client_ID!=-1:
    print ('Connected to remote API server')
    return True
  else:
    return False

def start_simulation():
  global sim
  global client_ID

  ###Start the Simulation: Keep printing out status messages!!!
  res = sim.simxStartSimulation(client_ID, sim.simx_opmode_oneshot_wait)

  if res == sim.simx_return_ok:
    print ("---!!! Started Simulation !!! ---")
    return True
  else:
    return False

def sim_shutdown():
  #Gracefully shutdown simulation

  global sim
  global client_ID

  #Stop simulation
  res = sim.simxStopSimulation(client_ID, sim.simx_opmode_oneshot_wait)
  if res == sim.simx_return_ok:
    print ("---!!! Stopped Simulation !!! ---")

  # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
  sim.simxGetPingTime(client_ID)

  # Now close the connection to CoppeliaSim:
  sim.simxFinish(client_ID)      

  return            

class Pioneer:
    
    #Handles
    pioneer_handle = None
    pioneer_left_motor_handle = None
    pioneer_right_motor_handle = None
    
    #state variables 
    goal_state = [0.0, 0.0] #x,y position
    current_state = [0.0, 0.0, 0.0] #x,y position and theta orientation
    
    robot_control = control.Go_to_goal_controller()
    
    # default constructor
    def __init__(self, id):
        """
        Sets up the pioneer
        """
        #Set up the name for the robot, must match coppeliasim 
        self.name = "/Pioneer"+str(id)
        self.get_handles() 
        self.setvel_pioneer(0.0, 0.0)
        return        

    def get_handles(self):
        #Get the handles to the sim items
        # Handle to Pioneer
        res , self.pioneer_handle = sim.simxGetObjectHandle(client_ID, self.name, sim.simx_opmode_blocking)
        res,  self.pioneer_left_motor_handle = sim.simxGetObjectHandle(client_ID, self.name + "/leftMotor", sim.simx_opmode_blocking)
        res,  self.pioneer_right_motor_handle = sim.simxGetObjectHandle(client_ID, self.name + "/rightMotor", sim.simx_opmode_blocking)
    
        # Get the position of the Pioneer1 for the first time in streaming mode
        res , pioneer_Position = sim.simxGetObjectPosition(client_ID, self.pioneer_handle, -1 , sim.simx_opmode_streaming)
        res , pioneer_Orientation = sim.simxGetObjectOrientation(client_ID, self.pioneer_handle, -1 , sim.simx_opmode_streaming)
      
        # Stop all joint actuations:Make sure Pioneer1 is stationary:
        res = sim.simxSetJointTargetVelocity(client_ID, self.pioneer_left_motor_handle, 0, sim.simx_opmode_streaming)
        res = sim.simxSetJointTargetVelocity(client_ID, self.pioneer_right_motor_handle, 0, sim.simx_opmode_streaming)
    
        print ("Succesfully obtained handles")
    
        return

    def localize_robot(self):
        #Function that will return the current location of Pioneer
        #PS. THE ORIENTATION WILL BE RETURNED IN RADIANS        
      
        res , pioneer_Position = sim.simxGetObjectPosition(client_ID, self.pioneer_handle, -1 , sim.simx_opmode_buffer)
        res , pioneer_Orientation = sim.simxGetObjectOrientation(client_ID, self.pioneer_handle, -1 , sim.simx_opmode_buffer)
      
        x = pioneer_Position[0]
        y = pioneer_Position[1]
        theta  =pioneer_Orientation[2]
        #print("robot", x,y,theta)
      
        self.current_state = [x, y, theta]
        return 
    
    def setvel_pioneer(self, V, W):
        #Function to set the linear and rotational velocity of pioneers
    
        # Limit v,w from controller to +/- of their max
        w = max(min(W, robot_params.pioneer_max_W), -1.0*robot_params.pioneer_max_W)
        v = max(min(V, robot_params.pioneer_max_V), -1.0*robot_params.pioneer_max_V)
              
        # Compute desired vel_r, vel_l needed to ensure w
        Vr = ((2.0*v) + (w*robot_params.pioneer_track_width))/(2*robot_params.pioneer_wheel_radius)
        Vl = ((2.0*v) - (w*robot_params.pioneer_track_width))/(2*robot_params.pioneer_wheel_radius)
                          
        # Set velocity
        sim.simxSetJointTargetVelocity(client_ID, self.pioneer_left_motor_handle, Vl, sim.simx_opmode_oneshot_wait)
        sim.simxSetJointTargetVelocity(client_ID, self.pioneer_right_motor_handle, Vr, sim.simx_opmode_oneshot_wait)
      
        return  
    
    def robot_at_goal(self):
        #Check if robot at goal
        #print (self.current_state)
        #print (self.goal_state)
        self.localize_robot()
        return self.robot_control.at_goal(self.current_state, self.goal_state)
    
    def run_controller(self):
        #Run the gtg_control lop for the robot

        #Localise the robot 
        self.localize_robot()
        
        #Check if robot is at goal
        if not self.robot_at_goal():
            #Run control loop 
            V, W = self.robot_control.gtg(self.current_state, self.goal_state)
            self.setvel_pioneer(V, W)
            time.sleep(0.5)
        else:
            #Stop robot
            print("Reached local goal")
            self.setvel_pioneer(0.0, 0.0)
    
    """
    Created on Fri July  5 16:04:00 2024
    
    @author: kanni
    """
    
    def get_cbf_controller(self):
        # Returns the control input
        
        #Localize the robot
        self.localize_robot()
        
        #Check if robot is at goal
        if not self.robot_at_goal():
            V, W = self.robot_control.gtg(self.current_state, self.goal_state)
            return V, W
        else:
            return 0.0, 0.0
    
    def run_cbf_controller(self, V, W):
        if not self.robot_at_goal():
            self.setvel_pioneer(V, W)
            time.sleep(0.01)
        else:
            print("Reached local goal")
            self.setvel_pioneer(0.0, 0.0)

    
   
       
      

