#!/usr/bin/env python

"""
Multi robot simulation setup
@author: Bijo Sebastian 
"""

#Import libraries
import time

#Import files
import sim_interface

def main():
    if (sim_interface.sim_init()):

        #Create three robot and setup interface for all three 
        robot1 = sim_interface.Pioneer(1)
        robot2 = sim_interface.Pioneer(2)
        robot3 = sim_interface.Pioneer(3)

        #Start simulation
        if (sim_interface.start_simulation()):
            
            robot1.goal_state = [4 - 0.5 , 1 - 0.5]
            robot2.goal_state = [9 - 0.5 , 1 - 0.5]
            robot3.goal_state = [9 - 0.5 , 2 - 0.5]
                    
            while not robot1.robot_at_goal() or not robot2.robot_at_goal() or not robot3.robot_at_goal():
                #Run the control loops for three robots
                robot1.run_controller()
                robot2.run_controller()
                robot3.run_controller()
                
        else:
            print ('Failed to start simulation')
    else:
        print ('Failed connecting to remote API server')
    
    #shutdown
    sim_interface.sim_shutdown()
    time.sleep(2.0)
    return

#run
if __name__ == '__main__':

    main()                    
    print ('Program ended')
            

 