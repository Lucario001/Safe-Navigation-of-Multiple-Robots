# -*- coding: utf-8 -*-
"""
Created on Thu Jul  4 16:34:17 2024

@author: kanni
"""

import time
import sim_interface
import numpy as np
import cvxopt
import robot_params

def main():
    
    N = 8                                                                       # No of robots in the environment
    list_robots = [None for _ in range(N)]
    sample_time = 0.01                                                          # Sample time
    
    l = 0.1                                                                     # Look ahead distance
    
    #curpos = cvxopt.matrix([0.0, 0.0, 0.0, 2.0, 0.0, 4.0, 0.0, 6.0, 4.0, 0.0, 4.0, 2.0, 4.0, 4.0, 4.0, 6.0])   Curpos of the robots [x1, y1, x2, y2, x3, y3, x4, y4, x5, y5, x6, y6, x7, y7, x8, y8]
    #goalpos = cvxopt.matrix([4.0, 6.0, 4.0, 2.0, 4.0, 4.0, 4.0, 0.0, 0.0, 2.0, 0.0, 6.0, 0.0, 0.0, 0.0, 4.0])  Goalpos of the robots [xg1, yg1, xg2, yg2, xg3, yg3, xg4, yg4, xg5, yg5, xg6, yg6, xg7, yg7, xg8, yg8]
    
    curpos = cvxopt.matrix([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    goalpos = cvxopt.matrix([4.5, 6.5, 4.5, 2.5, 4.5, 4.5, 4.5, 0.5, 0.5, 2.5, 0.5, 6.5, 0.5, 0.5, 0.5, 4.5])
    
    tolerance = np.sqrt(robot_params.goal_threshold ** 2 * N)                                                          # Tolerance
    d = 0.8
    alpha = 0.1
    

    if sim_interface.sim_init():
        for i in range(N):
            list_robots[i] = sim_interface.Pioneer(i + 1)
            list_robots[i].localize_robot()
            curpos[2 * i], curpos[2 * i + 1] = list_robots[i].current_state[0] + l * np.cos(list_robots[i].current_state[2]), list_robots[i].current_state[1] + l * np.sin(list_robots[i].current_state[2])
            list_robots[i].goal_state = [goalpos[2 * i], goalpos[2 * i + 1]]
    else:
        print ('Failed connecting to remote API server')
    
    P = cvxopt.matrix([[0.0 for _ in range(2 * N)] for _ in range(2 * N)])
    for i in range(2 * N):
        P[i * 2 * N + i] = 2.0
    
    while np.linalg.norm(curpos - goalpos) > tolerance:
        predicted_u_dot = cvxopt.matrix([0.0 for _ in range(2 * N)])
        for i in range(N):
            V, W = list_robots[i].get_cbf_controller()
            predicted_u_dot[2 * i], predicted_u_dot[2 * i + 1] = V * np.cos(list_robots[i].current_state[2]) - l * W * np.sin(list_robots[i].current_state[2]), V * np.sin(list_robots[i].current_state[2]) + l * W * np.cos(list_robots[i].current_state[2])
            
        g = [[0.0 for _ in range(N * (N - 1) // 2 + 2 * N)] for _ in range(2 * N)]
        h_t = [0.0 for _ in range(N * (N - 1) // 2 + 2 * N)]
        index = 0
        for i in range(N - 1):
            for j in range(i + 1, N):
                g[2 * i][index], g[2 * i + 1][index] = - 2 * (curpos[2 * i] - curpos[2 * j]), - 2 * (curpos[2 * i + 1] - curpos[2 * j + 1])
                g[2 * j][index], g[2 * j + 1][index] = - g[2 * i][index], - g[2 * i + 1][index]
                cbf = (curpos[2 * i] - curpos[2 * j]) ** 2 + (curpos[2 * i + 1] - curpos[2 * j + 1]) ** 2 - d ** 2
                h_t[index] = alpha * cbf
                index += 1
        for i in range(N):
            g[2 * i][index] = - 10 + 2 * curpos[2 * i]
            g[2 * i + 1][index + 1] = - 10 + 2 * curpos[2 * i + 1]
            h_t[index] = (curpos[2 * i] - 0.25) * (9.75 - curpos[2 * i])
            h_t[index + 1] = (curpos[2 * i + 1] - 0.25) * (9.75 - curpos[2 * i + 1])
            index += 2
        G = cvxopt.matrix(g)
        h = cvxopt.matrix(h_t)
        q = - 2 * predicted_u_dot
        u_dot = cvxopt.solvers.qp(P = P, q = q, G = G, h = h)['x']
        #print(np.array(u_dot).reshape(len(u_dot),))
        cvxopt.solvers.options['show_progress'] = False
        if (sim_interface.start_simulation()):
            for i in range(N):
                V = u_dot[2 * i] * np.cos(list_robots[i].current_state[2]) + u_dot[2 * i + 1] * np.sin(list_robots[i].current_state[2])
                W = - u_dot[2 * i] * np.sin(list_robots[i].current_state[2]) / l + u_dot[2 * i + 1] * np.cos(list_robots[i].current_state[2]) / l
                list_robots[i].run_cbf_controller(V, W)
                list_robots[i].localize_robot()
                curpos[2 * i], curpos[2 * i + 1] = list_robots[i].current_state[0] + l * np.cos(list_robots[i].current_state[2]), list_robots[i].current_state[1] + l * np.sin(list_robots[i].current_state[2])
            #print(np.array(curpos).reshape(len(curpos),))
            print(np.array(h).reshape(len(h),))
        else:
            print ('Failed to start simulation')
    time.sleep(5.0)
    sim_interface.sim_shutdown()
    time.sleep(2.0)
        
if __name__ == '__main__':
    main()
    print('Program ended')