#!/usr/bin/env python3
import numpy as np

def io_linearization_control_law(y1, y2, theta, y1d, y2d, doty1d, doty2d, b,thetad):
    # Define the two control gains. Notice we can define "how fast" we track on y_1 and y_2 _independently_
    k_1 = 1
    k_2 = 1
    
    #return virtual input doty1, doty2
    u_1 = doty1d + k_1*(y1d - y1)
    u_2 = doty2d + k_2*(y2d - y2)

    v = np.cos(theta) * u_1 + np.sin(theta) * u_2
    w = u_2/b * np.cos(theta) - u_1/b *np.sin(theta) #dottheta

    # if (abs(thetad - theta) < 0.06):
    #         v = np.sqrt( doty1d**2 + doty2d**2 )   
    #         w = 0
    # #return control input v, w
    # else: 
    #     w_arc=-1.65  #velocità angolare costante per percorrere l'arco (vedi generazione traiettoria circolare con raggio=4)
    #     v_arc=6.6   #velocità lineare costante per percorrere l'arco (vedi generazione traiettoria circolare)
    #     # v = np.cos(theta) * u_1 + np.sin(theta) * u_2
    #     # w = u_2/b * np.cos(theta) - u_1/b *np.sin(theta) #dottheta
    #     v=v_arc 
    #     w=w_arc
    return np.array([v, w])



