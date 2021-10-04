import numpy as np
import rospy
'''
This function implements a simple constant gain controller for $k_1(v_d,\omega_d)$
''' 
# zeta = 0.9  #0.1
# a = 1.45     #0.8

def k1_circ(v_d, w_d,zeta, a):

    return 2*zeta*a
    #return 1.2
   
'''
This function implements a simple constant gain controller for $k_3(v_d,\omega_d)$
''' 
def k3_circ(v_d, w_d,zeta, a):

    return 2*zeta*a
    #return 1.2

def k1(v_d, w_d,zeta, a):

    return 2*zeta*np.sqrt(v_d**2 + a*w_d**2)

def k3(v_d, w_d,zeta, a):

    return 2*zeta*np.sqrt(v_d**2 + a*w_d**2)

def nonLinear_control_law(e, v_d, w_d,zeta, a):
    k2 = (a**2 -w_d**2)/v_d
    u_1 = -k1(v_d, w_d,zeta,a) * e[0]
    # Be sure that if e[2] = 0 sin(e[2])/e[2] is computes to 1.0
    if e[2] == 0:
        u_2 = -k2*v_d*e[1] - k3(v_d,w_d,zeta,a)*e[2]
    else:
        u_2 = -k2*v_d*np.sin(e[2])/e[2]*e[1] - k3(v_d,w_d,zeta,a)*e[2]
    #compute control input
    v = v_d * np.cos(e[2]) - u_1
    w = w_d - u_2
    
    return np.array([v, w])

'''
This function implements the control. k1 and k3 functions
are used to (possibly) implement time varying gains, whereas
the gain k2 is set in the function.
'''
def Linear_control_law(e, v_d, w_d ,zeta,a):

    k2 = (a**2 -w_d**2)/v_d
    u_1 = -k1_circ(v_d, w_d,zeta,a) * e[0]
    u_2 = -k2*e[1] - k3_circ(v_d,w_d,zeta,a)*e[2]
    #compute control input
    v = v_d *np.cos(e[2])- u_1
    w = w_d - u_2
    
    return np.array([v, w])