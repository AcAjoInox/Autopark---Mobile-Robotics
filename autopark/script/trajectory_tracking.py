#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from rospy.core import rospyinfo
from std_msgs import msg
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates
import yaml
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
import numpy as np
from trajectory_generation import Trajectory_generation
from Linear_control import Linear_control_law, nonLinear_control_law
import goalpos as g
import dist_obj as dist
import math

class Trajectory_tracking():
    #attributes
    t = []
    x_d = []
    y_d = []
    v_d = []
    w_d = []
    theta_d = []
    q=[]
    dotx_d=[]
    doty_d=[]

    appov=[]
    appow=[]
    appox = []
    appoy = []
    appoteta = []
    appopsi = []
    thetaprec=0
    A_park=[]
    
    def __init__(self):
        print("Starting node Trajectory control")
        rospy.init_node('trajectory_tracking', anonymous=True)
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) 
        rospy.Subscriber('/ground_truth/state',Odometry, self.odometryCb)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
        self.a=dist.laser()
        msg = rospy.wait_for_message("/scan", LaserScan, timeout=5)
        self.a.get_flag(msg)

    def callback(self, data):
        # robot pose from ModelStates
        indice = data.name.index('ferrari')
        data_pose_x = data.pose[indice].position.x
        data_pose_y = data.pose[indice].position.y
        self.data_pose= np.array([data_pose_x,data_pose_y])
        return self.data_pose
         
    def odometryCb(self,msg):
        #current robot pose
        x = round(msg.pose.pose.position.x,4)
        y = round(msg.pose.pose.position.y,4)
        theta = round(self.get_angle_pose(msg.pose.pose),4) 

        y=round(y-1.4*np.cos(theta+np.pi),4) ##spostiamo il punto di riferimento dal centro della macchina al centro dell' asse posteriore
        x=round(x+1.4*np.sin(theta+np.pi),4) ##spostiamo il punto di riferimento dal centro della macchina al centro dell' asse posteriore

        self.q = np.array([x, y, theta])

        return self.q

    def get_angle_pose(self, quaternion_pose): 
        #compute angle from quaternion
        #     
        q = [quaternion_pose.orientation.x,
                quaternion_pose.orientation.y,
                quaternion_pose.orientation.z,
                quaternion_pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        theta = yaw
        tol=0.1
        if abs(abs(theta)-abs(self.thetaprec))>2*np.pi-tol and self.thetaprec!=0:
            theta=theta+2*np.pi-tol
        else:
            pass
        self.thetaprec=theta
        return theta

    def trajectory_generation(self, traj,a):

        data = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=5)
        posizione = self.callback(data)
        x = round(posizione[0],1)
        y = round(posizione[1],1)
        tg = Trajectory_generation()
        q_i = self.get_pose()
        self.trajectory=traj
        
        if(self.trajectory == "parallel_parking" ):
                        (self.x_d, self.y_d,self.dotx_d,self.doty_d,self.v_d, self.w_d , self.theta_d , self.psi, self.A_park) =tg.parallel_parking_trajectory(q_i, self.t,a)
            #self.A_park indica il punto di partenza della manovra di parcheggio, ovvero punto in corrispondenza del parcheggio successivo a quello libero

    def get_laser(self):
        flag_free=self.a.park()[0] # flag che indica se ha trovato  un parcheggio libero
        flag_occ=self.a.park()[1] # flag che indica se il parcheggio si trova prima(2), in corrispondenza(1) o dopo(0)
        if flag_free==1:
            self.send_velocities(0,0,0)
            self.send_velocities(0,0,0)
            self.send_velocities(0,0,0)
            self.send_velocities(0,0,0)
        return flag_free,flag_occ

    def get_pose(self):
        #get robot position updated from callback
        x = self.q[0]
        y = self.q[1]
        theta = self.q[2]
        return np.array([x, y, theta])

    def get_error(self, T,traj):

        if(self.trajectory == "parallel_parking" ):
            (x, y, theta) = self.get_pose() #NB: i punti x e y sono sull'asse posteriore, non è il centro della macchina
        else:
            (a, b, theta) = self.get_pose() #prendo solo theta
            x=self.data_pose[0]
            y=self.data_pose[1]
        #compute error
        e1 = (self.x_d[T] - x) * np.cos(theta) + (self.y_d[T] - y ) * np.sin(theta)   
        e2 = -(self.x_d[T] - x) * np.sin(theta) + (self.y_d[T] - y ) * np.cos(theta)
        # theta (derivante dall'odometria) quando va oltre 3,14 si inverte di segno (vede il 3,14 come -3.14 e va verso 0 come negativo)
        e3 = self.theta_d[T] - theta   if len(self.theta_d) else 0
        
        if e3>np.pi :
            e3-=2*np.pi 
        elif e3<-np.pi:
            e3+=2*np.pi 
        else:
            pass
        
        print("x_d:{} and x_odom:{} sample:{}".format(self.x_d[T][0],x,T))
        print("y_d:{} and y_odom:{} sample:{}".format(self.y_d[T][0],y,T))
        print("theta_d:{} and theta_odom:{} sample:{}".format(self.theta_d[T],theta,T))

        return np.array([float(e1), float(e2), e3])
  
    def unicicle_Linear_control(self,traj,zeta,a):

        rospy.sleep(0.1)    # need small time to setup q in callback
        max_t = self.t[len(self.t) - 1]
        len_t = len(self.t)
        self.trajectory=traj

        if(self.trajectory == "parallel_parking" ):
            for i in np.arange(0, len_t):   
                now = rospy.get_time()        
                err = self.get_error(i,self.trajectory)
                if round(0.03*len_t)<=i<=round(0.87*len_t) : #tra il 3% e l' 87% uso il controllore
                    (v, w) = Linear_control_law(err, self.v_d[i], self.w_d[i],zeta,a)  
                else: # utilizziamo nella parte iniziale e finale le desiderate 
                    #(evitiamo gli spike del controllore dovuti a valori prossimi allo zero di v e w)
                    v=self.v_d[i]
                    w=self.w_d[i]
                # (v, w) = Linear_control_law(err, self.v_d[i], self.w_d[i],zeta,a) 


                    

                print("theta_d:{} and theta_odom:{} sample:{}".format(self.theta_d[i],  self.q[2] ,  i) )
                print("v_d:{} and v:{} sample:{}".format(self.v_d[i],  v ,  i) )
                print("w_d:{} and w:{} sample:{}".format(-self.w_d[i],  w ,  i) )
                print('Errors{}'.format(err))

                self.send_velocities(v, w)
                diff = rospy.get_time() - now
                rospy.sleep(max_t/len_t + 0.0058)
                self.appov.append(v)
                self.appow.append(w)
                self.appox.append(self.q[0])
                self.appoy.append(self.q[1])
                self.appoteta.append(self.q[2])
                # self.appopsi.append(math.atan(w*2.85/v))
            
        else:
            pass

        self.send_velocities(0,0,0)
        self.send_velocities(0,0,0)
        self.send_velocities(0,0,0)
        self.send_velocities(0,0,0)
        self.plot_wmr()
    
    def plot_wmr(self):

        plot1 = plt.figure(1)
        plt.title('path')
        plt.plot(self.x_d,self.y_d)
        plt.plot(self.appox ,self.appoy )
        plt.plot()
        plt.xlabel('x')
        plt.ylabel('y')
        plot2 = plt.figure(2)
        plt.title('velocità')
        plt.plot(self.t,self.v_d)
        plt.plot(self.t,self.appov)
        plt.xlabel('time')
        plt.ylabel('Velocità lineare')
        plot3 = plt.figure(3)
        plt.plot(self.t,self.w_d)
        plt.plot(self.t,self.appow)
        plt.xlabel('time')
        plt.ylabel('Velocità angolare ')
        plot4 = plt.figure(4)
        plt.plot(self.t,self.theta_d)
        plt.plot(self.t,self.appoteta)
        plt.xlabel('time')
        plt.ylabel('teta ')

        # plot5 = plt.figure(5)
        # plt.plot(self.t,self.psi)
        # plt.plot(self.t,self.appopsi)
        # plt.xlabel('time')
        # plt.ylabel('psi')

        plt.show()

    def send_velocities(self, v, w, theta=None):

        twist_msg = Twist() # Creating a new message to send to the robot      
        twist_msg.linear.x = v 
        twist_msg.angular.z = -w
        self.twist_pub.publish(twist_msg)

    def to_point(self):

        toltheta=0.2
        tol=0.05
        vel=2
        q_i = self.get_pose()
        
        if np.pi/2-toltheta<=q_i[2]<=toltheta+np.pi/2 or -np.pi/2-toltheta<=q_i[2]<=toltheta-np.pi/2:
            while q_i[0]<=self.A_park[0][0]-tol or q_i[0]>=self.A_park[0][0]+tol:
                q_i = self.get_pose()
                self.send_velocities(vel,0,0)
                print("x_odom:{} and x_A:{}".format(q_i[0],  round(self.A_park[0][0],4)))
        elif  -toltheta<=q_i[2]<=toltheta or np.pi-toltheta<=q_i[2]<=toltheta+np.pi:
            while q_i[1]<=self.A_park[1][0]-tol or q_i[1]>=self.A_park[1][0]+tol:
                q_i = self.get_pose()
                self.send_velocities(vel,0,0)
                print("y_odom:{} and y_A:{}".format(q_i[1],  round(self.A_park[1][0],4)))
        self.send_velocities(0,0,0)
        self.send_velocities(0,0,0)
        self.send_velocities(0,0,0)
        self.send_velocities(0,0,0)
        print("STOP")
        print("Asse posteriore: x:{} and y:{}. Punto A: x:{} and y:{}".format(self.q[0],  self.q[1],self.A_park[0][0],self.A_park[1][0]))
      

if __name__ == "__main__":

    yaml_package_name = rospy.get_param('~yaml_package_name', 'object_spawner')
    yaml_relative_path = rospy.get_param('~yaml_relative_path', '/config/parcheggi2.yaml')
    m = g.parse_yaml(yaml_package_name,yaml_relative_path)

    tt=Trajectory_tracking()    
    tt.t = np.linspace(0, 5, 1500)
    trajectory = "parallel_parking" 
    toltheta=0.1
    if np.pi-toltheta<=tt.q[2]<=np.pi+toltheta or -toltheta<=tt.q[2]<=toltheta:
        a=tt.data_pose[1] #estraggo y odometria
    else: 
        a=tt.data_pose[0] #estraggo x odometria

    # SEZIONE AVANZAMENTO
    while tt.get_laser()[0]==0 and abs(a)<=13: # 13 fine strada parcheggi
        if tt.get_laser()[0]==1:
            print("Park Found")
        else:
            print("Park not Found")
            tt.send_velocities(3,0,0)
        if np.pi-toltheta<=tt.q[2]<=np.pi+toltheta or -toltheta<=tt.q[2]<=toltheta:
            a=tt.data_pose[1] #estraggo y odometria
        else: 
            a=tt.data_pose[0] #estraggo x odometria

    tt.send_velocities(0,0,0)
    tt.send_velocities(0,0,0)
    tt.send_velocities(0,0,0)
    if tt.get_laser()[0]==1:
        park=g.findpark(tt.q,m,tt.get_laser()[1]) # coordinate centro del parcheggio libero (x,y,theta): 
        print("Park Coodinate={} ".format(park))
        tt.trajectory_generation(trajectory,park) # trajectory generation
        print("Park beginning point (A): x={} and y={}".format(tt.A_park[0][0],tt.A_park[1][0]))
            
    if len(tt.A_park)>0:
        tt.to_point()
        rospy.sleep(1)
        zeta= 0.9 #parametri per controllo lineare
        a= 1.45
        tt.t = np.linspace(0, 5, 1500)
        trajectory = "parallel_parking"  
        tt.trajectory_generation(trajectory,park)
        tt.unicicle_Linear_control(trajectory,zeta,a)
        
    else:
        print("No Free Spot")
        

    

