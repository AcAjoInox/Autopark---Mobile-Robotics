#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from rospy.core import rospyinfo
from std_msgs import msg
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
import yaml
import matplotlib.pyplot as plt
import math
from sensor_msgs.msg import LaserScan


import numpy as np
from scipy.integrate import odeint
from trajectory_generation import Trajectory_generation
from io_linearization import io_linearization_control_law
from Linear_control import Linear_control_law, nonLinear_control_law
from posture_regulation import cartesian_regulation_control_law
import goalpos as g
import dist_obj as dist





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
    

    #methods
    def __init__(self):
        
        rospy.loginfo("Starting node Trajectory control")
        rospy.init_node('trajectory_tracking', anonymous=True) #make node
        #elf.twist_pub = rospy.Publisher('/r2d2_diff_drive_controller/cmd_vel', Twist, queue_size=10)
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) 
        
        rospy.Subscriber('/ground_truth/state',Odometry, self.odometryCb)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
        #rospy.Subscriber('move_base_simple/goal', PoseStamped, self.on_goal)
        self.a=dist.laser()
        msg = rospy.wait_for_message("/scan", LaserScan, timeout=5)
        # rospy.loginfo(" dopo dichiarazione")
        self.a.get_flag(msg)
        # rospy.loginfo(" dopo forzatura costruttore")

        
        
    # robot pose ModelStates
    def callback(self, data):
        # inserire manualmente il nome del modello di interesse
        indice = data.name.index('ferrari')
        data_pose_x = data.pose[indice].position.x
        data_pose_y = data.pose[indice].position.y
        self.data_pose= np.array([data_pose_x,data_pose_y])
        return self.data_pose
        
    #current robot pose
    def odometryCb(self,msg):
        
        x = round(msg.pose.pose.position.x,4)
        y = round(msg.pose.pose.position.y,4)
        theta = round(self.get_angle_pose(msg.pose.pose),4) 

        y=round(y-1.4*math.cos(theta+np.pi),4) ##spostiamo il punto di riferimento dal centro della macchina al centro dell' asse posteriore
        x=round(x+1.4*math.sin(theta+np.pi),4) ##spostiamo il punto di riferimento dal centro della macchina al centro dell' asse posteriore

        self.q = np.array([x, y, theta])

        return self.q

    def get_angle_pose(self, quaternion_pose):     #compute angle from quaternion
        q = [quaternion_pose.orientation.x,
                quaternion_pose.orientation.y,
                quaternion_pose.orientation.z,
                quaternion_pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        theta = yaw
        tol=0.1
        if abs(abs(theta)-abs(self.thetaprec))>2*math.pi-tol and self.thetaprec!=0:
            theta=theta+2*math.pi-tol
            # rospy.loginfo('STO CORREGGENDO THETA ')
        else:
            pass
        # rospy.loginfo('thetaPRECE  '  + str(self.thetaprec))
        # rospy.loginfo('thetaNOW   '+ str(theta))
        self.thetaprec=theta
        return theta

    def plot_figure(self, x, y , xp , yp , t , teta):
        plot1 = plt.figure(1)
        plt.plot(x,y)
        plt.title('path')
        plt.xlabel('x_d')
        plt.ylabel('y_d')
        plt.axis('equal')
        # plot2 = plt.figure(2)
        # plt.plot(t,xp)
        # plt.xlabel('time')
        # plt.ylabel('xp')
        # plot3 = plt.figure(3)
        # plt.plot(t,yp)
        # plt.xlabel('time')
        # plt.ylabel('yp')
        plot3 = plt.figure(4)
        plt.plot(t,teta)
        plt.xlabel('time')
        plt.ylabel('theta')
        
        plt.show()

    def trajectory_generation(self, traj,a):

        data = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=5)
        posizione = self.callback(data)
        x = round(posizione[0],1)
        y = round(posizione[1],1)
        tg = Trajectory_generation()
        q_i = self.get_pose()
        self.trajectory=traj
        # rospy.loginfo("flagpark :"+str(flagpark))
        rospy.loginfo("trajectory ="+str(self.trajectory))
        # exit()
        # rospy.loginfo("flagpark ="+str(flagpark))
        if(self.trajectory == "parallel_parking" ):#and flagpark==1):

            rospy.loginfo('estratto= '+str(a))
            # exit()
             #Initial trajectory posture (x_i,y_i,theta_i)
            # q_f = np.array([-21.000088, -4.9, 0])    #Final trajectory posture   (x_f,y_f,theta_f)
            (self.x_d, self.y_d,self.dotx_d,self.doty_d,self.v_d, self.w_d , self.theta_d , self.psi, self.A_park) =tg.parallel_parking_trajectory(q_i, self.t,a)
            #self.A_park indica il punto di partenza della manovra di parcheggio, ovvero punto in corrispondenza del parcheggio successivo a quello libero

        elif(trajectory == "square"):
            rospy.loginfo("Posizione iniziale ferrari x :" + str(x) + "y :" + str(y))
        
            q_0 = np.array([x, y])
            (self.x_d, self.y_d , self.dotx_d, self.doty_d ,self.t,self.theta_d,self.v_d,self.w_d) = tg.detecting_square_trajectory(q_0)
            rospy.loginfo("durata giro: " + str(self.t[-1]) + " s")
            rospy.loginfo("lunghezza vett tempi :"+str(len(self.t)))
            # rospy.loginfo("vettore x_d :"+str(self.x_d))
            # rospy.loginfo("vettore y_d :"+str(self.y_d))
            # rospy.loginfo("traiettoria quadrato caricata")
            self.plot_figure(self.x_d , self.y_d , self.dotx_d, self.doty_d ,self.t , self.theta_d)
            exit()

        elif(trajectory == "cyrcular"):
            centro = np.array([20.28, 20.28])
            iniz = np.array([20.28, 24.28])
            fin= np.array([24.28, 20.28])
            (self.x_d, self.y_d ,self.v_d, self.w_d , self.theta_d, self.dotx_d, self.doty_d) = tg.cyrcular_trajectory(self.t,centro[0],centro[1],iniz,fin,q_i[2])
            self.plot_figure(self.x_d , self.y_d , self.dotx_d, self.doty_d ,self.t , self.theta_d)
            exit()
          
        elif(self.trajectory == "cubic"):# and flagpark==1):
            
            
            initial_final_vel=3
            rospy.loginfo("trajectory ="+str(trajectory))
            self.A_park[2]=q_i[2]
            rospy.loginfo('A =' +str(self.A_park))
            rospy.loginfo('x_odom=' +str(q_i[0]))
            rospy.loginfo('y_odom=' +str(q_i[1]))
            rospy.loginfo('theta_odom=' +str(q_i[2]))
            
            (self.x_d, self.y_d,self.v_d, self.w_d , self.theta_d) =tg.cubic_trajectory(q_i,self.A_park, initial_final_vel, self.t)
            # exit()

        elif(self.trajectory == "retta"):# and flagpark==1):
               
            rospy.loginfo("trajectory ="+str(trajectory))
            self.A_park[2]=q_i[2]
            rospy.loginfo('A=' +str(self.A_park))
            rospy.loginfo('x_odom=' +str(q_i[0]))
            rospy.loginfo('y_odom=' +str(q_i[1]))
            rospy.loginfo('theta_odom=' +str(q_i[2]))
            
            (self.x_d, self.y_d,self.v_d, self.w_d , self.theta_d) =tg.retta(q_i,self.A_park, self.t)
            
        # exit()

    def get_laser(self):
        # rospy.loginfo(" dentro get_laser")
        # msg = rospy.wait_for_message("/scan", LaserScan, timeout=5)
        # a=dist.laser()
        # rospy.sleep(0.1)
        flag_free=self.a.park()[0] # flag che indica se ha trovato  un parcheggio libero
        flag_occ=self.a.park()[1] # flag che indica se il parcheggio si trova prima(2), in corrispondenza(1) o dopo(0)
        # rospy.loginfo('p=' +str(p))
        # rospy.loginfo('msg=' +str(msg))
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
        #slide 80 LDC
        if(self.trajectory == "parallel_parking" ):
            (x, y, theta) = self.get_pose() #NB: i punti x e y sono sull'asse posteriore, non è il centro della macchina
        else:
            (a, b, theta) = self.get_pose() #prendo solo theta
            x=self.data_pose[0]
            y=self.data_pose[1]
        #compute error
        e1 = (self.x_d[T] - x) * np.cos(theta) + (self.y_d[T] - y ) * np.sin(theta)   
        e2 = -(self.x_d[T] - x) * np.sin(theta) + (self.y_d[T] - y ) * np.cos(theta)
        #################################à theta (derivante dall'odometria) quando va oltre 3,14 si inverte di segno (vede il 3,14 come -3.14 e va verso 0 come negativo)
        #######inserire un controllo che se theta e theta_d sono opposti in segno, calcola l'errore come 6,28- errore calcolato
        e3 = self.theta_d[T] - theta   if len(self.theta_d) else 0
        
        if e3>np.pi :
            e3-=2*np.pi 
        elif e3<-np.pi:
            e3+=2*np.pi 
        else:
            pass
        #e3 = 0
        rospy.loginfo("teta d:" + str(self.theta_d[T]) + " teta_odom : " + str(theta))
        rospy.loginfo("x d:" + str(self.x_d[T]) + " x_odom : " + str(x))
        rospy.loginfo("y d:" + str(self.y_d[T]) + " y_odom : " + str(y))
        #e1 = (self.x_d[T] - x) 
        #e2 = (self.y_d[T] - y)
        #e3 = self.theta_d[T] - theta if len(self.theta_d) else 0
        return np.array([float(e1), float(e2), e3])
  
    def get_point_coordinate(self, b):
        #get robot position updated from callback
        # x = self.q[0]
        # y = self.q[1]
        x = self.data_pose[0]
        y = self.data_pose[1]
        theta = self.q[2]
        
        #robot point cooordinate to consider
        y1 = x + b * np.cos(theta)
        y2 = y + b * np.sin(theta)
        return [y1, y2, round(theta,2)]

    def unicycle_linearized_control(self): # inserire laser  
        ###########################################################################################################à
        # fp=tt.get_laser() #flag che indica se il parcheggio in direzione è libero (da mettere nella io lineariz)
        ##############################################################################################################à
        # Distance of point B from the point of contact P
        b = 3.5#0.75
        toltheta=0.05
        rospy.sleep(0.5)
        vel=30*10/36
        rac=20.28 #indica il punto limite (positivo o negativo) in cui si ferma il tratto lineare e inizia quello circolare
        # rospy.loginfo("la dimensione del vettore tempo è: "+str(len(self.t)))
        
        max_t = self.t[-1]
        len_t = len(self.t)
        # rospy.loginfo("theta_d= "+str(self.theta_d))
        # print("max_t: {}  ,  len_t: {}  ,  max_t/len_t: {}".format(max_t, len_t,max_t/len_t))
        # exit()

        for i in range(0,len(self.theta_d)): #aggiusto il theta desiderato per manternerlo nell'intervallo 0-2*pi
                if self.theta_d[i]>=2*np.pi:
                    appo=(self.theta_d[i])//(2*np.pi)
                    self.theta_d[i]=self.theta_d[i]-appo*2*np.pi

        for i in np.arange(0, len(self.theta_d)):
            (y1, y2, theta) = self.get_point_coordinate(b)
            # (x_odom, y_odom, theta) = self.get_pose() #odometria ###LA POSIZIONE CHE RESTITUISCE È RIFERITA AL CENTRO DELL'ASSE POSTERIORE, QUINDI NEI PRINT UTILIZZO self.data_pose
            if theta<0: # correggo theta perchè va da -3.14 a 3.14. Riporto theta nel range 0-2*pi per calcolare una differenza corretta nella condizione
                theta=2*np.pi-abs(theta)
            # (v, w) = io_linearization_control_law(y1, y2, theta, self.x_d[i], self.y_d[i], self.dotx_d[i], self.doty_d[i], b,self.theta_d[i])
            (v, w) = io_linearization_control_law(y1, y2, theta, self.x_d[i], self.y_d[i], self.dotx_d[i], self.doty_d[i], b,self.theta_d[i])
            print("x_odom: {}  and x_desiderata: {}".format(self.data_pose[0], self.x_d[i]))
            print("y_odom: {}  and y_desiderata: {}".format(self.data_pose[1], self.y_d[i]))
            print("theta_odom: {}  and theta_desiderata: {}".format(theta, self.theta_d[i]))
            # print("dotx_d: {}  and doty_d: {}".format(self.dotx_d[i], self.doty_d[i]))
            # print("v: {}  and w: {}".format(v, w))
            print("v_d: {}  and w_d: {}".format(self.v_d[i], self.w_d[i]))

           
            # if -rac<self.data_pose[0]<rac or -rac<self.data_pose[1]<rac:
            #     self.send_velocities(v, w, theta) #mando il controllore nel tratto lineare
            # else:
            #     self.send_velocities(0, 0, theta) #mando le desiderate nel tratto circolare
            self.send_velocities(v, w, theta)
            # self.send_velocities(self.v_d[i], -self.w_d[i], theta)
            rospy.sleep(max_t/len_t+0.022)#+0.019)
            
        #stop after times
        self.send_velocities(0,0,0)
        self.plot_figure(self.x_d , self.y_d , self.dotx_d, self.doty_d ,self.t , self.theta_d)
       
        
    def unicicle_Linear_control(self,traj,zeta,a):
        rac=20.28
        rospy.sleep(0.1)    #need small time to setup q in callback
        max_t = self.t[len(self.t) - 1]
        len_t = len(self.t)
        self.trajectory=traj
        # rospy.loginfo("maxt/lent: " + str(max_t/len_t))
        #exit()
        # rospy.loginfo("x: " + str(self.data_pose[0]))
        # rospy.loginfo("y: " + str(self.data_pose[1]))
        # exit()

        #comput control inputs variable from error
        # for k in range(0,len(self.theta_d)): #aggiusto il theta desiderato per manternerlo nell'intervallo 0-2*pi
        #     if self.theta_d[k]>=2*np.pi:
        #         appo=(self.theta_d[k])//(2*np.pi)
        #         self.theta_d[k]=self.theta_d[k]-appo*2*np.pi
        if(self.trajectory == "parallel_parking" ):
            for i in np.arange(0, len_t):   
                now = rospy.get_time()        
                err = self.get_error(i,self.trajectory)
                    #condizione x iniziare a far funzionare il controllore e non dividere per 0

                if round(0.03*len_t)<=i<=round(0.87*len_t) : 
                    (v, w) = Linear_control_law(err, self.v_d[i], self.w_d[i],zeta,a)   #passare a e zeta come parametri perchè variano tra la cubica e il parallel parking
                else: #tra il % e il % uso le desiderate
                    v=self.v_d[i]
                    w=self.w_d[i]

                #VA BENEEEEEEEEEEEEEEEEEEEEEE
                # if round(0.05*len_t)<=i<=round(0.4*len_t) : 
                #     (v, w) = Linear_control_law(err, self.v_d[i], self.w_d[i],zeta,a)   #passare a e zeta come parametri perchè variano tra la cubica e il parallel parking
                # elif round(0.55*len_t)<i<round(1*len_t):
                #     (v, w) = Linear_control_law(err, self.v_d[i], self.w_d[i],zeta,a)
                # else: #tra il % e il % uso le desiderate
                #     v=self.v_d[i]
                #     w=self.w_d[i]


                print("theta_d:{} and theta_odom:{} sample:{}".format(self.theta_d[i],  self.q[2] ,  i) )
                print("v_d:{} and v:{} sample:{}".format(self.v_d[i],  v ,  i) )
                print("w_d:{} and w:{} sample:{}".format(-self.w_d[i],  w ,  i) )
                print('Errors{}'.format(err))
                self.send_velocities(v, w)
                # self.send_velocities(self.v_d[i], -self.w_d[i])
                diff = rospy.get_time() - now
                rospy.sleep(max_t/len_t + + 0.0058)
                #rospy.loginfo("durata ciclo for: " + str(max_t/len_t + 0.0058))
                self.appov.append(v)
                self.appow.append(w)
                self.appox.append(self.q[0])
                self.appoy.append(self.q[1])
                self.appoteta.append(self.q[2])
                self.appopsi.append(math.atan(w*2.85/v))
            
        elif(self.trajectory == "square"):   
            # fp=tt.get_laser()
            tol=0.005
            i=0
            while i<len(self.x_d):#and fp==0:   #for i in np.arange(0, len_t):    #for per eseguire senza check parcheggi
                now = rospy.get_time()        
                err = self.get_error(i,self.trajectory)
                # utilizzo il controllore solo nei tratti dritti (uso come discriminante il theta con una piccola tolleranza):
                v=self.v_d[i]
                w=-self.w_d[i]
                rospy.loginfo("sto dando le desiderate")
                
                if abs(self.theta_d[i]-np.pi/2)<=tol or abs(self.theta_d[i])<=tol or abs(self.theta_d[i]-3*np.pi/2)<=tol or abs(self.theta_d[i]-np.pi)<=tol or abs(self.theta_d[i]-2*np.pi)<=tol:
                    (v, w) = nonLinear_control_law(err, self.v_d[i], -self.w_d[i],zeta,a)
                    w=0
                    rospy.loginfo("sto andando dritto")
                    rospy.sleep(0.0025)
                else: #raccordo circolare sui vertici della square (già fatto nella traiettoria desiderata)
                    v=self.v_d[i]
                    w=-self.w_d[i]
                    rospy.loginfo("sto girando")
                    rospy.sleep(0.0025)

                # if abs(self.theta_d[i]-np.pi/2)<=0.05 or abs(self.theta_d[i])<=0.05 or abs(self.theta_d[i]-3*np.pi/2)<=0.05 or abs(self.theta_d[i]-np.pi)<=0.05 or abs(self.theta_d[i]-2*np.pi)<=0.05:
                #     (v, w) = nonLinear_control_law(err, self.v_d[i], -self.w_d[i],zeta,a)
                #     rospy.loginfo("sto usando il controllore")
                # else: #raccordo circolare sui vertici della square (già fatto nella traiettoria desiderata)
                #     v=self.v_d[i]
                #     w=-self.w_d[i]
                #     rospy.loginfo("sto dando le desiderate")
                
                # fp=tt.get_laser()
                print("theta_d:{} and theta_odom:{} sample:{}".format(self.theta_d[i],  self.q[2] ,  i) )
                print("v_d:{} and v:{} sample:{}".format(self.v_d[i],  v ,  i) )
                print("w_d:{} and w:{} sample:{}".format(-self.w_d[i],  w ,  i) )
                print('Errors{}'.format(err))
                i+=1
                self.send_velocities(v, w)
                # self.send_velocities(self.v_d[i], -self.w_d[i])
                diff = rospy.get_time() - now
                rospy.sleep(max_t/len_t + 0.02)
                #rospy.loginfo("durata ciclo for: " + str(max_t/len_t + 0.0058))
                self.appov.append(v)
                self.appow.append(w)
                self.appox.append(self.data_pose[0])
                self.appoy.append(self.data_pose[1])
                self.appoteta.append(self.q[2])
                self.appopsi.append(math.atan(w*2.85/v))
                # rospy.sleep(0.0025)
               
            # self.send_velocities(v, w)
            # # self.send_velocities(self.v_d[i], -self.w_d[i])
            
            # diff = rospy.get_time() - now
            # rospy.sleep(max_t/len_t + 0.02)
            # #rospy.loginfo("durata ciclo for: " + str(max_t/len_t + 0.0058))

          
            # self.appov.append(v)
            # self.appow.append(w)
            
            
            # self.appox.append(self.data_pose[0])
            # self.appoy.append(self.data_pose[1])
            # self.appoteta.append(self.q[2])

            
            # self.appopsi.append(math.atan(w*2.85/v))


        #stop after time

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
        #twist_msg.linear.x = v * np.cos(theta)
        #twist_msg.linear.y = v * np.sin(theta)
        #twist_msg.angular.z = -w
        
        twist_msg.linear.x = v 
        twist_msg.angular.z = -w
        self.twist_pub.publish(twist_msg)

    def unicycle_cartesian_regulation(self):
            rospy.sleep(0.1)
            # (a, b, theta) = self.get_pose() #ci prendo solo theta
            # (x_odom, y_odom, theta) = self.get_pose() #odometria
            # rospy.loginfo('A=' +str(self.A_park))
            # rospy.loginfo('y_odom=' +str(y_odom))
            # x=self.data_pose[0]-self.x_d[0]
            # y=self.data_pose[1]-self.y_d[0]
            # rospy.loginfo('x=' +str(x))
            # rospy.loginfo('y=' +str(y))
            i=0
            while i<len(self.x_d)-1:
                rospy.loginfo('i_init=' +str(i))
                (a, b, theta) = self.get_pose() #ci prendo solo theta
                x=self.data_pose[0]-self.x_d[i]
                y=self.data_pose[1]-self.y_d[i]
                print("x_odom: {}  and x_d: {}".format(self.data_pose[0], self.x_d[i]))
                print("y_odom: {}  and y_d: {}".format(self.data_pose[1], self.y_d[i]))
                while abs(x) > 0.02 or abs(y) > 0.02 :
                    x=self.data_pose[0]-self.x_d[i]
                    y=self.data_pose[1]-self.y_d[i]
                    (v,w) = cartesian_regulation_control_law(x, y, theta)
                    print("linear:{} and angular:{}".format(v, w))           
                    #move robot
                    # self.send_velocities(v, w, theta)
                    i+=1
                    rospy.sleep(0.1)
                rospy.loginfo('i_fin=' +str(i))
                i+=1
                self.send_velocities(0,0,0)
            rospy.sleep(0.1)

            #stop after time
            self.send_velocities(0,0,0)

    def to_point(self):
        toltheta=0.2
        tol=0.05
        off_inerzia=0 #1.3 #offset di arresto derivante all'inerzia della macchina (visualizzato dal path)
        q_i = self.get_pose()
        
        # valido solo per theta=0
        A_park_inerzia=[self.A_park[0][0],self.A_park[1][0]+off_inerzia,self.A_park[1][0]]   ###################################################################à

        vel=2
        # exit()
        if np.pi/2-toltheta<=q_i[2]<=toltheta+np.pi/2 or -np.pi/2-toltheta<=q_i[2]<=toltheta-np.pi/2:
            while q_i[0]<=A_park_inerzia[0]-tol or q_i[0]>=A_park_inerzia[0]+tol:
                q_i = self.get_pose()
                self.send_velocities(vel,0,0)
                # rospy.loginfo("Sto inviando su x")
                print("x_odom:{} and x_A:{}".format(q_i[0],  round(self.A_park[0][0],4)))
        elif  -toltheta<=q_i[2]<=toltheta or np.pi-toltheta<=q_i[2]<=toltheta+np.pi:
            while q_i[1]<=A_park_inerzia[1]-tol or q_i[1]>=A_park_inerzia[1]+tol:
                q_i = self.get_pose()
                self.send_velocities(vel,0,0)
                # rospy.loginfo("Sto inviando su y")
                print("y_odom:{} and y_A:{}".format(q_i[1],  round(self.A_park[1][0],4)))
        self.send_velocities(0,0,0)
        self.send_velocities(0,0,0)
        self.send_velocities(0,0,0)
        self.send_velocities(0,0,0)
        rospy.loginfo("STOP")
        print("Asse posteriore: x:{} and y:{}. Punto A: x:{} and y:{}".format(self.q[0],  self.q[1],self.A_park[0][0],self.A_park[1][0]))
        

        

if __name__ == "__main__":

    yaml_package_name = rospy.get_param('~yaml_package_name', 'object_spawner')
    yaml_relative_path = rospy.get_param('~yaml_relative_path', '/config/parcheggi2.yaml')
    m = g.parse_yaml(yaml_package_name,yaml_relative_path)

    #prova raccordo per path quadrato
    # tt=Trajectory_tracking()
    # tt.t = np.linspace(0, 5, 1500)
    # trajectory = "cyrcular" 
    # park=g.findpark(tt.q,m)
    # tt.trajectory_generation(trajectory,park)
    # exit()

    #generazione path quadrato ############################################################à
    tt=Trajectory_tracking()
    trajectory = "square" 
    park=g.findpark(tt.q,m)
    tt.trajectory_generation(trajectory,park)
    tt.t = np.linspace(0, 5, 1500)
    # # #move path quadrato
    # # tt.unicycle_cartesian_regulation()
    # # tt.unicycle_linearized_control()
    zeta= 0.1 #parametri per controllo lineare
    a= 0.3
    tt.unicicle_Linear_control(trajectory,zeta,a)
    # exit()
    
    #estrazione punto A (punto finale della cubica ed iniziale del parcheggio)
    
    # tt.t = np.linspace(0, 5, 1500)
    trajectory = "parallel_parking" 

    # park=g.findpark(tt.q,m,tt.get_laser()[1]) # coordinate centro del parcheggio libero (x,y,theta): RICHIAMARE 1 SOLA VOLTA QUANDO SI CALCOLA IL PUNTO A_park
    # park=[0,0,0]
    
    # rospy.loginfo('park='+str(all(elem == 0 for elem in park)))
    # rospy.loginfo('tt.get_laser='+str(tt.get_laser()))

    toltheta=0.1
    if np.pi-toltheta<=tt.q[2]<=np.pi+toltheta or -toltheta<=tt.q[2]<=toltheta:
        a=tt.data_pose[1] #estraggo y odometria
    else: 
        a=tt.data_pose[0] #estraggo x odometria

    # SEZIONE AVANZAMENTO
    while tt.get_laser()[0]==0 and abs(a)<=13: #13 fine strada parcheggi
        
        if tt.get_laser()[0]==1:
            rospy.loginfo('park found')
        else:
            # rospy.loginfo('park not found')
            tt.send_velocities(3,0,0)
        if np.pi-toltheta<=tt.q[2]<=np.pi+toltheta or -toltheta<=tt.q[2]<=toltheta:
            a=tt.data_pose[1] #estraggo y odometria
        else: 
            a=tt.data_pose[0] #estraggo x odometria

    tt.send_velocities(0,0,0)
    tt.send_velocities(0,0,0)
    tt.send_velocities(0,0,0)

    rospy.loginfo('parcheggio trovato='+str(tt.get_laser()[0]))
    if tt.get_laser()[0]==1:
        park=g.findpark(tt.q,m,tt.get_laser()[1]) # coordinate centro del parcheggio libero (x,y,theta): RICHIAMARE 1 SOLA VOLTA QUANDO SI CALCOLA IL PUNTO A_park
        rospy.loginfo('park='+str(park))

        tt.trajectory_generation(trajectory,park) #generazione traiettoria (ritorna anche punto A)
        rospy.loginfo('A ='+str(tt.A_park))
    
    # exit()
    if len(tt.A_park)>0:
        # zeta= 0.9 #parametri per controllo lineare
        # a= 1.45
        # rospy.loginfo('unicycle_cartesian_regulation')
        # tt.unicycle_cartesian_regulation() #regolazione cartesiana per andare dal punto di stop della quadrata al punto A (inizio parcheggio)
        # trajectory = "retta"
        # tt.t = np.linspace(0, 5, 1500) #dichiaro nuovamente il tempo per fare la cubica con meno campioni (in questo modo il self.t viene sovrascritto)
        # tt.trajectory_generation(trajectory,m)
        # exit()
        # tt.unicicle_Linear_control(zeta,a)
        tt.to_point()
        # exit()
        rospy.sleep(1)
        zeta= 0.9 #parametri per controllo lineare
        a= 1.45
        tt.t = np.linspace(0, 5, 1500)
        trajectory = "parallel_parking"  
        tt.trajectory_generation(trajectory,park)
        tt.unicicle_Linear_control(trajectory,zeta,a)
        
    else:
        rospy.loginfo('Nessun parcheggio libero')
    # exit()
    



    #manovra parcheggio
    # exit()
    # tt.t = np.linspace(0, 5, 1500)
    # trajectory = "parallel_parking"
    # tt.trajectory_generation(trajectory,m)
    # tt.unicicle_Linear_control()
    #tt.unicycle_linearized_control() #manovra di parcheggio
    
    #tt.unicycle_cartesian_regulation() # no trajectory needed
    # rospy.spin()
    

