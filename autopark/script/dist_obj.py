#! /usr/bin/env python3

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


# global pos 
# global xgoal_w
# global ygoal_w
# global xgoal
# global ygoal

class laser():
    findpark=0 # flag che indica se ha trovato un parcheggio libero
    park_aff=0 # flag che indica se il parcheggio si trova prima(2), in corrispondenza(1) o dopo(0) 
    flag=1 
    inda=[]
    indb=[]
    rad_obj=[] #0 gradi a dx
    dist_obj=[] #distanza tra laser e oggetto
    posti_liberi_gradi=[]
    posti_liberi_dist=[]
    theta_odom=0
    thetaprec=0
  
    # inf_list=[]

    def __init__(self):
        # rospy.loginfo('init dist')
        # rospy.init_node('rotw5_node')
        mmm=rospy.Subscriber('/scan', LaserScan, self.get_flag) 
        rospy.Subscriber('/ground_truth/state',Odometry, self.odometryCb)
        
    # def unsub(self):
    #     mmm.unregister()

    def get_flag(self,msg):#,sub):
        self.inda=[]
        self.indb=[]
        self.gradi_obj=[] #0 gradi a dx
        self.dist_obj=[] #distanza tra laser e oggetto
        self.posti_liberi_gradi=[]
        self.posti_liberi_dist=[]
        self.theta_odom=self.get_pose()[2]
        theta_p=1000 # assegno a theta parcheggio un valore impossibile che userò come comndizione nel caso in cui l'odometria non rientri nei valori degli if successivi
        if -0.4<=self.theta_odom<=0.4:
            theta_p=0
        elif -0.4-np.pi/2<=self.theta_odom<=0.4-np.pi/2:
            theta_p=-np.pi/2
        elif -0.4+np.pi/2<=self.theta_odom<=0.4+np.pi/2:
            theta_p=np.pi/2
        elif -0.4+np.pi<=self.theta_odom<=0.4+np.pi or -0.4-np.pi<=self.theta_odom<=0.4-np.pi:
            theta_p=np.pi
        else:
            pass
            # rospy.loginfo('Theta odometria ha un valore errato: '+str(self.theta_odom))
        # rospy.loginfo('Theta_p= '+str(theta_p))
        # exit()
        for i in range(len(msg.ranges)):
            if msg.ranges[i]!=math.inf and self.flag==1:   #flag==1 inizio ostacolo
                self.inda.append(i)
                self.flag=0
                self.gradi_obj.append(self.inda[-1]*np.pi/len(msg.ranges))  # np.pi/len(msg.ranges) è il passo per i radianti
                self.dist_obj.append(msg.ranges[i])

            elif msg.ranges[i]==math.inf and self.flag==0:  #flag==1 fino ostacolo
                self.indb.append(i-1)
                self.flag=1
                self.gradi_obj.append(self.indb[-1]*np.pi/len(msg.ranges))
                self.dist_obj.append(msg.ranges[i-1])

            else:
                pass

        # rospy.loginfo('gradi_obj= '+str(self.gradi_obj))
        # rospy.loginfo('dist_obj= '+str(self.dist_obj))
        # rospy.loginfo('inda= '+str(self.inda))
        # rospy.loginfo('indb= '+str(self.indb))

        if  all(elem == math.inf for elem in msg.ranges): # se tutti gli elementi sono inf (nessuna macchina) pongo inda e indb come gli estremi del laser
            self.inda.append(0)
            self.gradi_obj.append(self.inda[-1]*np.pi/len(msg.ranges))  # np.pi/len(msg.ranges) è il passo per i radianti
            self.dist_obj.append(8)
            self.indb.append(len(msg.ranges)-1)
            self.gradi_obj.append(self.indb[-1]*np.pi/len(msg.ranges))
            self.dist_obj.append(8)
            self.alfa=np.pi
            self.beta=0

        #se c'è solo una macchina, per poter calcolare le distanze creo una "finta macchina" larga 1 indice(0,25°) prima o dopo di quella vera 
        elif (len(self.gradi_obj)==2): #ci sono 2 indici , quindi 1 macchina (bisogna distinguere se la macchina sta nel posto prima o dopo)
            if math.degrees(self.gradi_obj[0]+self.gradi_obj[1])<=180: #se indb è prima dei 90 gradi, quindi la macchina è sulla destra (più avanti), quindi inserisco una macchina fittizzia a sinistra
                self.inda.append(len(msg.ranges)-2)
                self.gradi_obj.append(self.inda[-1]*np.pi/len(msg.ranges))  # np.pi/len(msg.ranges) è il passo per i radianti
                self.dist_obj.append(8)
                self.indb.append(len(msg.ranges)-1)
                self.gradi_obj.append(self.indb[-1]*np.pi/len(msg.ranges))
                self.dist_obj.append(8)
                # rospy.loginfo('La macchina sta dopo la nostra, creo una macchina fittizia prima')
            elif math.degrees(self.gradi_obj[0]+self.gradi_obj[1])>180: #se indb è dopo i 90 gradi, quindi la macchina è sulla sinistra (più dietro), quindi inserisco una macchina fittizzia a destra
                self.inda.insert(0,0) #dato che la macchina da aggiungere si trova prima di quella registrata, devo inserire i vari elementi prima di quelli già presenti (quindi NON con append)
                self.gradi_obj.insert(0,self.inda[0]*np.pi/len(msg.ranges))  # np.pi/len(msg.ranges) è il passo per i radianti
                self.dist_obj.insert(0,8)
                self.indb.insert(0,1)
                self.gradi_obj.insert(1,self.indb[0]*np.pi/len(msg.ranges))
                self.dist_obj.insert(1,8)
                # rospy.loginfo('La macchina sta prima della nostra, creo una macchina fittizia dopo')
        else:
            pass
 
     
        # rospy.loginfo('gradi_obj= '+str(self.gradi_obj))
        # rospy.loginfo('dist_obj= '+str(self.dist_obj))
        # rospy.loginfo('inda= '+str(self.inda))
        # rospy.loginfo('indb= '+str(self.indb))
        
        for i in range(len(self.gradi_obj)): #converto la lista da radianti a gradi
            self.gradi_obj[i]=math.degrees(self.gradi_obj[i])

        # rospy.loginfo('len(self.gradi_obj)= '+str(len(self.gradi_obj)))
        # rospy.loginfo('gradi_obj= '+str(self.gradi_obj))
        # rospy.loginfo('dist_obj= '+str(self.dist_obj))
        # rospy.loginfo('inda= '+str(self.inda))
        # rospy.loginfo('indb= '+str(self.indb))
        # rospy.loginfo('len(msg.ranges)= '+str(len(msg.ranges)))
        # exit()

        if (len(self.gradi_obj)>2): #ci sono 4 indici , quindi 2 macchine
            
            i=0
            if theta_p!=1000:
                theta=theta_p-self.theta_odom ########################################################### diff tra theta parcheggio e teta odometria
                while i<=int(len(self.gradi_obj)/2-2): # and self.findpark==0: #i<int(len(self.gradi_obj)/2-2) perchè andiamo da 0 a 0 nel caso di due macchine (solo 1 calcolo)
                    # rospy.loginfo('ho più di due elementi')
                    self.posti_liberi_gradi.append(self.gradi_obj[2*i+1]-self.gradi_obj[2*i+2])   ###################################################### alfa-beta
                    self.beta=math.radians(self.gradi_obj[2*i+1])
                    self.alfa=math.radians(self.gradi_obj[2*i+2])
                    self.gamma=math.radians(self.posti_liberi_gradi[i])
                    self.a=self.dist_obj[2*i+1]
                    self.b=self.dist_obj[2*i+2]
                    # rospy.loginfo("beta-theta :"+str(np.sign(np.cos(self.beta-theta)))+" np.pi-self.alfa+theta :"+str(np.sign(np.cos(self.alfa-theta))))
                    if np.sign(np.cos(self.beta-theta))!=np.sign(np.cos(self.alfa-theta)): #significa che le proiezioni sono discordi (una a destra e una a sinistra), quindi le sommo per calcolare la distanza
                        self.posti_liberi_dist.append(abs(self.a*np.cos(self.beta-theta))+abs(self.b*np.cos(np.pi-self.alfa+theta)))
                    else: #segni concordi, quindi calcolo la distanza come differenza tra la proiezione di a e quella di b (discriminando se il posto e prima o dopo)
                        # proiezione di a - proiezione di b SOLO se il posto libero si trova dopo la macchina (quindi se self.gradi_obj[1]=0.25)
                        if math.degrees(self.gradi_obj[1])==0.25:
                            #self.posti_liberi_dist.append(abs(self.a*np.cos(self.beta-theta)) - abs(self.b*np.cos(self.alfa-theta)))
                            self.posti_liberi_dist.append(abs(abs(self.b*np.cos(self.alfa-theta))-abs(self.a*np.cos(self.beta-theta))))
                        else:
                            #self.posti_liberi_dist.append(abs(self.b*np.cos(self.alfa-theta))-abs(self.a*np.cos(self.beta-theta)))
                            self.posti_liberi_dist.append(abs(abs(self.b*np.cos(self.alfa-theta))-abs(self.a*np.cos(self.beta-theta))))

                    #rospy.loginfo('a= '+str(self.a))
                    #rospy.loginfo('b= '+str(self.b))
                    #rospy.loginfo('alfa= '+str(math.degrees(self.alfa)))
                    #rospy.loginfo('beta= '+str(math.degrees(self.beta)))
                    # rospy.loginfo('destra= '+str(abs(self.a*np.cos(self.beta-theta))))
                    # rospy.loginfo('sinistra= '+str(abs(self.b*np.cos(np.pi-self.alfa+theta))))
                    # rospy.loginfo('Larghezza posto per parcheggiare= '+str(self.posti_liberi_dist[i]))
                    
                    if self.posti_liberi_dist[i]>=5:  
                        self.findpark=1 
                        if abs(self.alfa)>= np.pi/2:
                            if abs(self.beta)>= np.pi/2:
                                self.park_aff=2 # posto libero si trova prima della macchina (precedente)
                            else :
                                self.park_aff=1 # posto libero si trova in corrispondenza della macchina (corrispondenza)
                        else:
                            self.park_aff=0 # posto libero si trova dopo la macchina (successivo)
                    else:
                        self.findpark=0
                        
                    i+=1
            else:
                self.findpark=0
                # rospy.loginfo('Impossibile estratte Theta_p (macchina troppo inclinata)')
        else: 
            
            self.posti_liberi_dist.append(abs(self.dist_obj[0])+abs(self.dist_obj[-1]))
            self.findpark=1  
            if abs(self.alfa)>= np.pi/2:
                if abs(self.beta)>= np.pi/2:
                    self.park_aff=2 # posto libero si trova prima della macchina
                else :
                    self.park_aff=1 # posto libero si trova in corrispondenza della macchina
            else:
                self.park_aff=0 # posto libero si trova dopo la macchina
            # rospy.loginfo('Largezza posto per parcheggiare= '+str(self.posti_liberi_dist[0]))
        
        
        return self.findpark,self.park_aff

    def park(self):
    
        if self.findpark==1:
            a=1
            rospy.loginfo('park found')
        else:
            a=0
            rospy.loginfo('park not found')
        return a,self.park_aff

    def odometryCb(self,msg):
        
        x = round(msg.pose.pose.position.x,4)
        y = round(msg.pose.pose.position.y,4)
        theta = round(self.get_angle_pose(msg.pose.pose),4) 

        y=round(y-1.4*math.cos(theta+np.pi),4) ##spostiamo il punto di riferimento dal centro della macchina al centro dell' asse posteriore
        x=round(x+1.4*math.sin(theta+np.pi),4) ##spostiamo il punto di riferimento dal centro della macchina al centro dell' asse posteriore

        self.q = np.array([x, y, theta])

        return self.q

    #compute angle from quaternion
    def get_angle_pose(self, quaternion_pose):
        q = [quaternion_pose.orientation.x,
                quaternion_pose.orientation.y,
                quaternion_pose.orientation.z,
                quaternion_pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        theta = yaw
        tol=0.1
        if abs(theta-self.thetaprec)>2*math.pi-tol and self.thetaprec!=0:
            theta=theta+2*math.pi-tol
            # rospy.loginfo('STO CORREGGENDO THETA ')
        else:
            pass
        # rospy.loginfo('thetaPRECE  '  + str(self.thetaprec))
        # rospy.loginfo('thetaNOW   '+ str(theta))
        self.thetaprec=theta
        return theta

    def get_pose(self):
        #get robot position updated from callback
        x = self.q[0]
        y = self.q[1]
        theta = self.q[2]
        return np.array([x, y, theta])



if __name__ == "__main__":
    rospy.loginfo('...')
    # a=prova_parcheggio()

    # msg = rospy.wait_for_message("/scan", LaserScan, timeout=5)
    # a.get_flag(msg)  #forzo il costruttore 

    # rospy.loginfo(a.park())
    # sub2= rospy.Subscriber('/steering_robot/odom', Odometry, call_pos)
    # pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # move = Twist()
    # rospy.spin()
