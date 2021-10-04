#!/usr/bin/env python3
import numpy as np
import rospy
import math
import matplotlib.pyplot as plt

#Trajectory generation
class Trajectory_generation():

    def parallel_parking_trajectory(self,q_i, t, a):
        p1=[]
        dot_p1=[]
        p2=[]
        dot_p2=[]
        w1=[]
        w2=[]
        appo=[]
        # a è del centro del parcheggio scelto dal laser scan
        # ci servirà per calcolare C

        # il punto iniziale del parcheggio giace sul quadrato in cui la macchina sta avanzando
        # e ricade esattamente alla metà della lunghezza del parcheggio successivo al posto individuato
        # il punto finale è esattamente il punto P sull'asse posteriore con l'auto tangente all'auto parcheggiata subito dietro

        L = 2.85  #distanza tra semiasse angeriore e posteriore
        d = 2.2   #lunghezza asse anteriore  
        size_y = 3.51  #dim auto lungo y
        size_x = 2.4    #dim auto lingo x
        max_steering_angle = 0.6 #massimo angolo di sterzata
        dim_battistrada = 0.2 #dimensione battistrata ruote
        lunghezza_parcheggio = 4.9
        larghezza_parcheggio=2.5
        wheel_radius=0.33

        psi1R = max_steering_angle
        rR_max = L/math.sin(psi1R)
        rL_max = rR_max + d
        psi1L = math.asin(L / rL_max)
        r = rL_max + wheel_radius    #approssimo il punto più esterno della macchina (possibile impatto) sommando al raggio rL il raggio della ruota
        # esiste un raggio che decreta o meno il fatto che si possa parcheggiare 
        # in una sola manovra larghezza_parcheggio
        offset=(larghezza_parcheggio-size_x)/2
        r_max = math.sqrt((lunghezza_parcheggio-wheel_radius)**2 + (rR_max*math.cos(psi1R)-offset-dim_battistrada/2)**2)
        

 
        beta1 = math.acos((rR_max*math.cos(psi1R)-dim_battistrada/2 -offset)/ r_max)
        beta2 = beta1 #per costruzione 
        a1 = rR_max*math.cos(psi1R) + size_x/2 - dim_battistrada/2

        l=(3/2)*lunghezza_parcheggio-wheel_radius-0.1 #distanza tra le due linee verticali che congiungono i centri e punti A e C
        linea=l-a1*math.sin(beta1)
        a2=linea/math.sin(beta2)

        psi2L = math.atan(L / (a2 - d/2))
        psi2R = math.atan(L / (a2 + d/2))



        psi1=math.atan(L / a1)
        psi2=-math.atan(L / a2)  # il raggio che va dal centro della circonferenza 2 a B è a2

        len_curv=beta2*a2+beta1*a1
        len_s2=round((beta2*a2*len(t))/len_curv)
        len_s1=len(t)-len_s2
        s2=np.linspace(0,beta2*a2, num=len_s2)
        s1=np.linspace(0,beta1*a1, num=len_s1)
      
        dots_c=1.75
        tc = (0 - len_curv + dots_c * t[-1]) / dots_c
    
        prop = int(tc*len(t)/t[-1])
        v1=np.linspace(0, dots_c, num=prop+1)
        v2=np.repeat(dots_c, len(t) - 2*prop)
        # v3=np.repeat(dots_c, prop-1)  # trapezioidale senza rampa di discesa
        v3=np.linspace(dots_c, 0, num=prop-1) #trapezioidale con rampa di discesa
        
        vel=-np.concatenate((v1, v2, v3))

        tol=0.1

        if(q_i[2]<np.pi+tol) and (q_i[2]>np.pi-tol) : # if(theta==3.14): ########################################################################################################################
            C = np.array([[a[0]+0.15], [a[1]-2.45+wheel_radius],[0]])  
            # C = np.array([[21.15], [2.45+wheel_radius],[0]])      #C è il punto finale noto ossia P_park 
            
            B = np.array([[a1 - a1*math.cos(beta1) + C[0,0]]   ,   [C[1,0] + a1 * math.sin(beta1)] , [0]])  #B va calcolato in base alla geometria del problema

            A =np.array([[B[0,0]+(a2-a2*math.cos(beta2))], [B[1,0]+linea],[0]])
            # generazione del path 

            c2=np.array([A[0]-a2,A[1],A[2]])
            c1=np.array([C[0]+a1,C[1],C[2]])
            for i in range(0,len_s2):
                p1.append(c2 + np.dot(self.rotx(np.pi),np.array([[a2*math.cos(s2[i]/a2)], [a2*math.sin(s2[i]/a2)], [0]]))) 
                dot_p1.append(np.dot(self.rotx(np.pi),np.array([[-vel[i]*math.sin(s2[i]/a2)] , [vel[i]*math.cos(s2[i]/a2)], [0]])))
                w1.append(math.tan(psi1)*(vel[i])/L)
  
            dot_p1=np.array(dot_p1)    
            p1=np.array(p1)

            for i in range(0,len_s1):
                p2.append(c1 + np.dot(self.rotz(np.pi-beta1),np.array([[a1*math.cos(s1[i]/a1)] ,[a1*math.sin(s1[i]/a1)], [0]])))
                dot_p2.append(np.dot(self.rotz(np.pi-beta1),np.array([[- vel[i+len_s2]*math.sin(s1[i]/a1)] , [vel[i+len_s2]*math.cos(s1[i]/a1)] ,[0]])))
                w2.append(math.tan(psi2)*(vel[i+len_s2])/L)

            p2=np.array(p2)
            dot_p2=np.array(dot_p2)
            pt1=np.concatenate((p1, p2))
            
            dotp =np.concatenate((dot_p1, dot_p2))
            dot_px  = dotp[:,0]
            dot_py  = dotp[:,1]
            w=np.concatenate((w1, w2))

            tetad = []
            for i in range(0,len(dot_px)):
                td = math.atan2( dot_py[i] , dot_px[i] ) + math.pi
                tetad.append(td) 

            for i in range(0,len(tetad)):
                if (tetad[i] != math.pi) :
                    tetad[i] = tetad[i]  - math.pi/2
                else:
                    pass
            for i in range(0,len(tetad)):
                if (i>0) and (i <len(tetad)-1) and (tetad[i] == math.pi):
                    tetad[i] = tetad[i-1]
                else:
                    pass

            psi1 = np.repeat(psi1 , len_s2)
            psi2 = np.repeat(psi2 , len_s1)
            psi = np.concatenate((psi1 , psi2))
            rospy.loginfo("Lato 1")

        elif(q_i[2]<-np.pi/2+tol) and (q_i[2]>-np.pi/2-tol) : # elif(theta==-1.57):  ########################################################################################################################

            C = np.array([[a[0]+2.45-wheel_radius], [a[1]+0.15],[0]])      #C è il punto finale noto ossia P_park 

            B = np.array([[C[0,0] - a1*math.sin(beta1)]   ,   [C[1,0] + a1 - a1 * math.cos(beta1)] , [0]])  #B va calcolato in base alla geometria del problema

            A =np.array([[B[0,0]-linea], [B[1,0]+(a2-a2*math.cos(beta2))],[0]])

            # generazione del path 

            c2=np.array([A[0],A[1]-a2,A[2]])
            c1=np.array([C[0],C[1]+a1,C[2]])

            for i in range(0,len_s2):
                p1.append(c2 + np.dot(self.rotz(np.pi/2-beta1),np.array([[a2*math.cos(s2[i]/a2)], [a2*math.sin(s2[i]/a2)], [0]]))) 
                dot_p1.append(np.dot(self.rotz(np.pi/2-beta1),np.array([[-vel[i]*math.sin(s2[i]/a2)] , [vel[i]*math.cos(s2[i]/a2)], [0]])))
                w1.append(math.tan(psi1)*(vel[i])/L)

            dot_p1=np.array(dot_p1)
            p1=np.array(p1)
            p1=p1[::-1]   #invertiamo l'array p1 
            dot_p1=dot_p1[::-1]

            for i in range(0,len_s1):
                p2.append(c1 + np.dot(np.dot(self.rotx(np.pi),self.rotz(np.pi/2)),np.array([[a1*math.cos(s1[i]/a1)] ,[a1*math.sin(s1[i]/a1)], [0]])))
                dot_p2.append(np.dot(np.dot(self.rotx(np.pi),self.rotz(np.pi/2)),np.array([[- vel[i+len_s2]*math.sin(s1[i]/a1)] , [vel[i+len_s2]*math.cos(s1[i]/a1)] ,[0]])))
                w2.append(math.tan(psi2)*(vel[i+len_s2])/L)

            p2=np.array(p2)
            dot_p2=np.array(dot_p2)
            p2=p2[::-1]   #invertiamo l'array p2
            dot_p2=dot_p2[::-1]

            pt1=np.concatenate((p1,p2))
            dotp =np.concatenate((dot_p1,dot_p2))
            dot_px  = dotp[:,0]
            dot_py  = dotp[:,1]
            w=np.concatenate((w1, w2))

            tetad = []
            for i in range(0,len(dot_px)):
                td = math.atan2( dot_py[i] , dot_px[i] ) 
                tetad.append(td) 

            for i in range(0,len(tetad)):
                if (tetad[i] != math.pi) :
                    tetad[i] = tetad[i]  - math.pi/2
                else:
                    pass
            for i in range(0,len(tetad)):
                if (i>0) and (i <len(tetad)-1) and (tetad[i] == -math.pi/2):
                    tetad[i] = tetad[i-1]
                else:
                    pass
        
            psi1 = np.repeat(psi1 , len_s2)
            psi2 = np.repeat(psi2 , len_s1)
            psi = np.concatenate((psi1 , psi2))
            rospy.loginfo("Lato 2")

        elif(q_i[2]<0+tol) and (q_i[2]>0-tol) : # elif(theta==0)   ##############################################################################################################################à

            C = np.array([[a[0]-0.15],[a[1]+2.45-wheel_radius],[0]])      #C è il punto finale noto ossia P_park 
                
            B = np.array([[C[0,0] - (a1 - a1*math.cos(beta1))]   ,   [C[1,0] - a1 * math.sin(beta1)] , [0]])  #B va calcolato in base alla geometria del problema

            A =np.array([[B[0,0]-(a2-a2*math.cos(beta2))], [B[1,0]-linea],[0]])

            # generazione del path 
            c2=np.array([A[0]+a2,A[1],A[2]])
            c1=np.array([C[0]-a1,C[1],C[2]])

            for i in range(0,len_s2):
                p1.append(c2 + np.dot(self.rotz(np.pi-beta1),np.array([[a2*math.cos(s2[i]/a2)], [a2*math.sin(s2[i]/a2)], [0]]))) 
                dot_p1.append(np.dot(self.rotz(np.pi-beta1),np.array([[-vel[i]*math.sin(s2[i]/a2)] , [vel[i]*math.cos(s2[i]/a2)], [0]])))
                w1.append(math.tan(psi1)*(vel[i])/L)
               
            dot_p1=np.array(dot_p1)
            p1=np.array(p1)
            p1=p1[::-1]   #invertiamo l'array p1 
            dot_p1=dot_p1[::-1]

            for i in range(0,len_s1):
                p2.append(c1 + np.dot(self.rotx(np.pi),np.array([[a1*math.cos(s1[i]/a1)] ,[a1*math.sin(s1[i]/a1)], [0]])))
                dot_p2.append(np.dot(self.rotx(np.pi),np.array([[- vel[i+len_s2]*math.sin(s1[i]/a1)] , [vel[i+len_s2]*math.cos(s1[i]/a1)] ,[0]])))
                w2.append(math.tan(psi2)*(vel[i+len_s2])/L)

            p2=np.array(p2)
            dot_p2=np.array(dot_p2)
            p2=p2[::-1]   #invertiamo l'array p2
            dot_p2=dot_p2[::-1]

            pt1=np.concatenate((p1,p2))
            dotp =np.concatenate((dot_p1,dot_p2))
            dot_px  = dotp[:,0]
            dot_py  = dotp[:,1]
            w=np.concatenate((w1, w2))

            tetad = []
            for i in range(0,len(dot_px)):
                td = math.atan2( dot_py[i] , dot_px[i] ) 
                tetad.append(td) 

            for i in range(0,len(tetad)):
                if (tetad[i] != math.pi) :
                    tetad[i] = tetad[i]  - math.pi/2
                else:
                    pass
            for i in range(0,len(tetad)):
                if (i>0) and (i <len(tetad)-1) and (tetad[i] == -math.pi/2):
                    tetad[i] = tetad[i-1]
                else:
                    pass

            psi1 = np.repeat(psi1 , len_s2)
            psi2 = np.repeat(psi2 , len_s1)
            psi = np.concatenate((psi1 , psi2))
            rospy.loginfo("Lato 3")

        elif(q_i[2]<np.pi/2+tol) and (q_i[2]>np.pi/2-tol) : # elif(theta==1.57):  ########################################################################################################################

            C = np.array([[a[0]-2.45+wheel_radius], [a[1]-0.15],[0]])      #C è il punto finale noto ossia P_park 
            
            B = np.array([[C[0,0] + a1*math.sin(beta1)]   ,   [C[1,0] - a1 + a1 * math.cos(beta1)] , [0]])  #B va calcolato in base alla geometria del problema

            A =np.array([[B[0,0]+linea], [B[1,0]-a2+a2*math.cos(beta2)],[0]])
            
            # generazione del path 

            c2=np.array([A[0],A[1]+a2,A[2]])
            c1=np.array([C[0],C[1]-a1,C[2]])
            
            for i in range(0,len_s2):
                p1.append(c2 + np.dot(self.rotz(3*np.pi/2-beta2),np.array([[a2*math.cos(s2[i]/a2)], [a2*math.sin(s2[i]/a2)], [0]]))) 
                dot_p1.append(np.dot(self.rotz(3*np.pi/2-beta2),np.array([[-vel[i]*math.sin(s2[i]/a2)] , [vel[i]*math.cos(s2[i]/a2)], [0]])))
                w1.append(math.tan(psi1)*(vel[i])/L)
        

            dot_p1=np.array(dot_p1)
            p1=np.array(p1)
            p1=p1[::-1]   #invertiamo l'array p1 
            dot_p1=dot_p1[::-1]

            for i in range(0,len_s1):
                p2.append(c1 + np.dot(np.dot(self.rotx(np.pi),self.rotz(3*np.pi/2)),np.array([[a1*math.cos(s1[i]/a1)] ,[a1*math.sin(s1[i]/a1)], [0]])))
                dot_p2.append(np.dot(np.dot(self.rotx(np.pi),self.rotz(3*np.pi/2)),np.array([[- vel[i+len_s2]*math.sin(s1[i]/a1)] , [vel[i+len_s2]*math.cos(s1[i]/a1)] ,[0]])))
                w2.append(math.tan(psi2)*(vel[i+len_s2])/L)

            p2=np.array(p2)
            dot_p2=np.array(dot_p2)
            p2=p2[::-1]   #invertiamo l'array p2
            dot_p2=dot_p2[::-1]    

            pt1=np.concatenate((p1,p2))
            dotp =np.concatenate((dot_p1,dot_p2))
            dot_px  = dotp[:,0]
            dot_py  = dotp[:,1]
            w=np.concatenate((w1, w2))

            tetad = []
            for i in range(0,len(dot_px)):
                td = math.atan2( dot_py[i] , dot_px[i] ) 
                tetad.append(td) 
            tetad[0] =  math.pi/2
            tetad[-1] =  math.pi/2
            for i in range(1,len(tetad)-1):
                if (tetad[i] != math.pi/2) :
                    tetad[i] = tetad[i]  - math.pi/2
                else:
                    pass
            for i in range(0,len(tetad)):
                if (i>0) and (i <len(tetad)-1) and (tetad[i] == -math.pi/2):
                    tetad[i] = tetad[i-1]
                else:
                    pass
        
            psi1 = np.repeat(psi1 , len_s2)
            psi2 = np.repeat(psi2 , len_s1)
            psi = np.concatenate((psi1 , psi2))
            rospy.loginfo("Lato 4")

        # PRINT DI CONTROLLO
        # rospy.loginfo("A= "+str(A))
        # rospy.loginfo("p1(0)= "+str(p1[0]))
        # rospy.loginfo("B= "+str(B))
        # rospy.loginfo("p1(end)= "+str(p1[-1]))
        # rospy.loginfo("p2(0)= "+str(p2[0]))
        # rospy.loginfo("C= "+str(C))
        # rospy.loginfo("p2(end)= "+str(p2[-1]))
        # rospy.loginfo("len(x_d)= "+str(len(p1[:,0])))
        # rospy.loginfo("len(y_d)= "+str(len(p1[:,1])))
        # exit()

        # GRAFICI 
        # plot = plt.figure(1)
        # plt.plot(pt1[:,0],pt1[:,1])
        # # plt.plot(p1[:,0],p1[:,1],p2[:,0],p2[:,1])
        # plt.title('path Totale')
        # plt.xlabel('x_d')
        # plt.ylabel('y_d')
        # plt.axis('equal')

        # plt.show()
        # exit()

        # for i in range(0,len(dot_px)):
        #     appo.append(dot_px[i]**2+dot_py[i]**2)
        # plot = plt.figure(2)
        # plt.plot(t,appo)
        # plt.title('velocità')
        # plt.xlabel('t')
        # plt.ylabel('v')
        # plt.axis('equal')
        
        # plot = plt.figure(3)
        # plt.plot(t,vel)
        # plt.title('velocità')
        # plt.xlabel('t')
        # plt.ylabel('v')
        # plt.axis('equal')
        # plot = plt.figure(4)
        # plt.plot(t,w)
        # plt.title('vel angolare')
        # plt.xlabel('t')
        # plt.ylabel('w')
        # plt.axis('equal')

        # plot = plt.figure(5)
        # plt.plot(t,tetad)
        # plt.title('tetad')
        # plt.xlabel('t')
        # plt.ylabel('tetad')
        # plt.axis('equal')

        # plot = plt.figure(6)
        # plt.plot(t,psi)
        # plt.title('psi')
        # plt.xlabel('t')
        # plt.ylabel('psi')
        # plt.axis('equal')

        # plt.show()
        # exit()
        
        return (pt1[:,0],pt1[:,1],dot_px,dot_py,vel,w , tetad , psi, A)

    def rotx(self,ang):
        return np.array([[ 1,              0,               0],
                         [ 0,  math.cos(ang),  -math.sin(ang)],
                         [ 0,  math.sin(ang),   math.cos(ang)]])
    
    def rotz(self,ang):
        return np.array([[math.cos(ang),  -math.sin(ang),   0],
                         [math.sin(ang),   math.cos(ang),   0],
                        [            0,               0,   1]])
    
    def roty(self,ang):
        return np.array([[math.cos(ang),    0,     math.sin(ang)],
                         [0,                1,                 0],
                         [-math.sin(ang),   0,     math.cos(ang)]])

    def cubic_trajectory(self, q_i, q_f, k, t):
        
        ''' cubic interpolation from initial to final pose
        Parameters:
            q_i: array of initial pose
            q_f: array of finale pose
            k: intial and final geometric velocity
            t: array of times  

        Returns:
            x: array of desired x
            y: array of desired y
            v: array of derired linear velocity
            w: array of desired angular velocity
            theta: array of desired angular pose                  
        '''
        # intial pose coordinates
        x_i = q_i[0]
        y_i = q_i[1]
        # finale pose coordinates
        x_f = q_f[0]
        y_f = q_f[1]
        #len_curva=0
                

            
        s = t/t[-1]
        tau = 1/t[-1]
          
        b_x = k*np.cos(q_i[2]) + 3*q_i[0]
        b_y = k*np.sin(q_i[2]) + 3*q_i[1]

        a_x = k*np.cos(q_f[2]) - 3*q_f[0]
        a_y = k*np.sin(q_f[2]) - 3*q_f[1]

        # print("Theta iniziale= {}  Theta finale= {} ".format(q_i[2],q_f[2]))
        #Cartesian cubic path interpolating intial-final poses
        x = x_f*s**3 - x_i*(s-1)**3 + a_x * s**2 * (s-1) + b_x * s * (s-1)**2
        y = y_f*s**3 - y_i*(s-1)**3 + a_y * s**2 * (s-1) + b_y * s * (s-1)**2
        z=np.repeat(0, len(x))

        # equazione retta che passa per punto finale e iniziale della cubica (retta rispetto alal quale specchio la cubica)
        m=(y[-1]-y[0])/(x[-1]-x[0])
        # rospy.loginfo("m= "+str(m))
        # rospy.loginfo("angolo=atan(m)= "+str(math.degrees(math.atan(m))))
        # exit()
        ang=math.atan(m) #inclinazione della retta rispetto all'asse x (in radianti)
        q=y[0]-m*x[0]
        # q=(x[0]*y[-1]-y[0]*x[-1])/(x[0]-x[-1])
        y_retta=m*x+q
        x_r=x
        # dist=[]
        # dist_y=[]
        # dist_x=[]
        # x_r=[]
        # y_r=[]
        # plot1 = plt.figure(1)
        # plt.plot(x,y,x_r,y_retta)
        # plt.title('path')
        # plt.xlabel('x_d')
        # plt.ylabel('y_d')
        # plt.axis('equal')

        
        # specchio la cubica per avere le concavità corrette
        # x_a=((1-m**2)*x+2*m*y-2*m*q)/(1+m**2)
        # y_a=(2*m*x+((m**2)-1)*y+2*q)/(1+m**2)
        # x=x_a
        # y=y_a

        # for i in range (0,len(x)):
        #     dist.append(abs(y[i]-(m*x[i]+q))/(np.sqrt(1+m**2))) #distanza tra il singolo punto e la retta
        #     dist_y.append(dist[i]*math.cos(ang)) # scomposizione della distanza su x e y
        #     dist_x.append(dist[i]*math.sin(ang))
        #     x_r.append((y[i]-q+(x[i]/m))/(m+1/m)) #coordinate del punto di intersezione tra la retta y_r e la retta perpendicolare ad essa che passa per x[i],y[i]
        #     y_r.append(m*x_r[i]+q)
        #     if y[i]>y_r[i]:
        #         # x[i]+=2*dist_x[i]
        #         y[i]-=2*dist_y[i]
        #     else:
        #         # x[i]-=2*dist_x[i]
        #         y[i]+=2*dist_y[i]
        #     if x[i]>x_r[i]:
        #         x[i]-=2*dist_x[i] 
        #     else:
        #         x[i]+=2*dist_x[i]

        # for i in range (0,len(x)):
           
        #     x_r.append((y[i]-q+(x[i]/m))/(m+1/m)) #coordinate del punto di intersezione tra la retta y_r e la retta perpendicolare ad essa che passa per x[i],y[i]
        #     y_r.append(m*x_r[i]+q)
        #     dist_y.append(abs(abs(y[i])-abs(y_r[i])))
        #     dist_x.append(abs(abs(x[i])-abs(x_r[i])))
        #     if y[i]>y_r[i]:
        #         # x[i]+=2*dist_x[i]
        #         y[i]-=2*dist_y[i]
        #     else:
        #         # x[i]-=2*dist_x[i]
        #         y[i]+=2*dist_y[i]
        #     if x[i]>x_r[i]:
        #         x[i]-=2*dist_x[i]
        #     else:
        #         x[i]+=2*dist_x[i]

        # rospy.loginfo("dist_y"+str(dist_y))
        # plot2 = plt.figure(2)
        # plt.plot(x_a,y_a,x_r,y_retta)
        # plt.title('path specchiato')
        # plt.xlabel('x_d')
        # plt.ylabel('y_d')
        # plt.axis('equal')
        # plot3 = plt.figure(3)
        # plt.plot(x,dist_y,x, dist_x)
        # plt.title('dist_y')
        # plt.xlabel('x_d')
        # plt.ylabel('y_d')
        # plt.show()
        # exit()
        #ruoto e ribalto la traiettoria (concavità generate errate)
        # appo=[]
        # appo_n=[]
        # toltheta=0.2
        # segno=1
        
        # for i in range(0,len(x)):
        #     if  -toltheta<=q_i[2]<=toltheta or np.pi-toltheta<=q_i[2]<=toltheta+np.pi:
        #         appo=y
        #         if np.sign(appo[-1-i])!=np.sign(appo[i]):
        #             c1=abs(abs(appo[-1-i])+abs(appo[i]))/2 # distanza tra il singolo punto e il centro della curva
        #             c2=abs(abs(appo[i])+abs(appo[len(x)-i-1]))/2
        #         else:
        #             c1=abs(abs(appo[-1-i])-abs(appo[i]))/2 # distanza tra il singolo punto e il centro della curva
        #             c2=abs(abs(appo[i])-abs(appo[len(x)-i-1]))/2
        #         if -toltheta<=q_i[2]<=toltheta:
        #             segno=-1
        #         else:
        #             segno=1

        #     elif np.pi/2-toltheta<=q_i[2]<=toltheta+np.pi/2 or -np.pi/2-toltheta<=q_i[2]<=toltheta-np.pi/2:
        #         appo=x
        #         # rospy.loginfo("siamo in pi/2 o -pi/2")
        #         if np.sign(appo[-1-i])!=np.sign(appo[i]):
        #             c1=abs(abs(appo[-1-i])+abs(appo[i]))/2 # distanza tra il singolo punto e il centro della curva
        #             c2=abs(abs(appo[i])+abs(appo[len(x)-i-1]))/2
        #         else:
        #             c1=abs(abs(appo[-1-i])-abs(appo[i]))/2 # distanza tra il singolo punto e il centro della curva
        #             c2=abs(abs(appo[i])-abs(appo[len(x)-i-1]))/2
        #         if np.pi/2-toltheta<=q_i[2]<=toltheta+np.pi/2:
        #             segno=1
        #         else:
        #             segno=-1
        #     else:
        #         rospy.loginfo("Angolo theta al di fuori della tolleranza. Auto troppo inclinata!")
          

        #     if i<len(x)/2:
        #         appo_n.append(appo[i]+(2*c1*segno))
        #     else:
        #         appo_n.append(appo[i]+(2*c2*(-segno)))

        # plot2 = plt.figure(2)
        # plt.plot(appo_n,y)
        # plt.title('path specchiato')
        # plt.xlabel('x_d')
        # plt.ylabel('y_d')
        
        # if np.pi/2-toltheta<=q_i[2]<=toltheta+np.pi/2 or -np.pi/2-toltheta<=q_i[2]<=toltheta-np.pi/2:
        #     x=appo_n
        #     c_x=x[int(len(x)/2-1)]
        #     c_y=y[int(len(y)/2-1)]
        #     rospy.loginfo("c_y= "+str(c_y))
        #     rospy.loginfo("c_x= "+str(c_x))
        #     for i in range(0,len(x)):
        #         x[i]-=c_x
        #         y[i]-=c_y
        #     plot3 = plt.figure(3)
        #     plt.plot(x,y)
        #     plt.title('path specchiato traslato in 0')
        #     plt.xlabel('x_d')
        #     plt.ylabel('y_d')
        # else: 
        #     y=appo_n
        #     c_x=x[int(len(x)/2-1)]
        #     c_y=y[int(len(y)/2-1)]
        #     for i in range(0,len(y)):
        #         y[i]-=c_y
        #         x[i]-=c_x
        #     plot3 = plt.figure(3)
        #     plt.plot(x,y)
        #     plt.title('path specchiato traslato in 0')
        #     plt.xlabel('x_d')
        #     plt.ylabel('y_d')
        
            
        
        # plt.show()
        # # exit()
        # xyz=np.concatenate(([x], [y], [z]))
     
        # for i in range(0,len(x)):
        #     xyz[:,i]=np.dot(self.rotz(-np.pi/2),xyz[:,i])
        # x=xyz[0,:]
        # y=xyz[1,:]
        

        # plot4 = plt.figure(4)
        # plt.plot(x,y)
        # plt.title('path specchiato, traslato in 0 e ruotato')
        # plt.xlabel('x_d')
        # plt.ylabel('y_d')
        
        
        
        # if np.pi/2-toltheta<=q_i[2]<=toltheta+np.pi/2 or -np.pi/2-toltheta<=q_i[2]<=toltheta-np.pi/2:
        #     for i in range(0,len(x)):
        #         x[i]+=c_y
        #         y[i]+=c_x
        # else: 
        #     for i in range(0,len(y)):
        #         y[i]+=c_x
        #         x[i]+=c_y

        # plot5 = plt.figure(5)
        # plt.plot(x,y)
        # plt.title('path in c')
        # plt.xlabel('x_d')
        # plt.ylabel('y_d')
        # plt.show()

        # exit()
        
        #Compute first derivative
        xp = 3*x_f*s**2 - 3*x_i*(s-1)**2 + a_x*(3*s**2 -2*s) + b_x*(s-1)*(3*s-1)
        yp = 3*y_f*s**2 - 3*y_i*(s-1)**2 + a_y*(3*s**2 -2*s) + b_y*(s-1)*(3*s-1)

        # xp_a=((1-m**2)*xp+2*m*yp)/(1+m**2)
        # yp_a=(2*m*xp+((m**2)-1)*yp)/(1+m**2)
        # xp=xp_a
        # yp=yp_a

        dots_c = 1.75
        dots_in = 0.5
        acc_sc = 1

        tc = abs((dots_c - dots_in) / acc_sc)
        prop = int(tc*len(t)/t[-1])
        # rospy.loginfo(len(t))
        # rospy.loginfo(prop)
        # exit()
        v1=np.linspace(dots_in, dots_c, num=prop+1)
        v2=np.repeat(dots_c, len(t) - 2*prop)
        v3=np.linspace(dots_c, dots_in, num=prop-1)
        
        vel=np.concatenate((v1, v2, v3))

        #We can compute the geometric velocity and the posture angle theta
        v = (np.sqrt(xp**2 + yp**2)/np.sqrt(xp**2 + yp**2)) *vel
        # v = (np.sqrt(xp**2 + yp**2))
        
        # theta = np.arctan2(yp,xp) #- math.pi
        # tol=0.4
        # if -tol<=q_i[2]<=tol:
        #     theta = np.arctan2(-yp,xp) #-yp oppure -pi/2
        #     rospy.loginfo("theta=0")
        # elif -tol-np.pi/2<=q_i[2]<=tol-np.pi/2:
        #     theta = np.arctan2(xp,yp)+np.pi/2
        #     theta[0]=theta[1] #correzione primo e ultimo punto dovuta allo spkie di atan2
        #     theta[-1]=theta[-2]
        #     rospy.loginfo("theta=-1.57")
        # elif -tol+np.pi/2<=q_i[2]<=tol+np.pi/2:
        #     theta = np.arctan2(yp,xp)
        #     rospy.loginfo("theta=1.57")
        # elif -tol+np.pi<=q_i[2]<=tol+np.pi or -tol-np.pi<=q_i[2]<=tol-np.pi:
        #     theta = np.arctan2(-yp,xp)
        #     theta[0]=theta[1] #correzione primo e ultimo punto dovuta allo spkie di atan2
        #     theta[-1]=theta[-2]
        #     rospy.loginfo("theta=3.14 (o -3.14)")
        # else:
        #     rospy.loginfo("Macchina troppo inclinata, theta oltre la tolleranza")

        # # per theta=0
        # theta = np.arctan2(-yp,xp) #-yp oppure -pi/2
        
        # per theta=-1.57
        # theta = np.arctan2(xp,yp)+np.pi/2
        # theta[0]=theta[1] #correzione primo e ultimo punto dovuta allo spkie di atan2
        # theta[-1]=theta[-2]

        # per theta=1.57
        # theta = np.arctan2(yp,xp)# -2*np.pi

        # per theta=3.14
        # theta = np.arctan2(-yp,xp)
        # theta[0]=theta[1] #correzione primo e ultimo punto dovuta allo spkie di atan2
        # theta[-1]=theta[-2]
 
        # theta=[q_i[2]]    
        # for i in range(1,len(x)):
        #     theta.append(math.atan2(y[i]-y[i-1],x[i]-x[i-1]))

        #Compute second derivative
        xpp = 6*x_f*s - 6*x_i*(s-1) + a_x*(6*s-2) + b_x*(6*s-4)
        ypp = 6*y_f*s - 6*y_i*(s-1) + a_y*(6*s-2) + b_y*(6*s-4)

        #Compute the angular velocity
        


        # for i in range (0,len(t)):
        #     if theta[i]<theta[0]-1.0472: ###################limito a 60°
        #         theta[i]=theta[0]-1.0472
        #         xp[i]=v[i]*math.cos(theta[i])
        #         yp[i]=v[i]*math.sin(theta[i])
        
        w = ((ypp*xp - xpp*yp)/(xp**2 + yp**2))*vel/(np.sqrt(xp**2 + yp**2)) 
        #w = -((ypp*xp - xpp*yp)/(v**2))

        # wmax_steer con velocità di crociera
        a = math.tan(math.atan2(2.85,(2.85/math.tan(0.6)+2.2/2)))*dots_c/2.85
        
        for i in range (0,len(w)):
            if abs(w[i])>a:
                wmaxc=math.copysign(a,w[i])
                w[i]=wmaxc
        len_curva=0
        for i in range(0,len(x)-2):
                len_curva+=math.sqrt((y[i+1]-y[i])**2+(x[i+1]-x[i])**2)
        # rospy.loginfo("la curva è lunga :" + str(len_curva))

        
        # rospy.loginfo(vel)
        # rospy.loginfo(len(vel))
        # rospy.loginfo(len(vel1))
        # rospy.loginfo(len(vel2))
        # rospy.loginfo(len(vel3))
        
        # rospy.loginfo(x[0])
        # rospy.loginfo(x[1])
        # rospy.loginfo(abs(abs(x[1])-abs(x[0])))
        # rospy.loginfo(math.atan(abs(abs(y[1])-abs(y[0]))/abs(abs(x[1])-abs(x[0]))))
        # rospy.loginfo("x[0]= "+str(x[0])+" y[0]= "+str(y[0]))
        # # rospy.loginfo("x[0]= "+str(x[0])+" y[0]= "+str(y[0]))
        # plot1 = plt.figure(1)
        # plt.plot(x,y)
        # plt.title('path')
        # plt.xlabel('x_d')
        # plt.ylabel('y_d')
        # plt.axis('equal')

        # plot2 = plt.figure(2)
        # plt.plot(t , v)
        # plt.title('velocità desiderata')
        # plt.xlabel('time')
        # plt.ylabel('v_d')


        # plot3 = plt.figure(3)
        # plt.plot(t , w)
        # plt.title('velocità angolare desiderata')
        # plt.xlabel('time')
        # plt.ylabel('w_d')

        # plot4 = plt.figure(4)
        # plt.plot(t , theta)
        # plt.title('angolo theta desiderato')
        # plt.xlabel('time')
        # plt.ylabel('theta')

        # plot5 = plt.figure(5)
        # plt.plot(t , xp)
        # plt.title('Xp')
        # plt.xlabel('time')
        # plt.ylabel('xp')

        # plot6 = plt.figure(6)
        # plt.plot(t , yp)
        # plt.title('Yp')
        # plt.xlabel('time')
        # plt.ylabel('yp')

        # plt.show()
        # exit()
        
        # w=np.repeat(0, len(x_r) )
        # theta=np.repeat(q_i[2], len(x_r) )
        # return [x, y, v, w, theta]
        return [x_r, y_retta, v, w, theta]
    
    def retta(self, q_i, q_f, t):

        toltheta=0.2
        if np.pi/2-toltheta<=q_i[2]<=toltheta+np.pi/2 or -np.pi/2-toltheta<=q_i[2]<=toltheta-np.pi/2:
            x = np.linspace(q_i[0], q_f[0], len(t)) +1.4
            y = np.repeat(q_i[1], len(t))
        elif  -toltheta<=q_i[2]<=toltheta or np.pi-toltheta<=q_i[2]<=toltheta+np.pi:
            x = np.repeat(q_i[0], len(t))   
            y = np.linspace(q_i[1], q_f[1], len(t)) +1.4
        else:
            rospy.loginfo("Theta al di fuori della tolleranza")
        
        dots_c = 1.75
        dots_in = 0.5
        acc_sc = 1
        tc = abs((dots_c - dots_in) / acc_sc)
        prop = int(tc*len(t)/t[-1])

        theta=np.repeat(q_i[2], len(t))
        w=np.repeat(0, len(t))
        v1=np.linspace(dots_in, dots_c, num=prop+1)
        v2=np.repeat(dots_c, len(t) - 2*prop)
        v3=np.linspace(dots_c, dots_in, num=prop-1)
        
        v=np.concatenate((v1, v2, v3))

        return [x, y, v, w, theta]
    
    def cyrcular_trajectory(self, t,x_c,y_c,iniz,fin,theta_in,ang_lato,vx_in,vy_in): # iniz è il punto iniziale dell'arco e fin il punto finale. ang_lato è la rotazione da imporre per avere l'arco sui diversi lati
        R = 4 # circle radius
        len_arc=R*np.pi/2 #lunghezza arco sempre fissa!
        # v_d_val = len_arc/(t[-1]-t[0]) # FORSE TEMPO TROPPO PICCOLO, QUINDI W GRANDE  
        v_d_val = len_arc/(1.06*(t[-1]-t[0])) #impongo un tempo a caso per vedere se funziona 
        # rospy.loginfo("v arco= "+str(v_d_val))
        w_d_val = v_d_val/R # [rad/s], const angular vel
        # rospy.loginfo("v arco= "+str(w_d_val))
        # exit()
        # desired trajectory, starting from origin (t=0)
        ang=np.linspace(0, np.pi/2, len(t))
        # rospy.loginfo("centro= "+str(x_c)+" "+str(y_c))
        x_d = iniz[0]+ R * np.cos(ang)
        y_d = iniz[1]+ R * np.sin(ang) -R
     
        dotx_d = -R*np.sin(ang) # time derivative of x_d
        doty_d =  +R*np.cos(ang) # time derivative of y_d
        # theta_d = theta_in+np.arctan2(doty_d, dotx_d)
        theta_d = theta_in+np.arctan2(doty_d, dotx_d)
        # for i in range(0,len(theta_d)): #aggiusto il theta desiderato per manternerlo nell'intervallo 0-2*pi
        #         if theta_d[i]>=2*np.pi:
        #             appo=(theta_d[i])//(2*np.pi)
        #             theta_d[i]=theta_d[i]-appo*2*np.pi
        # desired velocity
        v_d = np.sqrt(dotx_d**2 + doty_d**2)       
        w_d = w_d_val * np.ones(len(t)) # array of same const w/ length=array of time
        xy=[]
        # rospy.loginfo("vettore= "+str(x_d))
        # exit()
        for i in range(0,len(x_d)):
      
            xy.append(np.dot(self.rotz(ang_lato),np.array([x_d[i] ,y_d[i], 0])))
            x_d[i]=xy[i][0]
            y_d[i]=xy[i][1]
        
        return [x_d, y_d, v_d, w_d, theta_d, dotx_d, doty_d]
        
    def detecting_square_trajectory(self,q_i):

        # la posizione inziale (x0,y0) deve essere passata direttamente dal model spawn 
        # devo realizzare una sottoscrizione ad un topic x estrarre la posa iniziale

        #set dei parametri 
        lato=24.28*2 ###################################################################################################################à
        # lato=27*2
        vel_kmh = 25            #   velocità in km/h
        vel = vel_kmh*10/36     #   velocità in m/s
        passo = 0.01         

        #definisco i punti del quadrato 
        A_point = [lato/2 , lato/2]
        B_point = [-lato/2 , lato/2]
        C_point = [-lato/2 , -lato/2]
        D_point = [lato/2 , -lato/2]

        # A_point = [lato/2 , lato/2]
        # B_point = [-lato/2 , lato/2]
        # C_point = [-lato/2 , -lato/2]
        # D_point = [lato/2 , -lato/2]
        
        # intial pose coordinates
        self.x0 = q_i[0]
        self.y0 = q_i[1]
        
        x0 = self.x0
        y0 = self.y0
        # rospy.loginfo("x0= "+str(x0))
        # rospy.loginfo("y0= "+str(y0))
        a = math.pi
        # round(lato/2,1)
        if(x0 == round(lato/2,1)) and (y0 != round(lato/2,1)):
            #sono sul segmento DA
            yaw_angle = a 
        if(x0 == -round(lato/2,1)) and (y0 != -round(lato/2,1)):
            #sono sul segmento BC
            yaw_angle = 0
        if(y0 == round(lato/2,1)) and (x0 != -round(lato/2,1)):
            #sono sul segmento AB
            yaw_angle = -a/2
        if(y0 == -round(lato/2,1)) and (x0 != round(lato/2,1)):
        #sono sul segmento CD
            yaw_angle = a/2
        
       
        if (y0 == round(lato/2,1)):
            if(x0 == round(lato/2,1)):
                # sono in A
                # rospy.loginfo("sono nel vertice A del quadrato")
                (x,y, dotx, doty, t,yaw) = self.A_trajectory( A_point, B_point, C_point, D_point , lato, passo, vel,yaw_angle)
                
                return [x,y, dotx, doty, t ,yaw]
                

            elif(x0 == -round(lato/2,1)):
                # sono in B
                # rospy.loginfo("sono nel vertice B del quadrato")
                (x,y, dotx, doty, t,yaw) = self.B_trajectory( A_point, B_point, C_point, D_point , lato, passo, vel,yaw_angle)
                
                return [x,y, dotx, doty, t,yaw]
            else:
                pass
        
        if ( y0 == -round(lato/2,1)):

            if( x0 == -round(lato/2,1)):
                #sono in C
                # rospy.loginfo("sono nel vertice C del quadrato")
                (x,y, dotx, doty, t,yaw) = self.C_trajectory( A_point, B_point, C_point, D_point , lato, passo, vel,yaw_angle)
                
                return [x,y, dotx, doty, t,yaw]

            elif( x0 == round(lato/2,1)):
                #sono in D
                # rospy.loginfo("sono nel vertice D del quadrato")
                (x,y, dotx, doty, t,yaw) = self.D_trajectory( A_point, B_point, C_point, D_point , lato, passo, vel,yaw_angle)
                
                return [x,y, dotx, doty, t,yaw]

            else:
                rospy.loginfo("Non sono in un vertice del quadrato")

        
        if(y0 == round(lato/2,1)):
            # sono in 1
            rospy.loginfo("Sono nel lato 1 del quadrato")
            (x,y, dotx, doty, t,yaw) = self.trajectory_1( A_point, B_point, C_point, D_point , lato, passo, vel,yaw_angle)
            return [x,y, dotx, doty, t,yaw]
        elif(y0 == -round(lato/2,1)):
            # sono in 3
            rospy.loginfo("Sono nel lato 3 del quadrato")
            (x,y, dotx, doty, t,yaw,v_d,w_d) = self.trajectory_3( A_point, B_point, C_point, D_point , lato, passo, vel,yaw_angle)
            return [x,y, dotx, doty, t,yaw,v_d,w_d]
        elif(x0 == round(lato/2,1)):
            # sono in 4
            rospy.loginfo("Sono nel lato 4 del quadrato")
            (x,y, dotx, doty, t,yaw) = self.trajectory_4( A_point, B_point, C_point, D_point , lato, passo, vel,yaw_angle)
            return [x,y, dotx, doty, t,yaw]
        elif(x0 == -round(lato/2,1)):
            # sono in 2
            rospy.loginfo("Sono nel lato 2 del quadrato")
            (x,y, dotx, doty, t ,yaw) = self.trajectory_2( A_point, B_point, C_point, D_point , lato, passo, vel,yaw_angle)
            return [x,y, dotx, doty, t,yaw]
        else:
            # non sono sul quadrato
            rospy.logdebug("Position Error -- No square building")
        

        
    def A_trajectory(self,puntoA, puntoB, puntoC, puntoD , lato, passo, vel,yaw_angle):

        elementi=round(lato/(passo*vel))

        a = math.pi
        tfin1=elementi*passo
        t1=np.linspace(0,tfin1,elementi)
        tfin2=tfin1+elementi*passo
        t2=np.linspace(tfin1,tfin2,elementi)
        tfin3=tfin2+elementi*passo
        t3=np.linspace(tfin2,tfin3,elementi)
        tfin4=tfin3+elementi*passo
        t4=np.linspace(tfin3,tfin4,elementi)
        t=np.concatenate([t1,t2,t3,t4])

        # segmento AB
        x_c1 = puntoA[0]
        y_c1 = puntoA[1]

        x1 = lato/2*np.ones(len(t1)) - vel*t1
        y1 = np.repeat( y_c1 , len(t1))
        
        yAB = np.repeat(yaw_angle,len(t1))

        # segmento BC
        x_c2 = puntoB[0]
        y_c2 = puntoB[1]

        x2 = np.repeat( x_c2 , len(t2))
        y2 = lato/2*np.ones(len(t2)) - vel*t1

        yBC = np.repeat(yaw_angle + a/2,len(t1))

        # segmento CD
        x_c3 = puntoC[0]
        y_c3=  puntoC[1]

        x3 = -lato/2 *np.ones(len(t3)) + vel*t1
        y3 = np.repeat( y_c3 , len(t3))

        yCD = np.repeat(yaw_angle + a,len(t1))

        # segmento DA
        x_c4 = puntoD[0]
        y_c4=  puntoD[1]

        x4 = np.repeat( x_c4 , len(t4))
        y4 = -lato/2 *np.ones(len(t3)) + vel*t1

        yDA = np.repeat(yaw_angle + a,len(t1))

        
        x=np.concatenate([x1,x2,x3,x4])
        y=np.concatenate([y1,y2,y3,y4])
        
        #applico la trapezioidale
        # len_curva=0
        # for i in range(0,len(x)-1):
        #         len_curva+=math.sqrt((y[i+1]-y[i])**2+(x[i+1]-x[i])**2)
        # dots_c=vel
        # tc = (0 - len_curv + dots_c * t[-1]) / dots_c

        # prop = int(tc*len(t)/t[-1])
        # v1=np.linspace(0, dots_c, num=prop+1)
        # v2=np.repeat(dots_c, len(t) - 2*prop)
        # v3=np.repeat(dots_c, prop-1)#v3=np.linspace(dots_c, 0, num=prop-1)
        
        

        # calcolo delle derivate
        dotx1 =np.repeat(-vel,len(t1))
        doty1 =np.repeat(0,len(t1))
        
        dotx2 =np.repeat(0,len(t2))
        doty2 =np.repeat(-vel,len(t2))
        
        dotx3 =np.repeat(vel,len(t3))
        doty3 =np.repeat(0,len(t3))
        
        dotx4 =np.repeat(0,len(t4))
        doty4 =np.repeat(vel,len(t4))
        dotx = np.concatenate([dotx1, dotx2,dotx3,dotx4])
        doty =np.concatenate([doty1, doty2,doty3,doty4])

        yaw = np.concatenate([yAB , yBC, yCD, yDA])  

        # vel=-np.concatenate((v1, v2, v3)) #costruzione vettore velocità trapezioidale  
        
        return [x,y, dotx, doty, t,yaw]
    
    def B_trajectory(self,puntoA, puntoB, puntoC, puntoD , lato, passo, vel,yaw_angle):

        elementi=round(lato/(passo*vel))
        a = math.pi
        tfin1=elementi*passo
        t1=np.linspace(0,tfin1,elementi)
        tfin2=tfin1+elementi*passo
        t2=np.linspace(tfin1,tfin2,elementi)
        tfin3=tfin2+elementi*passo
        t3=np.linspace(tfin2,tfin3,elementi)
        tfin4=tfin3+elementi*passo
        t4=np.linspace(tfin3,tfin4,elementi)
        t=np.concatenate([t1,t2,t3,t4])

        # segmento BC
        x_c1 = puntoB[0]
        y_c1 = puntoB[1]

        yBC = np.repeat(yaw_angle,len(t1))

        x1 = np.repeat( x_c1 , len(t1))
        y1 = lato/2*np.ones(len(t1)) - vel*t1

        # segmento CD
        x_c2 = puntoC[0]
        y_c2 = puntoC[1]

        yaw_angle = yaw_angle+a/2
        yCD = np.repeat(yaw_angle,len(t1))
        

        x2 = -lato/2 *np.ones(len(t2)) + vel*t1
        y2 = np.repeat( y_c2 , len(t2))

        # segmento DA
        x_c3 = puntoD[0]
        y_c3=  puntoD[1]

        yaw_angle = yaw_angle+a/2
        yDA = np.repeat(yaw_angle,len(t1))

        x3 = np.repeat( x_c3 , len(t3))
        y3 = -lato/2 *np.ones(len(t3)) + vel*t1

        # segmento AB
        x_c4 = puntoA[0]
        y_c4=  puntoA[1]

        yaw_angle = yaw_angle+a/2
        yAB = np.repeat(yaw_angle,len(t1))

        x4 = lato/2*np.ones(len(t4)) - vel*t1
        y4 = np.repeat( y_c4 , len(t4))
        
        x=np.concatenate([x1,x2,x3,x4])
        y=np.concatenate([y1,y2,y3,y4])
        
        # calcolo delle derivate
        dotx1 =np.repeat(0,len(t1))
        doty1 =np.repeat(-vel,len(t1))
        
        dotx2 =np.repeat(+vel,len(t2))
        doty2 =np.repeat(0,len(t2))
        
        dotx3 =np.repeat(0,len(t3))
        doty3 =np.repeat(vel,len(t3))
        
        dotx4 =np.repeat(-vel,len(t4))
        doty4 =np.repeat(0,len(t4))
        dotx = np.concatenate([dotx1, dotx2,dotx3,dotx4])
        doty =np.concatenate([doty1, doty2,doty3,doty4])
        yaw = np.concatenate([yBC , yCD , yDA , yAB]) 
    
        return [x,y, dotx, doty, t,yaw]
    
    def C_trajectory(self,puntoA, puntoB, puntoC, puntoD , lato, passo, vel,yaw_angle):

        elementi=round(lato/(passo*vel))
        a = math.pi
        tfin1=elementi*passo
        t1=np.linspace(0,tfin1,elementi)
        tfin2=tfin1+elementi*passo
        t2=np.linspace(tfin1,tfin2,elementi)
        tfin3=tfin2+elementi*passo
        t3=np.linspace(tfin2,tfin3,elementi)
        tfin4=tfin3+elementi*passo
        t4=np.linspace(tfin3,tfin4,elementi)
        t=np.concatenate([t1,t2,t3,t4])

        # segmento CD
        x_c1 = puntoC[0]
        y_c1 = puntoC[1]

        yCD = np.repeat(yaw_angle,len(t1))

        x1 = -lato/2 *np.ones(len(t1)) + vel*t1
        y1 = np.repeat( y_c1 , len(t1))


        # segmento DA
        x_c2 = puntoD[0]
        y_c2 = puntoD[1]

        yaw_angle = yaw_angle+a/2
        yDA = np.repeat(yaw_angle,len(t1))

        x2 = np.repeat( x_c2 , len(t2))
        y2 = -lato/2 *np.ones(len(t2)) + vel*t1

        # segmento AB
        x_c3 = puntoA[0]
        y_c3=  puntoA[1]

        yaw_angle = yaw_angle+a/2
        yAB = np.repeat(yaw_angle,len(t1))

        x3 = lato/2*np.ones(len(t3)) - vel*t1
        y3 = np.repeat( y_c3 , len(t3))

        # segmento BC
        x_c4 = puntoB[0]
        y_c4=  puntoB[1]

        yaw_angle = yaw_angle+a/2
        yBC = np.repeat(yaw_angle,len(t1))
        
        x4 = np.repeat( x_c4 , len(t4))
        y4 = lato/2*np.ones(len(t4)) - vel*t1

        x=np.concatenate([x1,x2,x3,x4])
        y=np.concatenate([y1,y2,y3,y4])
        
        # calcolo delle derivate
        dotx1 =np.repeat(vel,len(t1))
        doty1 =np.repeat(0,len(t1))
        
        dotx2 =np.repeat(0,len(t2))
        doty2 =np.repeat(vel,len(t2))
        
        dotx3 =np.repeat(-vel,len(t3))
        doty3 =np.repeat(0,len(t3))
        
        dotx4 =np.repeat(0,len(t4))
        doty4 =np.repeat(-vel,len(t4))
        dotx = np.concatenate([dotx1, dotx2,dotx3,dotx4])
        doty =np.concatenate([doty1, doty2,doty3,doty4])
        yaw = np.concatenate([yCD , yDA , yAB , yBC]) 
     
        
        return [x,y, dotx, doty, t,yaw] 

    def D_trajectory(self,puntoA, puntoB, puntoC, puntoD , lato, passo, vel,yaw_angle):

        elementi=round(lato/(passo*vel))
        a = math.pi
        tfin1=elementi*passo
        t1=np.linspace(0,tfin1,elementi)
        tfin2=tfin1+elementi*passo
        t2=np.linspace(tfin1,tfin2,elementi)
        tfin3=tfin2+elementi*passo
        t3=np.linspace(tfin2,tfin3,elementi)
        tfin4=tfin3+elementi*passo
        t4=np.linspace(tfin3,tfin4,elementi)
        t=np.concatenate([t1,t2,t3,t4])

        # segmento DA
        x_c1 = puntoD[0]
        y_c1 = puntoD[1]

        yDA = np.repeat(yaw_angle,len(t1))

        x1 = np.repeat( x_c1 , len(t1))
        y1 = -lato/2 *np.ones(len(t1)) + vel*t1


        # segmento AB
        x_c2 = puntoA[0]
        y_c2 = puntoA[1]

        yaw_angle = yaw_angle+a/2
        yAB = np.repeat(yaw_angle,len(t1))

        x2 = lato/2*np.ones(len(t2)) - vel*t1
        y2 = np.repeat( y_c2 , len(t2))


        # segmento BC
        x_c3 = puntoB[0]
        y_c3=  puntoB[1]

        yaw_angle = yaw_angle+a/2
        yBC = np.repeat(yaw_angle,len(t1))

        x3 = np.repeat( x_c3 , len(t3))
        y3 = lato/2*np.ones(len(t3)) - vel*t1

        # segmento CD
        x_c4 = puntoC[0]
        y_c4=  puntoC[1]

        yaw_angle = yaw_angle+a/2
        yCD = np.repeat(yaw_angle,len(t1))
        
        x4 = -lato/2 *np.ones(len(t4)) + vel*t1
        y4 = np.repeat( y_c4 , len(t4))


        x=np.concatenate([x1,x2,x3,x4])
        y=np.concatenate([y1,y2,y3,y4])
        
        # calcolo delle derivate
        dotx1 =np.repeat(0,len(t1))
        doty1 =np.repeat(vel,len(t1))
        
        dotx2 =np.repeat(-vel,len(t2))
        doty2 =np.repeat(0,len(t2))
        
        dotx3 =np.repeat(0,len(t3))
        doty3 =np.repeat(-vel,len(t3))
        
        dotx4 =np.repeat(vel,len(t4))
        doty4 =np.repeat(0,len(t4))
        dotx = np.concatenate([dotx1, dotx2,dotx3,dotx4])
        doty =np.concatenate([doty1, doty2,doty3,doty4])
        yaw = np.concatenate([yDA , yAB , yBC , yCD]) 
        
        return [x,y, dotx, doty, t,yaw] 
    
    def trajectory_1(self,puntoA, puntoB, puntoC, puntoD , lato, passo, vel,yaw_angle):

        modulo1 = math.sqrt((puntoB[0] - self.x0)**2 + (puntoB[1] - self.y0)**2)
        modulo2 = math.sqrt((self.x0 - puntoA[0])**2 + (self.y0 - puntoA[1])**2)
        #differenti numero di step per il calcolo dei tempi
        elementi = round(lato/(passo*vel))
        elementi1=round(abs(puntoB[0] - self.x0) / ( passo*vel) ) 
        elementi2 = round(abs(puntoA[0] - self.x0) / ( passo*vel) ) 
        a = math.pi
        tempo_primo_pezzo = elementi1*passo
        tempi_primo_pezzo = np.linspace(0,tempo_primo_pezzo,elementi1)
        tempo_lato_quadrato = elementi*passo
        tempi_lato_quadrato = np.linspace(0,tempo_lato_quadrato,elementi)
        tempo_ultimo_pezzo = elementi2*passo
        tempi_ultimo_pezzo = np.linspace(0,tempo_ultimo_pezzo,elementi2)
        #calcolo dei tempi
        tfin1=elementi1*passo
        t1=np.linspace(0,tfin1,elementi1)
        tfin2=tfin1+elementi*passo
        t2=np.linspace(tfin1,tfin2,elementi)
        tfin3=tfin2+elementi*passo
        t3=np.linspace(tfin2,tfin3,elementi)
        tfin4=tfin3+elementi*passo
        t4=np.linspace(tfin3,tfin4,elementi)
        tfin5=tfin4+elementi2*passo
        t5=np.linspace(tfin4,tfin5,elementi2)
        t=np.concatenate([t1,t2,t3,t4,t5])

        # segmento x0y0_B
        x_c1 = self.x0
        y_c1 = self.y0

        y0B = np.repeat(yaw_angle,len(t1))

        x1 = x_c1*np.ones(len(t1)) +  (vel*tempi_primo_pezzo) * (puntoB[0] - x_c1)/modulo1
        y1 = np.repeat( y_c1 , len(t1))

        # segmento BC
        x_c2 = puntoB[0]
        y_c2 = puntoB[1]

        yaw_angle = yaw_angle+a/2
        yBC = np.repeat(yaw_angle,len(t2))

        x2 = np.repeat( x_c2 , len(t2))
        y2 = lato/2*np.ones(len(t2)) - vel*tempi_lato_quadrato

        # segmento CD
        x_c3 = puntoC[0]
        y_c3=  puntoC[1]

        yaw_angle = yaw_angle+a/2
        yCD = np.repeat(yaw_angle,len(t2))

        x3 = -lato/2 *np.ones(len(t3)) + vel*tempi_lato_quadrato
        y3 = np.repeat( y_c3 , len(t3))

        # segmento DA
        x_c4 = puntoD[0]
        y_c4=  puntoD[1]

        yaw_angle = yaw_angle+a/2
        yDA = np.repeat(yaw_angle,len(t2))

        x4 = np.repeat( x_c4 , len(t4))
        y4 = -lato/2 *np.ones(len(t3)) + vel*tempi_lato_quadrato

        #segmento A_x0y0

        x_c5 = puntoA[0]
        y_c5 = puntoA[1]

        yA0 = np.repeat(yaw_angle + a/2 , len(t5))

        x5 = puntoA[0]*np.ones(len(t5)) +  (vel*tempi_ultimo_pezzo) * ( x_c1 - x_c5 )/modulo2
        y5 = np.repeat( y_c5 , len(t5))
        
        x=np.concatenate([x1,x2,x3,x4,x5])
        y=np.concatenate([y1,y2,y3,y4,y5])
        
        # calcolo delle derivate
        dotx1 =np.repeat(vel*(puntoB[0] - x_c1) / modulo1,len(t1))
        doty1 =np.repeat(0,len(t1))
        
        dotx2 =np.repeat(0,len(t2))
        doty2 =np.repeat(-vel,len(t2))
        
        dotx3 =np.repeat(vel,len(t3))
        doty3 =np.repeat(0,len(t3))
        
        dotx4 =np.repeat(0,len(t4))
        doty4 =np.repeat(vel,len(t4))

        dotx5 =np.repeat(vel*(x_c1 - x_c5)/modulo2,len(t5))
        doty5 =np.repeat(0,len(t5))

        dotx = np.concatenate([dotx1, dotx2,dotx3,dotx4,dotx5])
        doty =np.concatenate([doty1, doty2,doty3,doty4,doty5])
        yaw = np.concatenate([y0B , yBC , yCD , yDA , yA0]) 

        
        return [x,y, dotx, doty, t,yaw]

    def trajectory_2(self,puntoA, puntoB, puntoC, puntoD , lato, passo, vel,yaw_angle):

        modulo1 = math.sqrt((puntoC[0] - self.x0)**2 + (puntoC[1] - self.y0)**2)
        modulo2 = math.sqrt((self.x0 - puntoB[0])**2 + (self.y0 - puntoB[1])**2)

        #differenti numero di step per il calcolo dei tempi
        elementi = round(lato/(passo*vel))
        elementi1=round(abs(puntoC[1] - self.y0) / ( passo*vel) ) 
        elementi2 = round(abs(self.y0 - puntoB[1]) / ( passo*vel) ) 
        a = math.pi
        tempo_primo_pezzo = elementi1*passo
        tempi_primo_pezzo = np.linspace(0,tempo_primo_pezzo,elementi1)
        tempo_lato_quadrato = elementi*passo
        tempi_lato_quadrato = np.linspace(0,tempo_lato_quadrato,elementi)
        tempo_ultimo_pezzo = elementi2*passo
        tempi_ultimo_pezzo = np.linspace(0,tempo_ultimo_pezzo,elementi2)

        #calcolo dei tempi
        tfin1=elementi1*passo
        t1=np.linspace(0,tfin1,elementi1)
        tfin2=tfin1+elementi*passo
        t2=np.linspace(tfin1,tfin2,elementi)
        tfin3=tfin2+elementi*passo
        t3=np.linspace(tfin2,tfin3,elementi)
        tfin4=tfin3+elementi*passo
        t4=np.linspace(tfin3,tfin4,elementi)
        tfin5=tfin4+elementi2*passo
        t5=np.linspace(tfin4,tfin5,elementi2)
        t=np.concatenate([t1,t2,t3,t4,t5])

        # segmento x0y0_C
        x_c1 = self.x0
        y_c1 = self.y0

        y0C = np.repeat(yaw_angle,len(t1))

        x1 = np.repeat( x_c1 , len(t1))
        y1 =  y_c1*np.ones(len(t1)) +  (vel*tempi_primo_pezzo) * (puntoC[1] - y_c1)/modulo1


        # segmento CD
        x_c2 = puntoC[0]
        y_c2 = puntoC[1]

        yaw_angle = yaw_angle+a/2
        yCD = np.repeat(yaw_angle,len(t2))

        x2 = -lato/2 *np.ones(len(t2)) + vel*tempi_lato_quadrato
        y2 = np.repeat( y_c2 , len(t2))

        # segmento DA
        x_c3 = puntoD[0]
        y_c3=  puntoD[1]

        yaw_angle = yaw_angle+a/2
        yDA = np.repeat(yaw_angle,len(t2))

        x3 = np.repeat( x_c3 , len(t3))
        y3 = -lato/2 *np.ones(len(t3)) + vel*tempi_lato_quadrato

        # segmento AB
        x_c4 = puntoA[0]
        y_c4 = puntoA[1]

        yaw_angle = yaw_angle+a/2
        yAB = np.repeat(yaw_angle,len(t2))

        x4 = lato/2*np.ones(len(t4)) - vel*tempi_lato_quadrato
        y4 = np.repeat( y_c4 , len(t4))

        #segmento B_x0y0

        x_c5 = puntoB[0]
        y_c5 = puntoB[1]

        yB0 = np.repeat(yaw_angle + a/2 , len(t5))

        x5 = np.repeat( x_c5 , len(t5))
        y5 = y_c5*np.ones(len(t5)) +  (vel*tempi_ultimo_pezzo) * (y_c1 - y_c5)/modulo2
        
        x=np.concatenate([x1,x2,x3,x4,x5])
        y=np.concatenate([y1,y2,y3,y4,y5])
        
        # calcolo delle derivate
        dotx1 =np.repeat(0,len(t1))
        doty1 =np.repeat(vel*(puntoC[1] - y_c1)/modulo1,len(t1))
        
        dotx2 =np.repeat(vel,len(t2))
        doty2 =np.repeat(0,len(t2))
        
        dotx3 =np.repeat(0,len(t3))
        doty3 =np.repeat(vel,len(t3))
        
        dotx4 =np.repeat(-vel,len(t4))
        doty4 =np.repeat(0,len(t4))

        dotx5 =np.repeat(0,len(t5))
        doty5 =np.repeat(vel*(y_c1 - y_c5)/modulo2,len(t5))

        dotx = np.concatenate([dotx1, dotx2,dotx3,dotx4,dotx5])
        doty =np.concatenate([doty1, doty2,doty3,doty4,doty5])
        yaw = np.concatenate([y0C , yCD, yDA , yAB, yB0]) 
        
        
        return [x,y, dotx, doty, t,yaw]

    def trajectory_3(self,puntoA, puntoB, puntoC, puntoD , lato, passo, vel,yaw_angle):
        rac=20.28 ####################################################################################
        # rac=23
        modulo1 = math.sqrt((puntoD[0] - self.x0)**2 + (puntoD[1] - self.y0)**2)
        modulo2 = math.sqrt((self.x0 - puntoC[0])**2 + (self.y0 - puntoC[1])**2)

        #differenti numero di step per il calcolo dei tempi
        elementi = round(lato/(passo*vel))
        elementi1=round(abs(puntoD[0] - self.x0) / ( passo*vel) ) 
        elementi2 = round(abs(self.x0  - puntoC[0]) / ( passo*vel) ) 
        a = math.pi
        tempo_primo_pezzo = elementi1*passo
        tempi_primo_pezzo = np.linspace(0,tempo_primo_pezzo,elementi1)
        tempo_lato_quadrato = elementi*passo
        tempi_lato_quadrato = np.linspace(0,tempo_lato_quadrato,elementi)
        tempo_ultimo_pezzo = elementi2*passo
        tempi_ultimo_pezzo = np.linspace(0,tempo_ultimo_pezzo,elementi2)

        #calcolo dei tempi
        tfin1=elementi1*passo
        t1=np.linspace(0,tfin1,elementi1)
        tfin2=tfin1+elementi*passo
        t2=np.linspace(tfin1,tfin2,elementi)
        tfin3=tfin2+elementi*passo
        t3=np.linspace(tfin2,tfin3,elementi)
        tfin4=tfin3+elementi*passo
        t4=np.linspace(tfin3,tfin4,elementi)
        tfin5=tfin4+elementi2*passo
        t5=np.linspace(tfin4,tfin5,elementi2)
        t=np.concatenate([t1,t2,t3,t4,t5])

        # segmento x0y0_D
        x_c1 = self.x0
        y_c1 = self.y0

        

        x1 = x_c1*np.ones(len(t1)) +  (vel*tempi_primo_pezzo) * (puntoD[0] - x_c1)/modulo1
        
        #taglio i vettori fino ai punti da cui deve iniziare il raccordo circolare
        # rospy.loginfo("len t1= "+str(len(t1)))
        # rospy.loginfo("t1 end= "+str(t1[-1]))
        x_appo=[]
        t1_appo=[]
        i1=0
        i2=0
        t_d1=[]
        # rospy.loginfo ("len t1="+str(len(t1)))
        for i in range (0,len(x1)):
            if round(x1[i],2)<rac:
                    i2=i+1
            if -rac<=x1[i]<=rac :
                x_appo.append(x1[i])  
                t1_appo.append(t1[i])
        x1=x_appo

        t_d1=np.concatenate([t_d1,t1[i2:len(t1)]])
        t1=t1_appo
        y0D = np.repeat(yaw_angle,len(t1))
        y1 = np.repeat( y_c1 , len(x1))
        
        # segmento DA
        x_c2 = puntoD[0]
        y_c2=  puntoD[1]

        yaw_angle = yaw_angle+a/2
        

        
        y2 = -lato/2 *np.ones(len(t2)) + vel*tempi_lato_quadrato
        t2_appo=[]
        y_appo=[]
        i1=0
        i2=0
        for i in range (0,len(y2)):
            if round(y2[i],2)<-rac:
                    i1=i+1
            if round(y2[i],2)<rac:
                    i2=i+1
            if -rac<=y2[i]<=rac :
                y_appo.append(y2[i])  
                t2_appo.append(t2[i]) 
        t_d1=np.concatenate([t_d1,t2[0:i1]])
        # rospy.loginfo ("t_d1 1="+str(t_d1[0]))
        # rospy.loginfo ("t_d1 end="+str(t_d1[-1]))
        # exit()
        t_d2=[]
        t_d2=np.concatenate([t_d2,t2[i2:len(t2)]])
        t2=t2_appo
        yDA = np.repeat(yaw_angle,len(t2))
        y2=y_appo
        x2 = np.repeat( x_c2 , len(y2))
        
        
        # segmento AB
        x_c3 = puntoA[0]
        y_c3 = puntoA[1]

        yaw_angle = yaw_angle+a/2
        

        x3 = lato/2*np.ones(len(t3)) - vel*tempi_lato_quadrato
        

        i1=0
        i2=0
        x_appo=[]
        t3_appo=[]
        for i in range (0,len(x3)):
           
            if round(x3[i],2)>-rac:
                    i2=i+1
            if round(x3[i],2)>rac:
                    i1=i+1
            if -rac<=x3[i]<=rac :
                x_appo.append(x3[i]) 
                t3_appo.append(t3[i])   
        # rospy.loginfo("x3= "+str(x3))
        x3=x_appo
        t_d2=np.concatenate([t_d2,t3[0:i1]])
        t_d3=[]
        t_d3=np.concatenate([t_d3,t3[i2:len(t3)]])
        t3=t3_appo
        yAB = np.repeat(yaw_angle,len(t2))
        y3 = np.repeat( y_c3 , len(x3))
        # rospy.loginfo ("x3="+str(len(x3)))
        # rospy.loginfo ("y3="+str(len(y3)))

        # segmento BC
        x_c4 = puntoB[0]
        y_c4 = puntoB[1]

        yaw_angle = yaw_angle+a/2
        

        
        y4 = lato/2*np.ones(len(t4)) - vel*tempi_lato_quadrato
        i1=0
        i2=0
        y_appo=[]
        t4_appo=[]
        for i in range (0,len(y4)):
            if round(y4[i],2)>-rac:
                    i2=i+1
            if round(y4[i],2)>rac:
                    i1=i+1
            if -rac<=y4[i]<=rac :
                y_appo.append(y4[i])
                t4_appo.append(t4[i])     
        y4=y_appo

        t_d3=np.concatenate([t_d3,t4[0:i1]])
        t_d4=[]
        t_d4=np.concatenate([t_d4,t4[i2:len(t4)]])
        t4=t4_appo
        yBC = np.repeat(yaw_angle,len(t2))
        x4 = np.repeat( x_c4 , len(y4))
        
        
        # rospy.loginfo ("x4="+str(len(x4)))
        # rospy.loginfo ("y4="+str(len(y4)))

        #segmento C_x0y0

        x_c5 = puntoC[0]
        y_c5 = puntoC[1]

        

        x5 = x_c5*np.ones(len(t5)) +  (vel*tempi_ultimo_pezzo) * (x_c1 - x_c5)/modulo2
        

        x_appo=[]
        i1=0
        t5_appo=[]
 
        for i in range (0,len(x5)):
            if round(x5[i],2)<-rac:
                    i1=i+1
            # if round(x5[i],2)>rac:
            #         i1=i+1
            if -rac<=x5[i]<=rac :
                x_appo.append(x5[i])  
                t5_appo.append(t5[i])     
        x5=x_appo   
        t_d4=np.concatenate([t_d4,t5[0:i1]])
        t5=t5_appo  
        yC0 = np.repeat(yaw_angle + a/2 , len(t5))   
        y5 = np.repeat( y_c5 , len(x5))

        
        # rospy.loginfo("t1= "+str(t1[0]))
        # rospy.loginfo("t1= "+str(t1[-1]))
        # rospy.loginfo("t_d1= "+str(t_d1[0]))
        # rospy.loginfo("t_d1= "+str(t_d1[-1]))
        # rospy.loginfo("t2= "+str(t2[0]))
        # rospy.loginfo("t2= "+str(t2[-1]))
        # rospy.loginfo("t_d2= "+str(t_d2[0]))
        # rospy.loginfo("t_d2= "+str(t_d2[-1]))
        # rospy.loginfo("t3= "+str(t3[0]))
        # rospy.loginfo("t3= "+str(t3[-1]))
        # rospy.loginfo("t_d3= "+str(t_d3[0]))
        # rospy.loginfo("t_d3= "+str(t_d3[-1]))
        # rospy.loginfo("t4= "+str(t4[0]))
        # rospy.loginfo("t4= "+str(t4[-1]))
        # rospy.loginfo("t_d4= "+str(t_d4[0]))
        # rospy.loginfo("t_d4= "+str(t_d4[-1]))
        # rospy.loginfo("t5= "+str(t5[0]))
        # rospy.loginfo("t5= "+str(t5[-1]))
        

      
      
        # exit() 
        
        # rospy.loginfo ("x5="+str(len(x5)))
        # rospy.loginfo ("y5="+str(len(y5)))
   
        
        
        # x=np.concatenate([x_d1])
        # y=np.concatenate([y_d1])

        # plot0 = plt.figure(0)
        # plt.title('arco')
        # plt.plot(x_d1,y_d1)
       

        # rospy.loginfo ("x="+str(len(x)))
        # rospy.loginfo ("y="+str(len(y)))
        # exit()
        
        # calcolo delle derivate
        dotx1 =np.repeat(vel*(puntoD[0] - x_c1)/modulo1,len(t1))
        doty1 =np.repeat(0,len(t1))
        
        dotx2 =np.repeat(0,len(t2))
        doty2 =np.repeat(vel,len(t2))
        
        dotx3 =np.repeat(-vel,len(t3))
        doty3 =np.repeat(0,len(t3))
        
        dotx4 =np.repeat(0,len(t4))
        doty4 =np.repeat(-vel,len(t4))

        dotx5 =np.repeat(vel*(x_c1 - x_c5)/modulo2,len(t5))
        doty5 =np.repeat(0,len(t5))

        (x_d1, y_d1, v_d1, w_d1, theta_d1, dotx_d1, doty_d1)=self.cyrcular_trajectory(t_d1,rac,rac,[x3[0],y3[0]],[x2[-1],y2[-1]],0,3*np.pi/2,dotx1[-1],doty1[-1])
        (x_d2, y_d2, v_d2, w_d2, theta_d2, dotx_d2, doty_d2)=self.cyrcular_trajectory(t_d2,rac,rac,[x3[0],y3[0]],[x2[-1],y2[-1]],y0D[-1],0,dotx2[-1],doty2[-1])
        (x_d3, y_d3, v_d3, w_d3, theta_d3, dotx_d3, doty_d3)=self.cyrcular_trajectory(t_d3,rac,rac,[x3[0],y3[0]],[x2[-1],y2[-1]],yDA[-1],np.pi/2,dotx3[-1],doty3[-1])
        (x_d4, y_d4, v_d4, w_d4, theta_d4, dotx_d4, doty_d4)=self.cyrcular_trajectory(t_d4,rac,rac,[x3[0],y3[0]],[x2[-1],y2[-1]],yAB[-1],np.pi,dotx4[-1],doty4[-1])

        # (x_d1,y_d1)=self.rotate(x_d1,y_d1, np.pi/2, x_c,y_c):
  
        x=np.concatenate([x1,x_d1,x2,x_d2,x3,x_d3,x4,x_d4,x5])
        y=np.concatenate([y1,y_d1,y2,y_d2,y3,y_d3,y4,y_d4,y5])

        v1=np.sqrt(dotx1**2+doty1**2)
        v2=np.sqrt(dotx2**2+doty2**2)
        v3=np.sqrt(dotx3**2+doty3**2)
        v4=np.sqrt(dotx4**2+doty4**2)
        v5=np.sqrt(dotx5**2+doty5**2)
        v=np.concatenate([v1,v_d1, v2,v_d2,v3,v_d3,v4,v_d4,v5])
        dotx = np.concatenate([dotx1,dotx_d1, dotx2,dotx_d2,dotx3,dotx_d3,dotx4,dotx_d4,dotx5])
        doty =np.concatenate([doty1, doty_d1,doty2,doty_d2,doty3,doty_d3,doty4,doty_d4,doty5])
        yaw = np.concatenate([y0D ,theta_d1, yDA ,theta_d2, yAB, theta_d3, yBC, theta_d4, yC0]) 
        w1=np.repeat(0,len(v1))
        w2=np.repeat(0,len(v2))
        w3=np.repeat(0,len(v3))
        w4=np.repeat(0,len(v4))
        w5=np.repeat(0,len(v5))
        w=np.concatenate([w1,w_d1, w2,w_d2,w3,w_d3,w4,w_d4,w5])

        # for i in range(0,len(yaw)): #aggiusto il theta desiderato per manternerlo nell'intervallo 0-2*pi
        #         if yaw[i]>=2*np.pi:
        #             appo=(yaw[i])//(2*np.pi)
        #             yaw[i]=yaw[i]-appo*2*np.pi
        t=[]
        
        t=np.concatenate([t1,t_d1,t2,t_d2,t3,t_d3,t4,t_d4,t5])
        
        i1= len(t1)
        i2= i1+len(t_d1)
        i3= i2+len(t2)
        i4= i3+len(t_d2)
        i5= i4+len(t3)
        i6= i5+len(t_d3)
        i7= i6+len(t4)
        i8= i7+len(t_d4)
        i9= i8+len(t5)
        ind=np.array([0,i1,i2,i3,i4,i5,i6,i7,i8,i9]) #lista che conterrà gli indici in corrispondenza della quale iniziano i diversi tratti (lineari e circolari)
        # rospy.loginfo("y0D[-1]= "+str(y0D[-1]))
        # rospy.loginfo("yDA[-1]= "+str(yDA[-1]))
        # rospy.loginfo("yAB[-1]= "+str(yAB[-1]))
        # rospy.loginfo("yBC[-1]= "+str(yBC[-1]))
        # rospy.loginfo("len x= "+str(len(x)))
        # rospy.loginfo("len y= "+str(len(y)))
        # rospy.loginfo("len dotx= "+str(len(dotx)))
        # rospy.loginfo("len doty= "+str(len(doty)))
        # rospy.loginfo("len yaw= "+str(len(yaw)))
        # rospy.loginfo("len t= "+str(len(t)))
        # rospy.loginfo("ind= "+str(ind))
    
        # exit()

        
        return [x,y, dotx, doty, t,yaw,v,w]

    def trajectory_4(self,puntoA, puntoB, puntoC, puntoD , lato, passo, vel,yaw_angle):

        modulo1 = math.sqrt((puntoA[0] - self.x0)**2 + (puntoA[1] - self.y0)**2)
        modulo2 = math.sqrt((self.x0 - puntoD[0])**2 + (self.y0 - puntoD[1])**2)

        #differenti numero di step per il calcolo dei tempi
        elementi = round(lato/(passo*vel))
        elementi1=round(abs(puntoA[1] - self.y0) / ( passo*vel) ) 
        elementi2 = round(abs(self.y0  - puntoD[1]) / ( passo*vel) ) 
        a = math.pi
        tempo_primo_pezzo = elementi1*passo
        tempi_primo_pezzo = np.linspace(0,tempo_primo_pezzo,elementi1)
        tempo_lato_quadrato = elementi*passo
        tempi_lato_quadrato = np.linspace(0,tempo_lato_quadrato,elementi)
        tempo_ultimo_pezzo = elementi2*passo
        tempi_ultimo_pezzo = np.linspace(0,tempo_ultimo_pezzo,elementi2)

        #calcolo dei tempi
        tfin1=elementi1*passo
        t1=np.linspace(0,tfin1,elementi1)
        tfin2=tfin1+elementi*passo
        t2=np.linspace(tfin1,tfin2,elementi)
        tfin3=tfin2+elementi*passo
        t3=np.linspace(tfin2,tfin3,elementi)
        tfin4=tfin3+elementi*passo
        t4=np.linspace(tfin3,tfin4,elementi)
        tfin5=tfin4+elementi2*passo
        t5=np.linspace(tfin4,tfin5,elementi2)
        t=np.concatenate([t1,t2,t3,t4,t5])

        # segmento x0y0_A
        x_c1 = self.x0
        y_c1 = self.y0

        y0A = np.repeat(yaw_angle,len(t1))

        x1 = np.repeat( x_c1 , len(t1))
        y1 = y_c1*np.ones(len(t1)) +  (vel*tempi_primo_pezzo) * (puntoA[1] - y_c1)/modulo1


        # segmento AB
        x_c2 = puntoA[0]
        y_c2 = puntoA[1]

        yaw_angle = yaw_angle+a/2
        yAB = np.repeat(yaw_angle,len(t2))

        x2 = lato/2*np.ones(len(t2)) - vel*tempi_lato_quadrato
        y2 = np.repeat( y_c2 , len(t2))

        # segmento BC
        x_c3 = puntoB[0]
        y_c3 = puntoB[1]

        yaw_angle = yaw_angle+a/2
        yBC = np.repeat(yaw_angle,len(t2))

        x3 = np.repeat( x_c3 , len(t3))
        y3 = lato/2*np.ones(len(t3)) - vel*tempi_lato_quadrato

        # segmento CD
        x_c4 = puntoC[0]
        y_c4 = puntoC[1]

        yaw_angle = yaw_angle+a/2
        yCD = np.repeat(yaw_angle,len(t2))

        x4 = -lato/2 *np.ones(len(t4)) + vel*tempi_lato_quadrato
        y4 = np.repeat( y_c4 , len(t4))

        #segmento D_x0y0

        x_c5 = puntoD[0]
        y_c5 = puntoD[1]

        yD0 = np.repeat(yaw_angle + a/2 , len(t5))

        x5 = np.repeat( x_c5 , len(t5))
        y5 = y_c5*np.ones(len(t5)) +  (vel*tempi_ultimo_pezzo) * (y_c1 - y_c5)/modulo2
        
        x=np.concatenate([x1,x2,x3,x4,x5])
        y=np.concatenate([y1,y2,y3,y4,y5])
        
        # calcolo delle derivate
        dotx1 =np.repeat(0,len(t1))
        doty1 =np.repeat(vel*(puntoA[1] - y_c1)/modulo1,len(t1))
        
        dotx2 =np.repeat(-vel,len(t2))
        doty2 =np.repeat(0,len(t2))
        
        dotx3 =np.repeat(0,len(t3))
        doty3 =np.repeat(-vel,len(t3))
        
        dotx4 =np.repeat(vel,len(t4))
        doty4 =np.repeat(0,len(t4))

        dotx5 =np.repeat(0,len(t5))
        doty5 =np.repeat(vel*(y_c1 - y_c5)/modulo2,len(t5))

        dotx = np.concatenate([dotx1, dotx2,dotx3,dotx4,dotx5])
        doty =np.concatenate([doty1, doty2,doty3,doty4,doty5])
        yaw = np.concatenate([y0A , yAB , yBC , yCD , yD0]) 
      
        
        return [x,y, dotx, doty, t,yaw]
    
def rotate(x,y, rad, x_c,y_c):

    qx = x_c + math.cos(rad) * (x - x_c) + math.sin(rad) * (y - y_c)
    qy = y_c + -math.sin(rad) * (x - x_c) + math.cos(rad) * (y - y_c)

    return qx, qy
