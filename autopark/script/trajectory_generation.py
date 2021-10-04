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
            print("Lato 1")
            
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
            print("Lato 2")

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
            print("Lato 3")

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
            print("Lato 4")

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
        
        # plot = plt.figure(2)
        # plt.plot(t,vel)
        # plt.title('velocità')
        # plt.xlabel('t')
        # plt.ylabel('v')
        # plt.axis('equal')

        # plot = plt.figure(3)
        # plt.plot(t,w)
        # plt.title('vel angolare')
        # plt.xlabel('t')
        # plt.ylabel('w')
        # plt.axis('equal')

        # plot = plt.figure(4)
        # plt.plot(t,tetad)
        # plt.title('tetad')
        # plt.xlabel('t')
        # plt.ylabel('tetad')
        # plt.axis('equal')

        # plot = plt.figure(5)
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

   