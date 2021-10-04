#!/usr/bin/env python3
import rospy
import rospkg
import yaml
import sys
import ruamel.yaml
import os
import random
from geometry_msgs.msg import Pose
from get_model_gazebo_pose import GazeboModel
from rospy.core import xmlrpcapi
from tf.transformations import euler_from_quaternion
import numpy as np


"""
string[] name
geometry_msgs/Pose[] pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w

"""

class ModelPoseStore():
    def __init__(self, model_to_track_list):

        # This are the models that we will generate information about.
        self.gz_model_obj = GazeboModel(model_to_track_list)
        

    def seq(self,*l):
        """
        funzione creata per gestire i formati di store del file yaml come flow_style_sequence
        """
        s = ruamel.yaml.comments.CommentedSeq(l)
        s.fa.set_flow_style()
        return s  

    def get_pose_of_model(self, robot_name):
        """
        Retrieves the position of an object from the world
        """
        pose_now = self.gz_model_obj.get_model_pose(robot_name)
        
        return pose_now
        
    def store_model_poses(self, model_name, file_to_store="poses.yaml" ):
        """
        We store in the Given File Name the poses of the given model, whihc has to be inside the init model list
        inside a Yaml file
        """
        
        with open(file_to_store, 'w') as outfile:

                #inizializzo il dict vuoto con chiave models e value lista vuota

                dict_in = dict(models = [])

                now_dict = dict_in

                el_listaURDF = self.get_square_pose()
                now_dict["models"].append(el_listaURDF)   #sostituire con now_dict
                rospy.loginfo("now_dict URDF inizializzato")

                # aggiorno il dizionario con tutti gli elementi presenti nella lista
                for robot_name in model_name :

                    now_pose = self.get_pose_of_model(robot_name)
                    el_lista = self.reformat_pose_to_dict(now_pose) #ritorna un dizionario che sarà elemento della lista
                    now_dict["models"].append(el_lista)              #eseguo l'append nella lista di models
                    
                rospy.loginfo("now_dict sdf inizializzati")
                
                # una volta inizializzati i models sdf posso scrivere nel file .yaml anche la posizione in cui deve essere spawnata l urdf
                # devo creare l'ultimo elemento dict della lista e fare l'append alla lista che andrà in scrittura

                """
                {"name" : "ferrari" , 
                    "type" : "urdf" , 
                    "package" : "autopark" , 
                    "pose ":self.seq(lista_parametri) 
                    }
                                        
                """
                

                # Scrittura su file .yaml
                
                yaml = ruamel.yaml.YAML()
                yaml.indent(mapping=3, sequence=4, offset=2)    #comando per indentare correttamente come nel file spawn_obj
                yaml.dump(now_dict, sys.stdout)                 #comando per stampare nella shell per visualizzare il corretto formato
                
                yaml.dump(now_dict, outfile)                    #comando che stampa all'interno del file .yaml

                rospy.loginfo("numero auto sdf spawnate : " + str(len(now_dict["models"])-1))
                
        
    def get_square_pose(self):

        # se cambia la  dimensione dell lato del quadrato cambiare manualmente il valore .. questo verrà modificato poi facendo l'import di lato 
        # dal trajectory_generation.py

        lato = 24.28*2
        delta = 0.1
        num_elementi = lato/delta
        # punti=np.linspace(-lato/2,lato/2,num_elementi)
        punti=np.linspace(-12,12,num_elementi) # puoi cambiare manualmente i limiti di spawn sulla strada (per evitare gli angoli)
        
        # una delle 2 coordinate x o y sarà fissa e assumerà valore lato/2 o -lato/2

        coordinate = ['x', 'y']
        val_fissi = [-lato/2 , lato/2]
        c = random.choice(coordinate)
        v = random.choice(val_fissi)
        dict_coordinata = {c:v}

        rospy.loginfo("coordinata fissa " + str(dict_coordinata.keys()) + " = " + str(dict_coordinata.get(c)))

        # ora inizializzo l'altra coordinata scegliendo un random tra -26 e 26
        if (c == 'x'):
            x = dict_coordinata[c]
            y = round(random.choice(punti.tolist()),2)
        else:
            x = round(random.choice(punti.tolist()),2)
            y = dict_coordinata[c]

        rospy.loginfo("spawn URDF in x: " + str(x) + " y: " + str(y))
        # a seconda dello spown deve essere fissato anche l'orientamento
        # ricordiamo che la vettura nel nostro caso si sviluppa su y  pertanto per orientarla su -x ho yaw += 1.57
        #
        # B----------A
        # |          |
        # |          |
        # C----------D
        #
        # square description
        ###########################################################
        a = 3.14
        if(x == lato/2) and (y != lato/2):
            #sono sul segmento DA
            yaw_angle = a 
        if(x == -lato/2) and (y != -lato/2):
            #sono sul segmento BC
            yaw_angle = 0
        if(y == lato/2) and (x != -lato/2):
            #sono sul segmento AB
            yaw_angle = -a/2
        if(y == -lato/2) and (x != lato/2):
            #sono sul segmento CD
            yaw_angle = a/2

        roll_angle = 0
        pitch_angle = 0
        z = 0.3 # offset asfalto

        lista_parametri = [x ,y ,z ,roll_angle ,pitch_angle ,yaw_angle ]
        #preparo il dict da aggiungere al file .yaml
        elemento_lista = {'name' : 'ferrari',
                       'type': 'urdf',
                       'package': 'autopark',
                       'pose': self.seq( x , y , z , roll_angle , pitch_angle , yaw_angle)}

        return elemento_lista
        
 
        
    def reformat_pose_to_dict(self, now_pose):
        """
        Converts Pose to dict
        """
        # now_pose è un dict in particolare { pose : [ {position : [{x : value , y:value , z:value} ] } , {orientation : [] } }
        # devo convertire i quaternioni in amgoli di eulero...estrarre i quaternioni da pose_now e convertirli in angoli RPY

        lato_corto_2 = 1.65 #1.45     # offset parcheggio
        
        #correggo gli offset x centrare le macchine nei parcheggi

        if abs(round(now_pose.position.x,2)) == 22.45:
            if now_pose.position.x < 0 :
                now_pose.position.x+=lato_corto_2
                now_pose.position.y-=0.4
            else :
                now_pose.position.x-=lato_corto_2
                now_pose.position.y+=0.4
            
        if abs(round(now_pose.position.y,2)) == 22.45:
            if now_pose.position.y < 0 :
                now_pose.position.y+=lato_corto_2
                now_pose.position.x+=0.4
            else :
                now_pose.position.y-=lato_corto_2
                now_pose.position.x-=0.4

        # correggo la z per renderla uguale all'asfalto che viene spownata nel mondo

        offset_asfalto = 0.3

        x = now_pose.position.x
        y = now_pose.position.y
        z = now_pose.position.z + offset_asfalto

        q1 = now_pose.orientation.x
        q2 = now_pose.orientation.y
        q3 = now_pose.orientation.z
        q4 = now_pose.orientation.w


        # converto i quaternioni in angoli di rulero RPY in radianti
        orientation_list = [q1,q2,q3,q4]

        euler = euler_from_quaternion( orientation_list )
        roll = euler[0]
        pitch = euler[1]
        yaw = round(euler[2],2) + np.pi


        # creo la lista dei parametri che mi servono nel campo pose:[] del file .yaml

        lista_parametri = [x ,y ,z ,roll ,pitch ,yaw ]

        # creo un dict con tutti i campi di cui ho bisogno nel file .yaml
        # settare le chiavi 'name' , ' type ' , 'package' , ' pose ' secondo le proprie necessità
        # i due stili sono equivalenti : usare quello preferito
        """
        {"name" : "park1" , 
            "type" : "sdf" , 
            "package" : "object_spawner" , 
            "pose ":self.seq(lista_parametri) 
            }
                                
        """
        lista_veicoli = ['macchina','pickup','ferrari','prius_hybrid','car_lexus','car_polo','car_volvo','car_golf']
        num_veicoli = 1

        #modificare qui implementando una funzione randomica se si vogliono piu veicoli casuali spawnati
        elemento_lista = {'name' : lista_veicoli[3],
                       'type': 'sdf',
                       'package': 'object_spawner',
                       'pose': self.seq( x , y , z , roll , pitch , yaw)}
        #"""
        # elemento_lista = {'name' : 'ferrari',
        #                'type': 'urdf',
        #                'package': 'autopark',
        #                'pose': self.seq( x , y , z , roll , pitch , yaw)}

        return elemento_lista
        
    
    

if __name__ == '__main__':
    rospy.init_node('store_model_poses_node', anonymous=True, log_level=rospy.DEBUG)
    

    num_parking = 28
    model_to_track_list = []

    # i nomi dei modelli sono settati nel file .world del pkg autopark
    # creo la lista con tutti i nomi dei parcheggi presenti nel file .world

    for i in range(num_parking) :
        model_to_track_list.append("parking"+ str(i+1))             
    
    rospy.loginfo(str(model_to_track_list))

    
    #randomizzare l'accesso agli elementi della lista

    number_to_select = random.randint(1,num_parking)
    model_to_track_list = random.sample(model_to_track_list, number_to_select)

    model_pose_store_obj = ModelPoseStore(model_to_track_list)
    
    rospack = rospkg.RosPack()
    # scegliere il pacchetto e la directory in cui effettuare il salvataggio del file .yaml
    path_to_package = rospack.get_path('object_spawner')
    pose_files_dir = os.path.join(path_to_package, "config")
    
    
    if not os.path.exists(pose_files_dir):
        os.makedirs(pose_files_dir)
    
    
    pose_file_name = "parcheggi"+".yaml"
    pose_file_path = os.path.join(pose_files_dir, pose_file_name)
    
    model_pose_store_obj.store_model_poses( model_name=model_to_track_list, file_to_store=pose_file_path )








   