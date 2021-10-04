#! /usr/bin/env python3

import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from yaml import load
from tf.transformations import quaternion_from_euler
import math 
import random
import sys
import numpy as np

class Model(object):
    def __init__(self, **entries): 
        self.__dict__.update(entries)

    def __repr__(self):
       return '{}({!r}, {!r}, {!r})'.format(
           self.__class__.__name__,
           self.name,self.type,self.package)

    def __unicode__(self):
        return u'name: %s, type: %s, package: %s' % (self.name,self.type,self.package)

    def __str__(self):
        return unicode(self).encode('utf-8')

def rename_duplicates( old ):
    """
    Append count numbers to duplicate names in a list
    So new list only contains unique names
    """
    seen = {}
    for x in old:
        if x in seen:
            seen[x] += 1
            yield "%s_%d" % (x, seen[x])
        else:
            seen[x] = 0
            yield x

def parse_yaml(package_name,yaml_relative_path):
    """ Parse a yaml into a dict of objects containing all data to spawn models
        Args:
        name of the package (string, refers to package where yaml file is located)
        name and path of the yaml file (string, relative path + complete name incl. file extension)
        Returns: dictionary with model objects
    """
    complete_path = rospkg.RosPack().get_path(package_name)+yaml_relative_path
    f = open(complete_path, 'r')
    # populate dictionary that equals to the yaml file tree data
    yaml_dict = load(f)

    # create a list of the names of all models parsed
    modelNames = [yaml_dict['models'][k]['name'] for k in range(len(yaml_dict['models']))]
    # create new list with count numbers on all names that were duplicated
    modelNamesUnique = list(rename_duplicates(modelNames))
    
    # rospy.loginfo("List of model names: %s" % modelNamesUnique)
    rospy.logdebug("Total number of models: ", len(yaml_dict['models']))

    # create a dict of Model objects where each key is the name of the model
    model_dict = {name: Model() for name in modelNamesUnique}
    # create list containing all nested dictionaries that hold data about each model   
    list_of_dict = [x for x in yaml_dict['models']]
    # parse YAML dictionary entries to Model class object attributes
    for idx, name in enumerate(modelNamesUnique):
        args = list_of_dict[idx]
        model_dict[name] = Model(**args)

    # add a unique model name that can be used to spawn an model in simulation
    count = 0
    for dict_key, mod_obj in model_dict.items():
        mod_obj.unique_name = modelNamesUnique[count] # add attribute 'unique_name'
        count += 1
    for key in model_dict.keys():
        if round(model_dict[key].pose[5],2)==6.28:
            model_dict[key].pose[5]=0.0
        elif round(model_dict[key].pose[5],2)==4.71:
            model_dict[key].pose[5]=-1.57
    return model_dict


def findpark(q_i,parcheggi,flag_occ):  # flag_occ è un flag che indica se il parcheggio si trova prima(2), in corrispondenza(1) o dopo(0)
    toltheta=0.2
    pos=[]
    dim_park=4.9
    
    if -np.pi-toltheta<=round(q_i[2])<=-np.pi+toltheta:  #correzione per theta= -3.14 (il nostro riferimento è 3.14)
        q_i[2]=-q_i[2] 
    q_i[1]=round(q_i[1]+1.4*math.cos(q_i[2]+np.pi),4) ##spostiamo il punto di riferimento dal centro della macchina al centro dell' asse posteriore
    q_i[0]=round(q_i[0]-1.4*math.sin(q_i[2]+np.pi),4) ##spostiamo il punto di riferimento dal centro della macchina al centro dell' asse posteriore
        

    for key in parcheggi.keys():
        # rospy.loginfo('theta')
        # rospy.loginfo(str(round(parcheggi[key].pose[5],2)-toltheta)+'<'+str(q_i[2])+'<'+str(round(parcheggi[key].pose[5],2)+toltheta))
        if (round(parcheggi[key].pose[5],2)-toltheta<=q_i[2]<=round(parcheggi[key].pose[5],2)+toltheta):
            if np.pi-toltheta<=abs(round(parcheggi[key].pose[5],2))<=np.pi+toltheta or -toltheta<=round(parcheggi[key].pose[5],2)<=toltheta:
                appo_y=math.cos(approssimazione(q_i[2],toltheta)) #math.cos(q_i[2]) # la funzione aggiunge + o -5 in base al lato in cui ti trovi, quindi appo (x o y) fornisce il segno
                appo_x=0
                tolx=4
                toly=3 
            else:
                tolx= 3 
                toly=4
                appo_y=0
                appo_x=math.sin(approssimazione(q_i[2],toltheta)) #math.sin(q_i[2]) ##########################################
            # rospy.loginfo('su x')
            # rospy.loginfo(str(round(parcheggi[key].pose[0],2)-tolx)+'<'+str(q_i[0])+'<'+str(round(parcheggi[key].pose[0],2)+tolx)+' valore x='+str(round(parcheggi[key].pose[0],2)))
            if(round(parcheggi[key].pose[0],2)-tolx<=q_i[0]<=round(parcheggi[key].pose[0],2)+tolx):
                # rospy.loginfo('su y')
                # rospy.loginfo(str(round(parcheggi[key].pose[1],2)-toly)+'<'+str(q_i[1])+'<'+str(round(parcheggi[key].pose[1],2)+toly)+' valore y='+str(round(parcheggi[key].pose[1],2)))
                if(round(parcheggi[key].pose[1],2)-toly<=q_i[1]<=round(parcheggi[key].pose[1],2)+toly):#compresa intervvalo
                    if pos!=[]:
                        if flag_occ==2  and round(pos[1],2)>round(parcheggi[key].pose[1],2):
                            pos=parcheggi[key].pose
                        elif flag_occ==1  and round(pos[1],2)<round(parcheggi[key].pose[1],2):
                            pos=parcheggi[key].pose
                    else:
                        pos=parcheggi[key].pose
                        
                else: 
                    pass
            else: 
                pass
        else: 
            pass
    if pos==[]:
        rospy.loginfo('park not found')
        return np.array([0,0,0])
    else:

    # rospy.loginfo('y attuale:')
    # rospy.loginfo(np.array([pos[0],pos[1],pos[5]]))
    # rospy.loginfo('appo_x='+str(appo_x))
    # rospy.loginfo('appo_y='+str(appo_y))
    # rospy.loginfo('y successivo:')
    # rospy.loginfo(np.array([pos[0]+dim_park*appo_x,pos[1]+dim_park*appo_y,pos[5]]))

        # rospy.loginfo("coordinate centro del parcheggio corrispondente (x,y,theta)= "+str(np.array([pos[0],pos[1],pos[5]])))
        if flag_occ==0:
            # rospy.loginfo("coordinate centro del parcheggio libero (x,y,theta) (successivo)= "+str(np.array([pos[0]+dim_park*appo_x,pos[1]-dim_park*appo_y,pos[5]])))
            return np.array([pos[0]+dim_park*appo_x,pos[1]-dim_park*appo_y,pos[5]]) #vettore con x,y,theta del parcheggio voluto
        elif flag_occ==1:
            # rospy.loginfo("coordinate centro del parcheggio libero (x,y,theta) (in corrispondenza)= "+str(np.array([pos[0],pos[1],pos[5]])))
            return np.array([pos[0],pos[1],pos[5]]) #vettore con x,y,theta del parcheggio voluto
        else: 
            # rospy.loginfo("coordinate centro del parcheggio libero (x,y,theta) (precedente)= "+str(np.array([pos[0]-dim_park*appo_x,pos[1]+dim_park*appo_y,pos[5]])))
            return np.array([pos[0]-dim_park*appo_x,pos[1]+dim_park*appo_y,pos[5]]) #vettore con x,y,theta del parcheggio voluto
    # return np.array([pos[0],pos[1],pos[5]]) #vettore con x,y,theta del parcheggio voluto

    
def approssimazione(theta,toltheta):
    if  -toltheta<=theta<=toltheta:
        theta=0
    elif np.pi-toltheta<=theta<=toltheta+np.pi:
        theta=np.pi
    elif np.pi/2-toltheta<=theta<=toltheta+np.pi/2:
        theta=np.pi/2
    else:
        theta=-np.pi/2
    return theta


if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('object_spawner_node', log_level=rospy.INFO)

    ###### usage example
    
    # retrieve node configuration variables from param server
    yaml_package_name = rospy.get_param('~yaml_package_name', 'object_spawner')
    yaml_relative_path = rospy.get_param('~yaml_relative_path', '/config/parcheggi2.yaml')
    random_order = rospy.get_param('~random_order', 'false')
    time_interval = rospy.get_param('~time_interval', 0.1)
    # parse yaml file to dictionary of Model objects
    m = parse_yaml(yaml_package_name,yaml_relative_path) # create dict called 'm'
    # q_i=[-23.0, 13.7, 0.13]
    # a=findpark(q_i,m)
    # rospy.loginfo(a)
    # exit()


   