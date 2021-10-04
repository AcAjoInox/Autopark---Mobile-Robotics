#!/usr/bin/env python3
#
# Author: Jonathan Sprinkle
# Copyright (c) 2015-2016 Arizona Board of Regents
# All rights reserved.
#
# Permission is hereby granted, without written agreement and without
# license or royalty fees, to use, copy, modify, and distribute this
# software and its documentation for any purpose, provided that the
# above copyright notice and the following two paragraphs appear in
# all copies of this software.
#
# IN NO EVENT SHALL THE ARIZONA BOARD OF REGENTS BE LIABLE TO ANY PARTY
# FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES
# ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN
# IF THE ARIZONA BOARD OF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.
#
# THE ARIZONA BOARD OF REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
# AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER
# IS ON AN "AS IS" BASIS, AND THE ARIZONA BOARD OF REGENTS HAS NO OBLIGATION
# TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

# This node converts cmd_vel inputs to the vehicle to the ROS topics that
# are exposed in Gazebo for moving the vehicle in simulation. Notably, the
# inputs to Gazebo are to joints on the wheel, so there is a multiplier of
# 2.8101 that is applied to the joint's velocity whenever we try to move
# so that the output in Gazebo will match the desired input velocity.

import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, Pose
import sys, getopt, math

class cmdvel2gazebo:

    def __init__(self,ns):
        self.ns = ns
        rospy.init_node('cmdvel2gazebo', anonymous=True)

        # the format(ns) looks for the namespace in the ros parameter server, I guess
        rospy.Subscriber('cmd_vel'.format(ns), Twist, self.callback)
        self.pub_steerL = rospy.Publisher('ferrari/front_left_steering_position_controller/command'.format(ns), Float64, queue_size=1)
        self.pub_steerR = rospy.Publisher('ferrari/front_right_steering_position_controller/command'.format(ns), Float64, queue_size=1)
        self.pub_rearL = rospy.Publisher('ferrari/joint1_velocity_controller/command'.format(ns), Float64, queue_size=1)
        self.pub_rearR = rospy.Publisher('ferrari/joint2_velocity_controller/command'.format(ns), Float64, queue_size=1)

        # initial velocity and tire angle are 0
        self.x = 0
        self.z = 0

        # TODO: get wheelbase and treadwidth from SDF or
        #       params database instead of hardcoded here

        # car Wheelbase (in m)
        # estrapolato dall urdf 
        self.L = 2.85

        # car Tread
        # estrapolato dall urdf
        self.d=2.2

        # how many seconds delay for the dead man's switch
        # TODO: set timeout from launch file or rosparam
        self.timeout=rospy.Duration.from_sec(0.2)
        self.lastMsg=rospy.Time.now()

        # we want maxsteer to be that of the "inside" tire, and since it is 0.6 in gazebo, we
        # set our ideal steering angle max to be less than that, based on geometry
        self.maxsteerInside=1.2
        # tan(maxsteerInside) = wheelbase/radius --> solve for max radius at this angle
        rMax = self.L/math.tan(self.maxsteerInside)
        # radius of inside tire is rMax, so radius of the ideal middle tire (rIdeal) is rMax+treadwidth/2
        rIdeal = rMax+(self.d/2.0)
        # tan(angle) = wheelbase/radius
        self.maxsteer=math.atan2(self.L,rIdeal)
        
        # the ideal max steering angle we can command is now set
        rospy.loginfo(rospy.get_caller_id() + " massimo angolo di sterzata:  {0}.".format(self.maxsteer))


    def callback(self,data):
        # 2.6101 is the gain factor in order to account for mechanical reduction of the tyres
        # non ho rapporti di trasmissione non unitari
        self.x = data.linear.x
        # constrain the ideal steering angle such that the ackermann steering is maxed out

        self.omega_maxsteer =abs(math.tan(self.maxsteer)* self.x/self.L)
        if (abs(self.omega_maxsteer) < abs(data.angular.z)):

            if (data.angular.z < 0 ):
                self.z = -self.omega_maxsteer
            else:
                self.z = self.omega_maxsteer
        else:
            self.z = data.angular.z
        
        #self.z = max(-self.omega_maxsteer,min(self.omega_maxsteer,data.angular.z))
        # rospy.loginfo("data.angular.z: " + str(data.angular.z))
        # rospy.loginfo("omega_maxsteer: " + str(self.omega_maxsteer))
        # rospy.loginfo("self.z: " + str(self.z))
        # if (abs(self.z) == abs(self.omega_maxsteer)):
        #     rospy.loginfo("la w è fuori dalla regione ammissibile ... w_max implementata : " + str(self.omega_maxsteer))
        # else:
        #     pass
        self.lastMsg = rospy.Time.now()

    def publish(self):
        # now that these values are published, we
        # reset the velocity, so that if we don't hear new
        # ones for the next timestep that we time out; note
        # that the tire angle will not change
        # NOTE: we only set self.x to be 0 after 200ms of timeout
        if rospy.Time.now() - self.lastMsg > self.timeout:
           # rospy.loginfo(rospy.get_caller_id() + " timed out waiting for new input, setting velocity to 0.")
            self.x = 0
            return

        if self.z != 0:
            D=self.d
            L=self.L
            # self.v is the linear *velocity*
            if math.atan2(self.L * self.z , self.x)>0:
                psi_ideale = math.pi-math.atan2(self.L * self.z , self.x)
            else:
                psi_ideale = -math.pi-math.atan2(self.L * self.z , self.x)
            rospy.loginfo("psi_ideale= "+ str(psi_ideale))
            # psi_ideale = math.atan2(self.L * self.z , self.x)
            r_ideal = L/math.fabs(math.tan(psi_ideale))
            
            rL = r_ideal-(math.copysign(1,self.z)*(D/2.0))
            rR = r_ideal+(math.copysign(1,self.z)*(D/2.0))
            msgRearR = Float64()
            # the right tire will go a little faster when we turn left (positive angle)
            # amount is proportional to the radius of the outside/ideal
            msgRearR.data = self.x*rR/r_ideal
            msgRearL = Float64()
            # the left tire will go a little slower when we turn left (positive angle)
            # amount is proportional to the radius of the inside/ideal
            msgRearL.data = self.x*rL/r_ideal

            self.pub_rearL.publish(msgRearL)
            self.pub_rearR.publish(msgRearR)

            msgSteerL = Float64()
            msgSteerR = Float64()
            # angolo sinistro di sterzata
            msgSteerL.data = math.atan2(L,rL)*math.copysign(1,self.z)
            self.pub_steerL.publish(msgSteerL)

            # angolo destro di sterzata
            msgSteerR.data = math.atan2(L,rR)*math.copysign(1,self.z)
            self.pub_steerR.publish(msgSteerR)
        else:
            # if we aren't turning, everything is easy!
            msgRear = Float64()
            msgRear.data = self.x
            self.pub_rearL.publish(msgRear)
            self.pub_rearR.publish(msgRear)

            msgSteer = Float64()
            msgSteer.data = self.z

            self.pub_steerL.publish(msgSteer)
            self.pub_steerR.publish(msgSteer)
def usage():
    print('cmdvel2gazebo -n catvehicle')


def main(argv):
    # we eventually get the ns (namespace) from the ROS parameter server for this node
    ns=''
    node = cmdvel2gazebo(ns)
    rate = rospy.Rate(100) # run at 100Hz
    while not rospy.is_shutdown():
        node.publish()
        rate.sleep()

if __name__ == '__main__':
    main(sys.argv[1:])


