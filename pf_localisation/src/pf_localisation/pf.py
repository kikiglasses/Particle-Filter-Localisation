from ossaudiodev import SOUND_MIXER_ALTPCM
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from . pf_base import PFLocaliserBase
import math
import rospy
from numpy import searchsorted

from . util import rotateQuaternion, getHeading
from random import random
import random

import numpy as np
#from sklearn.cluster import DBSCAN

from time import time

PI_OVER_TWO = math.pi/2

print(PoseArray)
class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        self.n = 50     # Number of particles
        
        self.best_pose = Pose()  # robot best pose

        # ----- Set motion model parameters
        '''SOMETHING TO TALK ABOUT'''
            #Initial placement noise
        self.INIT_ROTATION_NOISE = PI_OVER_TWO/6        # 99.7% of the time, the robot should be at most 90 degrees off (assumption)
        self.INIT_TRANSLATION_NOISE = 0.02              #
        self.INIT_DRIFT_NOISE = 0.02
            #Update step noise   #Given in super.
        self.UPDA_ROTATION_NOISE = PI_OVER_TWO/6
        self.UPDA_TRANSLATION_NOISE = 0.02
        self.UPDA_DRIFT_NOISE = 0.02
        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict
        
       
    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise

        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
        

        #self.particlecloud = PoseArray()
        #pose_array.header.frame_id = "map" ????

        for i in range(self.n):
            part = Pose()
            part.position.x = initialpose.pose.pose.position.x + random.gauss(0, 1)*self.INIT_TRANSLATION_NOISE  # Check what noise should be later
            part.position.y = initialpose.pose.pose.position.y + random.gauss(0, 1)*self.INIT_DRIFT_NOISE
            part.orientation = rotateQuaternion(initialpose.pose.pose.orientation, random.gauss(0, 1)*self.INIT_ROTATION_NOISE)
            self.particlecloud.poses.append(part)

            #print(self.particle_cloud)
        return self.particlecloud
 
    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
       
        '''SOMETHING TO TALK ABOUT'''
        cumul_weights = [0]
        max = (0,0)
        i = 0
        for part in self.particlecloud.poses:
            x = self.sensor_model.get_weight(scan, part)
            cumul_weights.append(x + cumul_weights[-1])
            print(cumul_weights) # for debugging purposes
            if x > max[0]:
                max = (x,i)
            i += 1

        new_particlecloud = PoseArray()

        for i in range(round(self.n)):       # Change for more random particles
            r = random.random() * cumul_weights[-1]
            j = searchsorted(cumul_weights, r) - 1            # Binary Search is something to talk about
            
            part = Pose()

            part.position.x = self.particlecloud.poses[j].position.x + random.gauss(0, 1)*self.UPDA_TRANSLATION_NOISE  # Check what noise should be later
            part.position.y = self.particlecloud.poses[j].position.y + random.gauss(0, 1)*self.UPDA_DRIFT_NOISE
            part.orientation = rotateQuaternion(self.particlecloud.poses[j].orientation, random.gauss(0, 1)*self.UPDA_ROTATION_NOISE)

            new_particlecloud.poses.append(part)

        #for i in range(math.round(self.n*0.1)):
            # random particles for kidnapped robot
        
        self.particlecloud = new_particlecloud


        max = (0,0)
        i = 0
        for part in self.particlecloud.poses:
            x = self.sensor_model.get_weight(scan, part)
            if x > max[0]:
                max = (x,i)
            i += 1
        self.best_pose = self.particlecloud.poses[max[1]]
        

    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """
        # TODO add clustering


        # Calculate the average position and heading of particles
        avgX = 0
        avgY = 0
        avgZ = 0
        avgO = (0,0,0,0)
        i = 0
        # Sum all the poses in the particle cloud 
        # TODO change self.particlecloud to be new clustered particle cloud
        for part in self.particlecloud.poses:
            avgX = self.particlecloud.poses[i].position.x
            avgY = self.particlecloud.poses[i].position.y
            avgZ = self.particlecloud.poses[i].position.z
            avgO = (self.particlecloud.poses[i].orientation.getX(),
                    self.particlecloud.poses[i].orientation.getY(),
                    self.particlecloud.poses[i].orientation.getZ(),
                    self.particlecloud.poses[i].orientation.getW())
            i += 1
        avgX = avgX / i
        avgY = avgY / i
        avgZ = avgZ / i
        avgO = (avgO[0]/i, avgO[1]/i, avgO[2]/i, avgO[3]/i)

        self.best_pose.position.x = avgX
        self.best_pose.position.y = avgY
        self.best_pose.position.z = avgZ
        self.best_pose.orientation = Quaternion(avgO[0], avgO[1], avgO[2], avgO[3])
        '''
        cloud = np.array(self.particlecloud.poses)
        clustering = DBSCAN(eps = 0.5, min_samples = 2).fit(cloud)
        labels = clustering.labels_
        best = 0
        for i in labels:
            if 
        '''
        return self.best_pose
