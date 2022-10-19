from geometry_msgs.msg import Pose, PoseArray, Quaternion
from . pf_base import PFLocaliserBase
import math
import rospy
from numpy import searchsorted

from . util import rotateQuaternion, getHeading, multiply_quaternions
from random import random
import random

import numpy as np
#from sklearn.cluster import DBSCAN

from time import time

PI_OVER_TWO = math.pi/2

#print(PoseArray)
class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        self.n = 300     # Number of particles
        
        self.best_pose = Pose()  # robot best pose

        # ----- Set motion model parameters
        '''SOMETHING TO TALK ABOUT'''

        # constants used for adaptive MCL
        self.b = 0.9                    # exponential scaling factor
        self.b20 = pow(self.b, 20)      # b^20

            #Initial placement noise
        self.INIT_ROTATION_NOISE = PI_OVER_TWO/3        # TALK ABOUT ASSUMPTIONS
        self.INIT_TRANSLATION_NOISE = 0.05              # .....
        self.INIT_DRIFT_NOISE = 0.05                    # .....
            #Update step noise   #Given in super.
        self.UPDA_ROTATION_NOISE = PI_OVER_TWO/3
        self.UPDA_TRANSLATION_NOISE = 0.05
        self.UPDA_DRIFT_NOISE = 0.05
            #Kidnapped random noise
        self.RAND_TRANSLATION_NOISE = 1
        self.RAND_DRIFT_NOISE = 1
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
 
    def kidnapped_particles(self, max):
        if max >= 20:
            return 0
        num = pow(self.b,max) - self.b20
        num = 20*num/(1 - self.b20)
        return round(num)
    
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
            #print(cumul_weights) # for debugging purposes
            if x > max[0]:
                max = (x,i)
            i += 1
        print(max[0])

        new_particlecloud = PoseArray()


            ### resampled particles
        for i in range(self.n):       # Change for more random particles
            r = random.random() * cumul_weights[-1]
            j = searchsorted(cumul_weights, r) - 1            # Binary Search is something to talk about
            
            part = Pose()

            part.position.x = self.particlecloud.poses[j].position.x + random.gauss(0, 1)*self.UPDA_TRANSLATION_NOISE
            part.position.y = self.particlecloud.poses[j].position.y + random.gauss(0, 1)*self.UPDA_DRIFT_NOISE
            part.orientation = rotateQuaternion(self.particlecloud.poses[j].orientation, random.gauss(0, 1)*self.UPDA_ROTATION_NOISE)

            new_particlecloud.poses.append(part)



            ### random particles for kidnapped robot problem
        for i in range(self.kidnapped_particles(max[0])):
            j = random.randint(0, self.n-1)
            part = Pose()

            part.position.x = self.particlecloud.poses[j].position.x + random.gauss(0, 10)*self.RAND_TRANSLATION_NOISE      #Takes random particle and adds gaussian noise with large s.d.
            part.position.y = self.particlecloud.poses[j].position.y + random.gauss(0, 10)*self.RAND_DRIFT_NOISE
            part.orientation.z = math.pi*(random.random()*2 - 1)                                                             #Totally random yaw

            new_particlecloud.poses.append(part)
        
        self.particlecloud = new_particlecloud


        max = (0,0)
        i = 0
        for part in self.particlecloud.poses:
            x = self.sensor_model.get_weight(scan, part)
            if x > max[0]:
                max = (x,i)
            i += 1
        self.best_pose = self.particlecloud.poses[max[1]]
        

    def avg_pose(self, arr):
        avgX = 0
        avgY = 0
        avgZ = 0
        avgQ = (0,0,0,0)
        for part in arr:
            avgX = part.position.x
            avgY = part.position.y
            avgZ = part.position.z
            avgQ = (part.orientation.getX(),
                    part.orientation.getY(),
                    part.orientation.getZ(),
                    part.orientation.getW())
            i += 1
        avgX = avgX / i
        avgY = avgY / i
        avgZ = avgZ / i
        avgQ = (avgQ[0]/i, avgQ[1]/i, avgQ[2]/i, avgQ[3]/i)

        avgPose = Pose()
        avgPose.x = avgX
        avgPose.y = avgY
        avgPose.z = avgZ
        avgPose.orientation = Quaternion(avgQ[0], avgQ[1], avgQ[2], avgQ[3])

        return avgPose

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
        '''
        # TODO work out threshold
        DISSIMILARITY_THRESHOLD = 1

        clusters = [[self.particlecloud.poses[0]]]
        i = 1
        for part in self.particlecloud.poses[1:]:
            closestCluster = (1000, clusters[0])
            for cluster in clusters:
                diff = part.inverseTimes(self.particlecloud.poses[i], self.avg_pose(cluster))
                if diff < closestCluster[0] :
                    closestCluster = (diff, cluster)
            if (closestCluster[0] > DISSIMILARITY_THRESHOLD):
                clusters.append([part])
        
        best_cluster = clusters[0]
        for cluster in clusters:
            if len(cluster) >= len(best_cluster):
                best_cluster = cluster

        self.best_pose = self.avg_pose(best_cluster)
        '''
        return self.best_pose
