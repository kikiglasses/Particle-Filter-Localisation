from geometry_msgs.msg import Pose, PoseArray, Quaternion
from . pf_base import PFLocaliserBase
import math
import rospy

from . util import rotateQuaternion, getHeading
from random import random

from time import time

print(PoseArray)
class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        self.n = 100 #Number of particles
        
        # ----- Set motion model parameters
            #Initial placement noise
        self.INIT_ROTATION_NOISE = 1
        self.INIT_TRANSLATION_NOISE = 1
        self.INIT_DRIFT_NOISE = 1
            #Update step noise
        self.UPDA_ROTATION_NOISE = 1
        self.UPDA_TRANSLATION_NOISE = 1
        self.UPDA_DRIFT_NOISE = 1
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
            part.position.x = initialpose.position.x + random.gauss(0, 1)*self.INIT_TRANSLATION_NOISE  # Check what noise should be later
            part.position.y = initialpose.position.y + random.gauss(0, 1)*self.UPDA_DRIFT_NOISE
            part.orientation.z = rotateQuaternion(initialpose.pose.orientation, random.gauss(0, 1)*self.INIT_ROTATION_NOISE)
            self.particlecloud.append(part)

            #print(self.particle_cloud)
        return self.particlecloud
 
    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
        weights = [0]
        for part in self.particlecloud:
            weights.append(self.sensor_model.get_weight(scan, part) + weights[-1])

        new_particlecloud = PoseArray()

        for i in range(math.round(self.n*1)):       # Change for more random particles
            r = random.random() * weights[-1]
            j = 0
            while(r >= weights[j]):
                j+=1
            
            part = Pose()
          
            part.position.x = self.particlecloud[j].position.x + random.gauss(0, 1)*self.UPDA_TRANSLATION_NOISE  # Check what noise should be later
            part.position.y = self.particlecloud[j].position.y + random.gauss(0, 1)*self.UPDA_DRIFT_NOISE
            part.orientation.z = rotateQuaternion(self.particlecloud[j].pose.orientation, random.gauss(0, 1)*self.UPDA_ROTATION_NOISE)

            new_particlecloud.append(self.particlecloud[j])

        #for i in range(math.round(self.n*0.1)):
            # random particles for kidnapped robot
        
        self.particlecloud = new_particlecloud
        

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
        pass
