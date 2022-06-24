#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from visualization_msgs.msg import MarkerArray
import time
import numpy as np

class SavePoses(object):
    def __init__(self):
        ##self._header_gz_model = 0.0
        self._pose_gz_model = Pose()
        self._headeramcl = 0.0
        self._poseamcl = Pose()
        self._pose_sub_gz_model = rospy.Subscriber('/gazebo/model_states', ModelStates , self.sub_callback_gz_model)
        self._pose_sub_amcl = rospy.Subscriber('/poses', MarkerArray , self.sub_callback_amcl)
        self.write_to_file()

    def sub_callback_gz_model(self, msg):
        
        self._pose_gz_model = msg.pose[-1]
        ##self._header_gz_model = msg.header.stamp.secs

    def sub_callback_amcl(self, msg):
        
        self._poseamcl = msg.markers[0].pose
        self._headeramcl = msg.markers[0].header.stamp.secs
       
       
    def write_to_file(self):

        while not rospy.is_shutdown():
            with open('poses.txt', 'a') as file:
                time.sleep(1)
                file.write("gz_model;stamp;"+str(self._headeramcl)+";x;"+str(-self._pose_gz_model.position.x)+";y;"+str(-self._pose_gz_model.position.y)+";z;"+str(-self._pose_gz_model.position.z)+"\n")                   
                rospy.loginfo("gz_model;stamp;"+str(self._headeramcl)+";x;"+str(-self._pose_gz_model.position.x)+";y;"+str(-self._pose_gz_model.position.y)+";z;"+str(-self._pose_gz_model.position.z))
                file.write("amcl;stamp;"+str(self._headeramcl)+";x;"+str(self._poseamcl.position.x)+";y;"+str(self._poseamcl.position.y)+";z;"+str(self._poseamcl.position.z)+"\n")                   
                rospy.loginfo("amcl;stamp;"+str(self._headeramcl)+";x;"+str(self._poseamcl.position.x)+";y;"+str(self._poseamcl.position.y)+";z;"+str(self._poseamcl.position.z))
        

if __name__ == "__main__":
    rospy.init_node('spot_recorder', log_level=rospy.INFO) 
    save_spots_object = SavePoses()
    #rospy.spin() # mantain the service open.
