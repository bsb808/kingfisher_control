#!/usr/bin/env python
'''
This utility node is meant to convert from a message (user-defined) 
to a Float32 message for use with the Pid package with ROS.
'''
# Python
import sys
from math import pi

# ROS
import rospy
from dynamic_reconfigure.server import Server
#from YawDynamic.cfg import YawDynamicConfig #TutorialsConfig
#from YawDynamicConfig import YawDynamicConfig #TutorialsConfig
from kingfisher_control.cfg import YawDynamicConfig

from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from kingfisher_msgs.msg import Drive
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from gazebo_msgs.msg import ModelStates

# BSB
import pypid


class Node():
    def __init__(self):
        Kp=0.5
        Ki=0.0
        Kd=0.0
        self.pid = pypid.Pid(Kp,Ki,Kd)
        self.pid.set_setpoint(0.0)
        #self.pid.set_inputisangle(True,pi)
        self.drivemsg = None
        self.publisher = None
        self.lasttime = None
        
    def set_setpoint(self,msg):
        self.pid.set_setpoint(msg.data)

    def callback(self,msg):
        # Got a RPY
        yawrate = msg.twist[1].angular.z
        now = rospy.get_time()
        if self.lasttime is None:
            self.lasttime = now
            return
        dt = now-self.lasttime
        self.lasttime = now
        #print("dt: %.6f"%dt)
        out = self.pid.execute(dt,yawrate)
        torque = out[0]
        self.drivemsg.left=-1*torque
        self.drivemsg.right=torque
        self.publisher.publish(self.drivemsg)
    def dynamic_callback(self,config,level):
        #rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\ 
        #  {str_param}, {bool_param}, {size}""".format(**config))
        rospy.loginfo("Reconfigure request...")
        #print config.keys()
        #print config['Kp']
        self.pid.Kp = config['Kp']
        self.pid.Ki = config['Ki']
        self.pid.Kd = config['Kd']
        return config
        



if __name__ == '__main__':
    
    rospy.init_node('kingfisher_yawrate_pid', anonymous=True)
    
    # Initiate node object
    node=Node()
    
    # Setup outbound message
    node.drivemsg = Drive()

    in_topic = "gazebo/model_states"
    out_topic = "cmd_drive"
    # Setup publisher
    rospy.loginfo("Subscribing to %s"%
                  (in_topic))
    rospy.loginfo("Publishing to %s"%
                  (out_topic))
    node.publisher = rospy.Publisher(out_topic,Drive,queue_size=10)

    # Setup subscribers
    rospy.Subscriber(in_topic,ModelStates,node.callback)
    rospy.Subscriber("set_setpoint",Float64,node.set_setpoint)
    
    # Dynamic configure
    srv = Server(YawDynamicConfig, node.dynamic_callback)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
