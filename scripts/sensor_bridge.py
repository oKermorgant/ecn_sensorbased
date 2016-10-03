#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
import rospy
from vrep_common.msg import ProximitySensorData, JointSetStateData
from vrep_common.srv import simRosGetObjectHandle, simRosGetObjectPose
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_matrix, euler_from_matrix
from pylab import array, dot

class Listener:
    def __init__(self, i):
        self.d = 0
        self.sub = rospy.Subscriber('vrep/us%i' % i, ProximitySensorData, self.us_callback)
        
    def us_callback(self, msg):
        self.d = msg.detectedPoint.z
         
        
if __name__ == "__main__":
    
    rospy.init_node('us_transform')
    
    us_range = range(16)
    
    # set transforms as ros param
    get_handle = rospy.ServiceProxy('/vrep/simRosGetObjectHandle', simRosGetObjectHandle)
    pioneer_ID = get_handle('Pioneer_p3dx').handle
    us_ID = [get_handle('Pioneer_p3dx_ultrasonicSensor%i' % (i+1)).handle for i in us_range]
    get_pose = rospy.ServiceProxy('/vrep/simRosGetObjectPose', simRosGetObjectPose)
    us_Pose = [get_pose(n, pioneer_ID).pose.pose for n in us_ID]
    
    iRi = array([[0,1,0],[0,0,1],[1,0,0]])
    for i, pose in enumerate(us_Pose):
        this_param = {'x': pose.position.x, 'y': pose.position.y}
        oRi = quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[:3,:3]
        this_param['theta'] = euler_from_matrix(dot(oRi, iRi))[2]
        rospy.set_param('sensors/ultrasonic%i' % (i+1), this_param)
    
    listeners = [Listener(i+1) for i in us_range]    
    
    msg = JointState()
    msg.name = ['sensor%i' % (i+1) for i in us_range]
    msg.position = [0 for i in us_range]
    us_pub = rospy.Publisher('vrep/us_dist', JointState, queue_size=10)
    
    # joint testing
    j_msg = JointSetStateData()
    j_msg.handles.data = [get_handle('Pioneer_p3dx_leftMotor').handle, get_handle('Pioneer_p3dx_rightMotor').handle]
    j_msg.setModes.data = [2,2]
    j_msg.values.data = [0.1,0.1]
    j_pub = rospy.Publisher('/vrep/joint_setpoint', JointSetStateData, queue_size=10)
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        
        # update measurements
        for i in us_range:
            msg.position[i] = listeners[i].d
        
        # Publish
        msg.header.stamp = rospy.Time.now()
        us_pub.publish(msg)
        
        j_pub.publish(j_msg)
        
        rate.sleep()
        
