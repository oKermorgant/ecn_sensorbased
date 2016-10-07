#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
import rospy
from vrep_common.srv import simRosStopSimulation
from vrep_common.msg import JointSetStateData

class Checker:
    def __init__(self):
        self.life = 0
        self.sub = rospy.Subscriber('vrep/joint_setpoint', JointSetStateData, self.callBack)
        
    def callBack(self, msg):
        self.life += .15
         
if __name__ == "__main__":
    
    rospy.init_node('check_vitals')
    
    stopSim = rospy.ServiceProxy('/vrep/simRosStopSimulation', simRosStopSimulation)
    rate = rospy.Rate(3)
    
    check = Checker()
    alive = False
    
    while not rospy.is_shutdown():
        
        check.life = min(max(check.life-.1, 0),1)
        
        if check.life > .6:
            alive = True
        elif check.life < .4:
            if alive:
                stopSim()  
            alive = False        
            
        rate.sleep()
        
