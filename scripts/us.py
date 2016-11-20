#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
import rospy
from vrep_common.msg import ProximitySensorData
from pylab import *


def buildM(xyt):
    x,y,t = xyt
    return matrix([[cos(t),-sin(t),x],[sin(t),cos(t),y],[0,0,1]])
    
    
class Listener:
    def __init__(self, i):
        self.d = matrix([[1.],[0.],[1.]])
        self.sub = rospy.Subscriber('vrep/us%i' % i, ProximitySensorData, self.us_callback)
        self.t = rospy.Time.now().to_sec()
        
    def us_callback(self, msg):
        self.t = rospy.Time.now().to_sec()
        self.d[0,0] = norm([msg.detectedPoint.x,msg.detectedPoint.y,msg.detectedPoint.z])
        
    def get_d(self,t):        
        if t - self.t > 0.5:
            self.d[0,0] = 1.
        return self.d
            
            

# main
rospy.init_node('us_plot')
rate = rospy.Rate(10)
# sensor poses
us_range = range(1,17)
Ms = []
Mis = []
for i in us_range:
    par = "/sensors/us%i/" % i
    Ms.append(buildM([rospy.get_param(par+v) for v in ('x','y','theta')]))
    Mis.append(inv(Ms[-1]))

listeners = [Listener(i) for i in us_range]    
    
# plot
close('all')
ion()
f = figure()
pt, = plot([],[],'*')
ln = [plot([],[],'b')[0] for i in us_range]
ax = gca()
ax.axis([-1.3,1.3,-1.3,1.3])
ax.set_aspect('equal')

# plot sensor frames
for i in xrange(16):
    O = Ms[i]*matrix([[0],[0],[1]])
    X = concatenate((O, Ms[i]*matrix([[.1],[0],[1]])),1).tolist()    
    Y = concatenate((O, Ms[i]*matrix([[0],[.1],[1]])),1).tolist()
    plot(X[0],X[1],'r',linewidth=2)
    X = concatenate((O, Ms[i]*matrix([[1.],[0],[1]])),1).tolist()
    plot(X[0],X[1],'r--',linewidth=1)
    plot(Y[0],Y[1],'g',linewidth=2)
    text(O[0,0],O[1,0],str(i+1))

show()


while not rospy.is_shutdown():
    t = rospy.Time.now().to_sec()
    D = [listeners[i].get_d(t) for i in range(16)]
    xy = [(Ms[i]*D[i])[:2,0].tolist() for i in range(16)]
    xy = array(xy)[:,:,0].transpose()
    print 'setting data to',xy
    pt.set_data(xy[0],xy[1])
    
    
    # plot lines
    n = 2
    for i in xrange(16):
        if D[i][0,0] == 1:
            ln[i].set_data([],[])
        else:
            X = [D[i]]
            for k in range(1,n+1):
                X.append(Mis[i]*Ms[(i-k) % 16]*D[(i-k) % 16])
                X.append(Mis[i]*Ms[(i+k) % 16]*D[(i+k) % 16])
            
            # fit a parabola
            y = array([p[1,0] for p in X]).reshape(2*n+1,1)
            y = concatenate((y*y,y,ones((2*n+1,1))),1)
            x = array([p[0,0] for p in X]).reshape(2*n+1,1)
            a,b,c = dot(pinv(y),x).reshape(3,).tolist()
            # corresponding points
            y = [p[1,0] for p in X]
            y = linspace(min(y),max(y),20).reshape(1,20)
            x = a*y*y+b*y+c
            X = (Ms[i]*matrix(concatenate((x,y,ones(x.shape)),0))).tolist()
            ln[i].set_data(X[0],X[1])
    
    
    
    
    
    
    
    f.canvas.draw()    
    f.canvas.flush_events()
    

    rate.sleep()
