#!/bin/usr/env python3
import numpy as np
import rospy
from segway.msg import BaseCommand,Odometry,Path
from util import Transformer
from math import hypot
class PurePursuit:
    def __init__(self,L,goal_tol):
        self.path=[]
        self.pose=None
        self.goal_tol=goal_tol
        self.L=L
    def getControl(self,v):
        #v is the desired forward vel
        if self.pose is None:
            return 0,0
        while len(self.path)>1 and hypot(self.path[1][0]-self.pose[0],self.path[1][1]-self.pose[1])<self.goal_tol:
            #if we are within lookahead dist of next waypoint, increment our finger
            self.path.pop(0)
        if (len(self.path)==1 and hypot(self.pose[0]-self.path[0][0],self.pose[1]-self.path[0][1])<self.goal_tol):
            self.path.pop(0)
        if len(self.path)==0:return 0,0
        x,y=self.getLookahead()
        H=Transformer.pose2Mat(*self.pose)
        H_rob_glo=Transformer.invH(H)
        xrob,yrob=Transformer.applyH(H_rob_glo,x,y)
        k=(2*yrob)/(self.L**2)
        w=v*k
        return v,w
    def getLookahead(self):
        #an invariant here is that the lookahead should always be between the 0th and 1st elements in self.path
        if len(self.path)==1:
            return self.path[0]
        res=.01
        curP=np.array(self.path[0])
        robP=np.array([self.pose[0],self.pose[1]])
        dp=np.array([self.path[1][0]-self.path[0][0],self.path[1][1]-self.path[0][1]])
        dp=.025*dp/np.linalg.norm(dp)
        #first go forward until distance to robot starts increasing
        dist=np.linalg.norm(robP-curP)
        newdist=np.linalg.norm(robP-(curP+dp))
        while(newdist<dist):
            curP=curP+dp
            dist=newdist
            newdist=np.linalg.norm(robP-(curP+dp))
        #now go forward until we reach lookahead point
        while(np.linalg.norm(curP-robP)<self.L):
            curP+=dp
        return curP[0],curP[1]
    def updatePath(self,pathMsg):
        #if we have a pose, append it to the beginning
        if self.pose is None:
            self.path=[(m.x,m.y) for m in pathMsg.waypoints]
        else:
            self.path=[(self.pose[0],self.pose[1])]+[(m.x,m.y) for m in pathMsg.waypoints]
    def updateOdom(self,odomMsg):
        #if this is the first reading, append it to the beginning of a path if there is one
        if self.pose is None and len(self.path)>0:
            self.pose=(odomMsg.x,odomMsg.y,odomMsg.th)
            self.path=[(odomMsg.x,odomMsg.y)]+self.path
        else:
            self.pose=(odomMsg.x,odomMsg.y,odomMsg.th)
if __name__=="__main__":
    from geometry_msgs.msg import Point
    pp=PurePursuit(.2,.1)
    path=Path()
    path.waypoints=[Point(1.,0,0)]
    pp.updatePath(path)
    o=Odometry()
    o.x=.95
    pp.updateOdom(o)
    print(pp.getControl(.1))
