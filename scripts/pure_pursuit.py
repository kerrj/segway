#!/bin/usr/env python3
import numpy as np
import rospy
from segway.msg import BaseCommand,Odometry,Path
from util import Transformer,scale
from math import hypot,pi
class PurePursuit:
    def __init__(self,L,mid_tol,goal_tol):
        self.path=[]
        self.slowdowns=[]
        self.pose=None
        self.goal_tol=goal_tol
        self.mid_tol=mid_tol
        self.slowdist=.4
        self.L=L
    def getControl(self,v):
        #v is the desired forward vel
        if self.pose is None:
            return 0,0
        while len(self.path)>1 and hypot(self.path[1][0]-self.pose[0],self.path[1][1]-self.pose[1])<self.mid_tol:
            #if we are within lookahead dist of next waypoint, increment our finger
            self.path.pop(0)
            self.slowdowns.pop(0)
        if (len(self.path)==1 and hypot(self.pose[0]-self.path[0][0],self.pose[1]-self.path[0][1])<self.goal_tol):
            self.path.pop(0)
            self.slowdowns.pop(0)
        if len(self.path)==0:return 0,0
        x,y=self.getLookahead()
        H=Transformer.pose2Mat(*self.pose)
        H_rob_glo=Transformer.invH(H)
        xrob,yrob=Transformer.applyH(H_rob_glo,x,y)
        k=(2*yrob)/(self.L**2)
        if len(self.path)>1:
            distToNext=hypot(self.path[1][0]-self.pose[0],self.path[1][1]-self.pose[1])
            d2last=hypot(self.path[0][0]-self.pose[0],self.path[0][1]-self.pose[1])
            s1=scale(distToNext,0,self.slowdist,self.slowdowns[1],1)
            s2=scale(d2last,0,self.slowdist,self.slowdowns[0],1)
            s=min(s1,s2)
        else:
            distToNext=hypot(self.pose[0]-self.path[0][0],self.pose[1]-self.path[0][1])
            s=scale(distToNext,0,self.slowdist,self.slowdowns[0],1)
        v=s*v
        w=v*k
        return v,w
    def getLookahead(self):
        #an invariant here is that the lookahead should always be between the 0th and 1st elements in self.path
        if len(self.path)==1:
            return self.path[0]
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
    def getSlowdown(self,i):
        if i==len(self.path)-1:return 0
        if i==0:return 1
        scale=.3/(pi/2)
        p1=self.path[i-1]
        p2=self.path[i]
        p3=self.path[i+1]
        v1=np.array([p2[0]-p1[0],p2[1]-p1[1]])
        v2=np.array([p3[0]-p2[0],p3[1]-p2[1]])
        th=np.arccos(v1.dot(v2)/(np.linalg.norm(v1)*np.linalg.norm(v2)))
        return abs(th)*scale
    def updatePath(self,pathMsg):
        #if we have a pose, append it to the beginning
        if self.pose is None:
            self.path=[(m.x,m.y) for m in pathMsg.waypoints]
        else:
            self.path=[(self.pose[0],self.pose[1])]+[(m.x,m.y) for m in pathMsg.waypoints]
            self.slowdowns=[self.getSlowdown(i) for i in range(len(self.path))]
    def updateOdom(self,odomMsg):
        #if this is the first reading, append it to the beginning of a path if there is one
        if self.pose is None and len(self.path)>0:
            self.pose=(odomMsg.x,odomMsg.y,odomMsg.th)
            self.path=[(odomMsg.x,odomMsg.y)]+self.path
            self.slowdowns=[self.getSlowdown(i) for i in range(len(self.path))]
        else:
            self.pose=(odomMsg.x,odomMsg.y,odomMsg.th)
if __name__=="__main__":
    from geometry_msgs.msg import Point
    pp=PurePursuit(.2,.1,.05)
    path=Path()
    path.waypoints=[Point(1.,0,0),Point(1.,1.,0)]
    pp.updatePath(path)
    o=Odometry()
    pp.updateOdom(o)
    print(pp.slowdowns)
    print(pp.getControl(.1))
