from queue import deque
import numpy as np
import time
from scipy import signal
from math import copysign,pi,atan2
class SavgolFilter(object):
    def __init__(self,size,order):
        self.size=size
        if self.size%2==0:self.size+=1
        self.histlen=size*2
        self.q=deque()
        self.order=order
    def add(self,elem):
        if len(self.q) == self.histlen:
            self.q.append(elem)
            self.q.popleft()
        else:
            self.q.append(elem)
    def value(self):
        data=np.array(list(self.q))
        if data.shape[0]<self.size:return np.mean(data)
        filtered=signal.savgol_filter(data,self.size,self.order)
        return filtered[-1]

class RunningAverage(object):
    def __init__(self,size):
        self.q=deque()
        self.sum=0
        self.size=size
    def add(self,elem):
        if len(self.q) == self.size:
            self.q.append(elem)
            self.sum=self.sum+elem-self.q.popleft()
        else:
            self.q.append(elem)
            self.sum+=elem
    def value(self):
        return self.sum/len(self.q)
class AdaptiveAverage(object):
    def __init__(self,windows,cutoffs):
        #cutoffs should include 0, corresponds to range [c[i],c[i+1]] for window size i
        self.avgs=[RunningAverage(l) for l in windows]
        cutoffs.sort
        self.cutoffs=cutoffs
    def add(self,e):
        self.val=None
        for i in range(len(self.avgs)):
            self.avgs[i].add(e)
            if abs(e)>=self.cutoffs[i]:
                self.val=i
    def value(self):
        return self.avgs[self.val].value()

class MedianFilter(object):
    def __init__(self,size,n):
        self.q=deque()
        self.size=size
        self.n=n
    def add(self,elem):
        if len(self.q) == self.size:
            self.q.append(elem)
            self.q.popleft()
        else:
            self.q.append(elem)
    def value(self):
        l=list(self.q)
        l.sort()
        d=(self.n-1)//2
        mid=len(l)//2
        return np.mean(l[mid-d:mid+d+1])
class WeightedSum:
    def __init__(self,weight):
        self.w=weight
        self.sum=0
    def add(self,e):
        self.sum=self.w*e+self.sum*(1-self.w)
    def value(self):
        return self.sum
class CascadeAverage(object):
    def __init__(self,sizes):
        self.avgs=[RunningAverage(s) for s in sizes]

    def add(self,e):
        self.avgs[0].add(e)
        for i in range(1,len(self.avgs)):
            self.avgs[i].add(self.avgs[i-1].value())
    def value(self):
        return self.avgs[-1].value()
class ButterFilter(object):
    def __init__(self,rate,order,cutoff):
        self.q=deque()
        self.zq=deque()
        self.histlen=rate
        self.sos=signal.butter(order,cutoff,output='sos',fs=rate)

    def add(self,elem):
        if len(self.q) == self.histlen:
            self.q.append(elem)
            self.q.popleft()
        else:
            self.q.append(elem)
    def value(self):
        data=np.array(list(self.q))
        filtered=signal.sosfilt(self.sos,data)
        self.val=filtered[-1]
        return filtered[-1]
def clip(v,minv,maxv):
    return min(max(v,minv),maxv)

def increment(v,t,a):
    dv=copysign(a,t-v)
    newv=v+dv
    if abs(v-t)<=1.1*a:newv=t
    return newv,newv-v
def wrapTo2Pi(th):
    while th>=2*pi:
        th-=2*pi
    while th<0:
        th+=2*pi
    return th
class Transformer:
    @staticmethod
    def pose2Mat(x,y,th):
        return np.array([
            [np.cos(th),-np.sin(th),x],
            [np.sin(th),np.cos(th),y],
            [0,0,1]])
    @staticmethod
    def mat2Pose(H):
        x=H[0,2]
        y=H[1,2]
        th=atan2(H[1,0],H[0,0])
        return (x,y,th)
    @staticmethod
    def invH(H):
        R=H[0:2,0:2]
        Hinv=np.eye(3)
        Hinv[0:2,0:2]=R.T
        Hinv[0:2,2:3]=-R.T.dot(H[0:2,2:3])
        return Hinv
    @staticmethod
    def applyH(H,x,y):
        p=np.array([[x],[y],[1]])
        pt=H.dot(p)
        return (pt[0,0],pt[1,0])
