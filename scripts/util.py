from queue import deque
import numpy as np
import time
from scipy.signal import savgol_filter
class SavgolFilter(object):
    def __init__(self,size):
        self.size=size
        if self.size%2==0:self.size+=1
        self.histlen=100
        self.q=deque()
    def add(self,elem):
        if len(self.q) == self.histlen:
            self.q.append(elem)
            self.q.popleft()
        else:
            self.q.append(elem)
    def value(self):
        data=np.array(list(self.q))
        if data.shape[0]<self.size:return np.mean(data)
        filtered=savgol_filter(data,self.size,1)
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
def clip(v,minv,maxv):
    return min(max(v,minv),maxv)
