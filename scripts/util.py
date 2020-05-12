from queue import deque
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
    def avg(self):
        return self.sum/len(self.q)
def clip(v,minv,maxv):
    return min(max(v,minv),maxv)
