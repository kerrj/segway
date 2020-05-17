import numpy as np

class EKF:
    def __init__(self,f,h,Q,R,x_init,P_init):
        '''
        f and h are functions which compute their respective values. They must return
        column vectors representing x and z respectively. Q is process covariance, R is 
        observation covariance
        They will be numerically differentiated to determine the jacobian for F and H
        matrices.
        '''
        self.f=f
        self.h=h
        self.Q=Q
        self.R=R
        self.x=x_init
        self.P=P_init

    def update(self,z,fargs=(),hargs=()):
        F=self.jacobian(self.f,self.x,args=fargs)
        H=self.jacobian(self.h,self.x,args=hargs)
        xhat=self.f(self.x,*fargs)
        self.P=F.dot(self.P).dot(F.T)+self.Q
        y=z-self.h(xhat,*hargs)
        S=H.dot(self.P).dot(H.T)+self.R
        K=self.P.dot(H.T).dot(np.linalg.inv(S))
        self.x=xhat+K.dot(y)
        self.P=(np.eye(self.x.shape[0])-K.dot(H)).dot(self.P)
        return self.x

    def jacobian(self,f,x,eps=.0001,args=()):
        '''
        Returns takes the jacobian of f WRT x. x is a column vector
        '''
        J=np.zeros((f(x,*args).shape[0],x.shape[0]))
        for col in range(x.shape[0]):
            #compute the col of the jacobian
            f1=f(x,*args)
            newx=x.copy()
            newx[col,:]+=eps
            f2=f(newx,*args)
            J[:,col:col+1]=(f2-f1)/eps#the funny indexing here matches dims
        return J
