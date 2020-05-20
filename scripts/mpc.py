import rospy
import osqp
import numpy as np
from scipy import sparse,linalg

class LinearMPC:
    #shamelessly copied from Tom Scherlis' drone project and modified: https://github.com/Toms42/drone_controls/blob/master/drone_control/scripts/drone_mpc/drone_mpc.py
    def __init__(self, A, B, Q, R, S, N, dt, x_constraints=None, u_constraints=None):
        A = dt * A + np.eye(len(A))
        B = dt * B
        self.N = N
        self.dt = dt

        xdim = len(A)
        udim = B.shape[1]

        self.xdim = xdim
        self.udim = udim

        INF = 1e10

        if x_constraints is None:
            x_constraints = np.hstack((-INF * np.ones((xdim, 1)), INF * np.ones((xdim, 1))))
        if u_constraints is None:
            u_constraints = np.hstack((-INF * np.ones((udim, 1)), INF * np.ones((udim, 1))))
            print(u_constraints)

        L_x = np.vstack((np.eye(xdim), -np.eye(xdim)))
        b_x = np.vstack((x_constraints[:, 1][np.newaxis].transpose(), -x_constraints[:, 0][np.newaxis].transpose()))
        L_u = np.vstack((np.eye(udim), -np.eye(udim)))
        b_u = np.vstack((u_constraints[:, 1][np.newaxis].transpose(), -u_constraints[:, 0][np.newaxis].transpose()))

        self.L_u_bar = linalg.block_diag(*tuple([L_u] * N))
        self.L_x_bar = linalg.block_diag(*tuple([L_x] * (N + 1)))
        self.b_u_bar = np.vstack(tuple([b_u] * N))
        self.b_x_bar = np.vstack(tuple([b_x] * (N + 1)))
        self.Q_bar = linalg.block_diag(*tuple([Q] * (N + 1)))
        self.Q_bar[-xdim - 1:-1, -xdim - 1:-1] = S
        self.R_bar = linalg.block_diag(*tuple([R] * N))
        self.A_bar = self._build_a_bar(A)
        self.B_bar = self._build_b_bar(A, B)
        self.m = None

    def solve(self, x, x_ref):
        assert x.shape[0] == self.xdim
        assert x.shape[1] == 1

        assert x_ref.shape[0] == self.xdim
        assert x_ref.shape[1] == self.N + 1

        f_bar = np.dot(self.A_bar, x)
        G_bar = x_ref.transpose().reshape((self.xdim * (self.N + 1), 1))
        C = np.dot(np.dot(f_bar.transpose(), self.Q_bar), self.B_bar) - np.dot(np.dot(G_bar.transpose(), self.Q_bar),
                                                                               self.B_bar)
        b = np.vstack((self.b_u_bar, self.b_x_bar - np.dot(self.L_x_bar, f_bar)))
        if self.m is None:
            H = np.dot(np.dot(self.B_bar.transpose(), self.Q_bar), self.B_bar) + self.R_bar
            L = np.vstack((self.L_u_bar, np.dot(self.L_x_bar, self.B_bar)))
            self.m=osqp.OSQP()
            self.m.setup(P=sparse.csc_matrix(H), q=C.transpose(), l=None, A=sparse.csc_matrix(L), u=b, verbose=False,warm_start=True)
        else:
            self.m.update(q=C.transpose(),l=None,u=b)
        res = self.m.solve()
        u = res.x
        x = f_bar + np.dot(self.B_bar, u[np.newaxis].transpose())

        u = u.reshape(self.N, self.udim).transpose()
        x = x.reshape(self.N + 1, self.xdim).transpose()
        return u, x
    def _build_a_bar(self, A):
        rm = A.shape[0]
        cm = A.shape[1]
        A_bar = np.zeros((rm * (self.N+1), cm))
        for i in range(self.N+1):
            A_bar[rm * i:rm * (i + 1), :] = np.linalg.matrix_power(A, i)
        return A_bar

    def _build_b_bar(self, A, B):
        rm = B.shape[0]
        cm = B.shape[1]
        B_bar = np.zeros((rm * (self.N+1), cm * self.N))
        for r in range(self.N+1):
            for c in range(self.N):
                order = r - c - 1
                if order < 0:
                    B_bar[rm * r:rm * (r + 1), cm * c:cm * (c + 1)] = np.zeros(B.shape)
                else:
                    B_bar[rm * r:rm * (r + 1), cm * c:cm * (c + 1)] = np.dot(np.linalg.matrix_power(A, order), B)
        return B_bar
def test_buggy():
    vf=10
    l=1.6
    A=np.array([[0,vf,0],[0,0,vf/l],[0,0,0]])
    B=np.array([[0],[0],[1]])
    Q=np.eye(3)
    Q[0,0]=10
    R=1
    S=Q
    N=50
    dt=.02
    mpc=LinearMPC(A,B,Q,R,S,N,dt)
    #state is lateral offset,ang offset, steer
    #input is steering rate
    import time
    x=np.array([[1],[0],[0]])
    for i in range(100):
        xr=np.hstack([np.array([[2],[0],[0]])]*(N+1))
        utraj,xtraj=mpc.solve(x,xr)
        x=x+dt*A.dot(x)+dt*B.dot(utraj[0,0])
        print('x',x.T)
def test_segway():
    A=np.zeros((4,4))
    A[0,1]=1
    A[1,2]=6.9126
    A[2,3]=1
    A[3,2]=81.6765
    B=np.array([[0],[.7432],[0],[3.6319]])
    Q=np.eye(4)
    Q[0,0]=10
    Q[1,1]=10
    R=1
    S=Q
    rate=100
    N=50
    dt=1/rate
    ucons=np.array([[-10,10]])
    mpc=LinearMPC(A,B,Q,R,S,N,dt,u_constraints=ucons)
    import time
    x=np.array([[1],[0],[0],[0]])
    for i in range(1000):
        start=time.time()
        xr=np.hstack([np.array([[0],[0],[0],[0]])]*(N+1))
        utraj,xtraj=mpc.solve(x,xr)
        x=x+dt*A.dot(x)+dt*B.dot(utraj[0,0])
        print(time.time()-start)
        print('x',x.T)
if __name__=="__main__":
    test_segway()
