import numpy as np
import numpy.linalg as LA
from numpy import linalg 
import scipy.optimize as optimize

def fmin(f):
    #y = np.dot(A, x) - b
    W = np.array([[1,0,0],[0,1,0],[0,0,1]])
    J = np.matmul(np.matmul(np.transpose(f),W),f)
    return J



B = 0.538
taus  = [8,0,6.5]
fmins = [-30,-30,-30]
fmaxs = [35,35,35]

def constraint(f):
    # return f
    if (f > 35)  : 
        
        return 0
    if(f < -30)  :
        return 0
    else                            : return f
T = np.array([[1,1,0],[0,0,1],[0.5*B,-0.5*B,0]])
# {'type': 'ineq', 'fun': lambda f: constraint(f[0])},
cons = (
        {'type': 'eq',   'fun': lambda f: T[0][0]*constraint(f[0]) +T[0][1]*constraint(f[1]) + T[0][2]*constraint(f[2]) - taus[0]},
        {'type': 'eq',   'fun': lambda f: T[1][0]*constraint(f[0]) +T[1][1]*constraint(f[1]) + T[1][2]*constraint(f[2]) - taus[1]},
        {'type': 'eq',   'fun': lambda f: T[2][0]*constraint(f[0]) +T[2][1]*constraint(f[1]) + T[2][2]*constraint(f[2]) - taus[2]}
        )

x0 = [0,0,0]
#bnds = ((-30, 35), (-30, 35), (-30, 35))
res = optimize.minimize(fmin, x0 ,constraints=cons,tol=1e-1,method='SLSQP', options={'disp': False})
achieved_forces = np.matmul(T,res.x)
print("Desired forces: {}".format(taus))
print ("Calculated thrusts: {}".format(np.round(res.x)))
print("Achieved forces: {}".format(np.round(achieved_forces)))
