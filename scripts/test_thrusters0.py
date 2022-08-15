import numpy as np
B = 0.538
C = 0.1
def thruster_forcesV2(tau_x,tau_y,tau_N,clipping = True):  
    #B = 0.538
    #tau_L=0.5*tau_x
    #tau_R=0.5*tau_y
    f = np.array([[1,1,0],[ 0, 0, 1],[0.5*B, -0.5*B, C]])
    h = np.array([tau_x,tau_y,tau_N])
    #print(h)
    taus = np.matmul(np.linalg.inv(f) ,np.array([tau_x,tau_y,tau_N]))
    tau_L,tau_R,tau_M = taus[0],taus[1],taus[2]
    return tau_L,tau_R,tau_M
tau_L,tau_R,tau_M = thruster_forcesV2(-38.6275,9.56817,9.303, True)
print(tau_L,tau_R,tau_M)      