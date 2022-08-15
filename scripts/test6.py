import math
Xu = 2.245 #2.245 # linear drag coeff along x-direction - surge motion
Xuu = 22
m = 18
kp_u = 1 #25 #proportional gain in the surge contoller
kp_v = 0.5 #proportional gain in the sway contoller
kp_psi = 2.5 #proportional gain in the yaw heading contoller
kd_psi = 2.5 #derivative gain in the yaw heading contoller
ki_psi = 2 #integral gain in the yaw heading contoller
e_psi_integral = 0



k_u_amax=5 #positive surge acceleration gain
u_dot_max = 5 # maximum allowed surge acceleration
u_d_yaw = 5 # desired surge velocity for periods of yaw motion

k_v_amax=0.1 #positive sway acceleration gain
v_dot_max = 0.1 # maximum allowed sway acceleration
v_d_yaw = 0.1 # desired sway velocity for periods of yaw motion

 
e_integral = 0
K_P = 1
K_I = 0.25
Beta = 0 #the s
def tau_x(eta_u,u):
    X_drag=Xu*u+Xuu*abs(u)*u
    tau_x=m*eta_u+X_drag
    return tau_x
def eta_u(u,u_d,psi,psi_d):
        #e_psi= psi_d-psi
        #e_psi=e_psi
        e_psi = 0.17396
        u_d_ref=min(u_d_yaw+(u_d-u_d_yaw)*math.exp(-5.73-abs(e_psi)),u_d)
        u_d_dot=u_dot_max*math.tanh(k_u_amax*(u_d_ref-u)/u_dot_max)
        e_u=u - u_d
        eta_u=u_d_dot-kp_u*e_u
        return eta_u
eta_u0 = -1.899
u = 0.425
u_d =0.1
psi = 1.18
psi_d = 1.35
tau_x = tau_x(eta_u0,u)
print(tau_x)
eta_ue = eta_u(u,u_d,psi,psi_d)
print(eta_ue)
