from scipy.optimize import fsolve



#             tau_L + tau_R
#                     tau_M
# (B*tau_L)/2 - (B*tau_R)/2
#   LP1*tau_L <= 35
    

#x[0]: tau_L
#x[1]: tau_R
#x[2]: tau_M

#x[3]: LP1
#x[4]: LP2
#x[5]: LP3

#x[6]: LN1
#x[7]: LN2
#x[8]: LN3
B = 0.532
tau_x = 10
tau_y = 0
tau_N = 2
def func2(x):
    return [x[0] + x[1] - tau_x,
            x[2] - tau_y,
            B*x[0]/2 - B*x[1]/2 - tau_N,
            x[3]*x[0] - 35,
            x[4]*x[1] - 35,
            x[5]*x[2] - 35]
            
root = fsolve(func2, [0.1, 0.1,0.1, 0.1,0.1, 0.1])