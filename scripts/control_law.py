
from tkinter import S
from xml.sax.saxutils import XMLFilterBase
import numpy as np
import math 
import matplotlib.pyplot as plt
from sympy import symbols, Eq, solve
from scipy.optimize import fsolve
import numpy.linalg as LA
from numpy import linalg 
import scipy.optimize as optimize

class control_jellyfishbot():
    def __init__(self):
        ## defining model paramaeters 
        self.m= 18 #kg
        self.Iz= 2.37 #inertia around z
        self.dt=1.0/10 #time interval in seconds, depending on the frequency of the sensor readings
        self.t_final=10
        self.B = 0.538 #distance between the left and the right thrusters
        self.C = 0.1 #ditance between middle thruster and center of gravity

        self.Xu = 2.245 #2.245 # linear drag coeff along x-direction - surge motion
        self.Xuu = 22 #19.05 # non-linear drag coeff along x-direction - surge motion
        self.Yv = 0.19 #-0.1972 # linear drag coeff along y-direction - sway motion
        self.Yvv = 78 #71.74 # non-linear drag coeff along y-direction - sway motion
        self.Nr = 2.591 #-0.01391 # linear drag coeff around z-direction - yaw motion
        self.Nrr = 3 #2.591 # non-linear drag coeff around z-direction - yaw motion
        #Dx= Xu*u+Xuu*|u|*u

        self.kp_u = 1 #25 #proportional gain in the surge contoller
        self.kp_v = 0.5 #proportional gain in the sway contoller
        self.kp_psi = 2.5 #proportional gain in the yaw heading contoller
        self.kd_psi = 2.5 #derivative gain in the yaw heading contoller
        self.ki_psi = 2 #integral gain in the yaw heading contoller
        self.e_psi_integral = 0



        self.k_u_amax=5 #positive surge acceleration gain
        self.u_dot_max = 5 # maximum allowed surge acceleration
        self.u_d_yaw = 5 # desired surge velocity for periods of yaw motion

        self.k_v_amax=0.1 #positive sway acceleration gain
        self.v_dot_max = 0.1 # maximum allowed sway acceleration
        self.v_d_yaw = 0.1 # desired sway velocity for periods of yaw motion

        self.e_psi0 = 0
        self.e_psi = 0

        self.e_integral = 0
        self.K_P = 1
        self.K_I = 0.25
        self.Beta = 0 #the sideslip angle (drift) which can be neglected to simplify steering law

     
        self.y_k1 = 0
        self.y_k = 0
        self.x_k1 = 0
        self.x_k = 0
        self.xf = 10000
        self.yf = 10000
        self.xf0 = 0
        self.yf0 = 0
        self.distance = 1000
        self.enable_print = True

        self.xd = 9999
        self.yd = 9999
        self.u_d = 0
        self.v_d = 0


        self.Yp = 0
        self.ax = 0
        self.ay = 0
        self.delta =0

        self.xp = 0
        self.yp = 0
        self.slope = 0
        self.MATH_ERROR = False
        self.getting_away = False
        self.distance0 = 10000
        self.sign = 1
        self.clip_counter = 0
        self.counter = 0

        self.stop = False #A flag for check if the robot arrived to the final point and should stop
    def tau_x(self,eta_u,u):
        X_drag=self.Xu*u+self.Xuu*abs(u)*u
        tau_x=self.m*eta_u+X_drag
        return tau_x
    def tau_y(self,eta_v,v):
        Y_drag=self.Yv*v+self.Yvv*abs(v)*v
        tau_y=self.m*eta_v+Y_drag
        return tau_y
    def tau_N(self,eta_psi,r):
        N_drag=self.Nr*r+self.Nrr*abs(r)*r
        tau_N=self.Iz*eta_psi+N_drag
        print ("Iz: {}".format(self.Iz))
        print ("eta_psi: {}".format(eta_psi))
        return tau_N

    def eta_u(self,u,u_d,psi,psi_d):
        #e_psi= psi_d-psi
        #self.e_psi=e_psi
        u_d_ref=min(self.u_d_yaw+(u_d-self.u_d_yaw)*math.exp(-5.73-abs(self.e_psi)),u_d)
        u_d_dot=self.u_dot_max*math.tanh(self.k_u_amax*(u_d_ref-u)/self.u_dot_max)
        e_u=self.u - u_d
        eta_u=u_d_dot-self.kp_u*e_u
        return eta_u

    def eta_v(self,v,v_d,psi,psi_d):
        #e_psi= psi_d-psi
        v_d_ref=min(self.v_d_yaw+(v_d-self.v_d_yaw)*math.exp(-5.73-abs(self.e_psi)),v_d)
        v_d_dot=self.v_dot_max*math.tanh(self.k_v_amax*(v_d_ref-v)/self.v_dot_max)
        e_v=v_d-self.v
        eta_v=v_d_dot-self.kp_v*e_v
        return eta_v

    def eta_psi(self,v,v_d,psi,psi_d):
        #e_psi= psi_d-psi
        print("e_psi: {}".format(self.psi))
        e_dot_psi= (self.e_psi-self.e_psi0)/self.dt
        e_psi = self.e_psi-self.e_psi0
        self.e_psi_integral = self.e_psi_integral + e_psi*self.dt
        eta_psi=-self.kp_psi*self.e_psi-self.kd_psi*e_dot_psi - self.ki_psi*self.e_psi_integral
        self.e_psi0=self.e_psi
        #5print ("self.e_psi: {}".format(self.e_psi))
        #5print ("self.e_psi0: {}".format(self.e_psi0))
        #5print ("e_dot_psi: {}".format(e_dot_psi))
        #5print ("e_psi: {}".format(e_psi))
        return eta_psi


    def u_d(self,t):
        u_d=math.sin(t)
        return u_d
    def v_d(self,t):
        v_d=math.cos(t)
        return v_d
    def psi_d(self,t):
        psi_d=20 #should be the desired heading angle towards the target
        return psi_d

    def thruster_forces(self,tau_x,tau_y,tau_N):  #FIXME

        #tau_L=0.5*tau_x
        #tau_R=0.5*tau_y
        tau_M= tau_y
        tau_L, tau_R = symbols('tau_L tau_R')
        eq1 = Eq(tau_L + tau_R - tau_x)
        eq2 = Eq((self.B*(tau_L-tau_R))/2 - tau_N)
        sols= solve((eq1,eq2), (tau_L, tau_R))
        tau_L = sols[tau_L]
        tau_R = sols[tau_R]
        return tau_L,tau_R,tau_M
    def thruster_forcesV2(self,tau_x,tau_y,tau_N,clipping = True):  
        #B = 0.538
        #tau_L=0.5*tau_x
        #tau_R=0.5*tau_y
        f = np.array([[1,1,0],[ 0, 0, 1],[0.5*self.B, -0.5*self.B, self.C]])
        h = np.array([tau_x,tau_y,tau_N])
        #print(h)
        taus = np.matmul(np.linalg.inv(f) ,np.array([tau_x,tau_y,tau_N]))
        tau_L,tau_R,tau_M = taus[0],taus[1],taus[2]
        self.counter = self.counter +1
        if clipping == True:
            if tau_L > 35 or tau_R > 35 or tau_M > 35 or tau_L < -30 or tau_R < -30 or tau_M < -30:
                tau_L,tau_R,tau_M = self.clip_thrusters_const(tau_x,tau_y,tau_N)
                self.clip_counter = self.clip_counter +1
        #5print ("How many times, the clipping was applied: {}/{}".format(self.clip_counter,self.counter))
        return tau_L,tau_R,tau_M

    def clip_thrusters(self,tau_L,tau_R,tau_M):
        tau_max = max(tau_L,tau_R,tau_M)
        tau_min = min(tau_L,tau_R,tau_M)

        if tau_max > abs(tau_min) and tau_max > 30:
            tau_L =  (tau_L/tau_max)*30
            tau_R =  (tau_R/tau_max)*30
            tau_M =  (tau_M/tau_max)*30
        if tau_max < abs(tau_min) and tau_min < -30: 
            tau_L =  (tau_L/abs(tau_min))*30
            tau_R =  (tau_R/abs(tau_min))*30
            tau_M =  (tau_M/abs(tau_min))*30
        return tau_L,tau_R,tau_M
    def constraint(self,f):
        # return f
        if (f > 35)  : 
            
            return 0
        if(f < -30)  :
            return 0
        else                            : return f
    def fmin(self,f):
            W = np.array([[1,0,0],[0,1,0],[0,0,1]])
            J = np.matmul(np.matmul(np.transpose(f),W),f)
            return J
    def clip_thrusters_const(self,tau_x,tau_y,tau_N):

        taus  = [tau_x,tau_y,tau_N]
        T = np.array([[1,1,0],[0,0,1],[0.5*self.B,-0.5*self.B,self.C]])
        # {'type': 'ineq', 'fun': lambda f: constraint(f[0])},
        cons = (
                {'type': 'eq',   'fun': lambda f: T[0][0]*self.constraint(f[0]) +T[0][1]*self.constraint(f[1]) + T[0][2]*self.constraint(f[2]) - taus[0]},
                {'type': 'eq',   'fun': lambda f: T[1][0]*self.constraint(f[0]) +T[1][1]*self.constraint(f[1]) + T[1][2]*self.constraint(f[2]) - taus[1]},
                {'type': 'eq',   'fun': lambda f: T[2][0]*self.constraint(f[0]) +T[2][1]*self.constraint(f[1]) + T[2][2]*self.constraint(f[2]) - taus[2]}
                )

        x0 = [0,0,0]
        res = optimize.minimize(self.fmin, x0 ,constraints=cons,tol=1e-1,method='SLSQP', options={'disp': False})
        tau_L,tau_R,tau_M = res.x[0],res.x[1], res.x[2]
        return tau_L,tau_R,tau_M


    def force_to_pwm(self,force):
        if force <= 0:
            p1 =       15.76  
            p2 =       163.3  
            p3 =        1453  
            pwm = p1*force**2 + p2*force + p3
        else:
            p1 =      -10.45  
            p2 =       130.4  
            p3 =        1548  

            pwm = p1*force**2 + p2*force + p3

        return pwm 
    def pwm_L(self,tau_L): 
        pwm_L=self.force_to_pwm(tau_L)
        return pwm_L
    def pwm_R(self,tau_R): 
        pwm_R=self.force_to_pwm(tau_R)
        return pwm_R
    def pwm_M(self,tau_M): 
        pwm_M=self.force_to_pwm(tau_M)
        return pwm_M

    def jellyfishbot(self,t,pwm_L,pwm_R,pwm_M):
        u=self.u_d(t)*1.1
        v=self.v_d(t)*1.1
        r=self.u_d(t)*1.1
        psi_d=self.psi_d(t)
        psi=psi_d*1.4
        return u,v,r,psi
    def projected_point(self):
        a,b = symbols('a b')
        eq1 = Eq(-1/math.tan(self.Yp)*(self.y-a)+b-self.x)
        eq2 = Eq(math.tan(self.Yp)*a+self.ax-b)

        sol_dict = solve((eq1,eq2), (a,b))
        yp = sol_dict[a]
        xp = sol_dict[b]
        return [yp,xp]
    def projected_point2(self):
        #5print("ax = {}".format(self.ax))
        #5print("jellyfishbot_control_system.x = {}".format(self.x))
        def func(x):
            return [-1/math.tan(self.Yp)*(self.y-x[0])+x[1]-self.x,
            math.tan(self.Yp)*x[0]+self.ax-x[1]]
            
        root = fsolve(func, [0.1, 0.1])
        return root
    def projected_point3(self):
        a = self.x - self.ax
        b = self.y - self.ay
        c = self.xf - self.ax
        d = self.yf - self.ay
        dot = a*c + b*d
        len_sq = c * c + d * d
        param = -1
        if (len_sq != 0): #// in case of 0 length line
            param = dot / len_sq
        if param < 0:
            xx = self.ax
            yy = self.ay
        elif param > 1:
            xx = self.xf
            yy = self.yf
        else:
            xx = self.ax + param * c
            yy = self.ay + param *d

        #print("xx,yy = {},{}".format(xx,yy))
        return yy,xx
    def calculate_math_error(self):
        a = (1+self.slope**2)
        b = -2*self.yp -2*self.slope*self.xp
        c = self.yp**2 +self.xp**2 -self.delta**2
        # a,b,c = 2.0,-20.5272577369,101.671038775
        D = b**2-4*a*c
        if D < 0:
            self.MATH_ERROR = True
        else:
            self.MATH_ERROR  = False
            self.sign = 1
        
    def virtual_target(self): 
        # a = (1+self.slope**2)
        # b = -2*self.yp -2*self.slope*self.xp
        # c = self.yp**2 +self.xp**2 -self.delta**2
        self.ay = -self.ay
        a = (1+self.slope**2)
        b = -2*self.yp + 2*self.slope**2*self.ay - 2*self.slope*self.xp
        c = self.yp**2 + self.slope**2*self.ay**2 - 2*self.slope*self.ay*self.ax + self.xp**2 - self.delta**2
        
        # a = (1+self.slope**2)
        # b = (-2*self.yp - 2*self.slope**2*self.ay - 2*self.slope*self.ax - 2*self.slope*self.xp)
        # c = (self.yp**2 + self.slope**2*self.ay**2 + 2*self.slope*self.ay + self.ax**2 + 2*self.slope*self.ay*self.xp - 2*self.ax*self.xp + self.xp**2)
        #5print ("a,b,c = {},{},{}".format(a,b,c))
        #5print ("yp,xp = {},{}".format(self.yp,self.xp))
        D = b**2-4*a*c
        if D < 0:
            self.sign = -1
        else:
            self.sign = 1
        yd = (-b+math.sqrt(self.sign*(b**2-4*a*c)))/(2*a)
        
        # if self.Yp >= 0:
        #     yd = (-b+math.sqrt(self.sign*(b**2-4*a*c)))/(2*a)
        # else:
        #     print("hehehehe")
        #     yd = (-b-math.sqrt(self.sign*(b**2-4*a*c)))/(2*a) 
        #5print("slope = {}".format(self.slope))
        #5print("ay,ax = {},{}".format(self.ay,self.ax))
        self.ay = -self.ay
        xd = self.slope*(yd-self.ay)+self.ax
        return yd,xd
    def virtual_target2(self): 
        # check for horizontal line ax = xf = xp
        print("xf,yf: {},{}".format(self.xf,self.yf))
        if self.ax == self.xf:
            if self.yf >= self.yp:
                yd = self.yp + self.delta
            else:
                yd = self.yp - self.delta
            xd = self.xp
        # check for vertical line ay = yf = yp
        elif self.ay == self.yf:
            yd = self.yp
            if self.xf >= self.xp:
                xd = self.xp + self.delta
            else:
                xd = self.xp - self.delta
        else:
            if self.yf - self.yp == 0:
                slope = 99999999999999999
            else:
                slope = (self.xf - self.xp)/(self.yf - self.yp)
            def func(x):
                return [(x[1]-self.yp)**2+(x[0]-self.xp)**2-self.delta**2,
                        slope*(x[1]-self.ay)+self.ax-x[0]]
                        # slope*(x[1]-self.ay)+self.ax-x[0]
            
            root = fsolve(func, [0.1, 0.1])
            xd = root[0] 
            yd = root[1] 
            if self.xp <= xd <= self.xf or self.xp >= xd >= self.xf:
                c1 = True
            else:
                c1 = False
            if self.yp <= yd <= self.yf or self.yp >= yd >= self.yf:
                c2 = True
            else:
                c2 = False
            if c1 and c2:
                pass
            else:
                xd = xd + (self.xp-xd)*2
                yd = yd + (self.yp-yd)*2
        # if xd > self.xf and xd > self.xp or xd < self.xf and xd < self.xp:
        #     xd = self.xf
        # if yd > self.yf and yd > self.yp or yd < self.yf and yd < self.yp:
        #     yd = self.yf 
        if xd > self.xf and xd > self.xp:
            if self.xf >= self.xp:
                xd = self.xf
            else:
                xd = self.xp
        elif xd < self.xf and xd < self.xp:
            if self.xf < self.xp:
                xd = self.xf
            else:
                xd = self.xp
        if yd > self.yf and yd > self.yp :
            if self.yf >= self.yp:
                yd = self.yf
            else:
                yd = self.yp
        elif yd < self.yf and yd < self.yp:
            if self.yf < self.yp:
                yd = self.yf
            else:
                yd = self.yp         
       
        return yd,xd

    def check_arrival(self,threshold):
        self.distance = math.sqrt((self.xf - self.x)**2 + (self.yf-self.y)**2)
        if self.distance <= threshold:
            return True
        else:
            return False
    def check_direction(self):
        if self.distance - self.distance0 > 0:
            self.getting_away = True

    ###########################


    def test(self):
        u=0     #initial condition for the surge speed
        v=0     #initial condition for the sway speed
        r=0     #initial condition for angular speed around z - yaw motion
        psi=0   #initial condition for the psi angle
        us=[]
        vs=[]
        rs=[]
        psis=[]
        u_ds=[]
        for t in np.linspace(0,self.t_final,int(self.t_final/self.dt)):
            #print(55555)

            u_d= self.u_d(t)
            v_d= self.v_d(t)
            psi_d= self.psi_d(t)

            eta_u= self.eta_u(u,u_d,psi,psi_d)
            eta_v= self.eta_v(v,v_d,psi,psi_d)
            eta_psi= self.eta_psi(v,v_d,psi,psi_d)
            
            tau_x= self.tau_x(eta_u,u)
            tau_y= self.tau_y(eta_v,v)
            tau_N= self.tau_N(eta_psi,r)

            tau_L,tau_R,tau_M=self.thruster_forces(tau_x,tau_y,tau_N)

            pwm_L=self.pwm_L(tau_L)
            pwm_R=self.pwm_R(tau_R)
            pwm_M=self.pwm_M(tau_M)

            u,v,r,psi=self.jellyfishbot(t,pwm_L,pwm_R,pwm_M)
            us.append(u)
            vs.append(v)
            rs.append(r)
            psis.append(psi)
            u_ds.append(u_d)
        plt.plot(us)
        plt.plot(u_ds)
        plt.show()
    
    def test_hehe(self):
        ##5print(f"I got it! >> {u},{v},{r},{psi}")
       
        #5print("I got them all! {},{},{},{},{},{}".format(self.x,self.y,self.psi,self.u,self.v,self.r))
        return 0


    def log_current_velocity(self,u,v,r):
        self.u=u
        self.v=v
        self.r=r
  
    def log_current_heading(self,psi):
        self.psi=psi
    
    def log_current_location(self,x,y):
        self.x=x
        self.y=y





    def obtain__thruster_commands_LOS_FOSSEN(self,u_d,v_d,ax):  #simple steering with speed control
        # East  E: x-direction
        # North N: y-direction
        

        #Xp = math.atan2(self.x_k1-self.x_k,self.y_k1-self.y_k) #path tangenial angle
        #Xp = math.atan2(self.y_k1-self.y_k,self.x_k1-self.x_k)
        Yp = math.pi/5
        delta = 2
        #Xp = math.atan((self.y_k1-self.y_k)/(self.x_k1-self.x_k))
        
        
        
        #5print("Yp = {}".format(math.tan(Yp)))
        rx = self.x
        ry = self.y
        
        #5print ("rx,ry = {},{}".format(rx,ry))
        
        sol_dict = self.projected_point(Yp,rx,ry,0)
        a,b = symbols('a b')
        yp = sol_dict[a]
        xp = sol_dict[b]
        #5print ("yp,xp = {},{}".format(yp,xp))
        #print('yp = {}'.format(yp))s
        #print('xp = {}'.format(xp))


        ###ye = -(self.x-xp)*math.sin(Yp)+(self.y-yp)*math.cos(Yp)

        #ye = -(self.y-yp)*math.sin(Yp)+(self.x-xp)*math.cos(Yp)
        #ye = math.sqrt((self.y-yp)**2+(self.x-xp)**2)
        ye = -(self.y-yp)*math.sin(Yp)+(self.x-xp)*math.cos(Yp)
        ye = -ye
        self.ye = ye
        self.e_integral = self.e_integral + ye*self.dt*2
        Xr = math.atan(-self.K_P*ye-self.K_I*self.e_integral)
        #5print(Xr)
        #e = -(self.x-self.x_k)*math.sin(Xp)+(self.y-self.y_k)*math.cos(Xp)
        
        #5print("ye = {}".format(ye))

        #5print("atan(ye/delta) = {}".format(math.atan(ye/delta)))
        #print("Beta = {}".format(math.atan(self.v/u_d)))
        
        #psi_d = Yp  - math.atan(ye/delta) - math.atan(-self.v/u_d)
        psi_d = math.tan(Yp) #+ Xr - math.atan(self.v/u_d)

        if psi_d < -0.5*math.pi or self.psi < -0.5*math.pi or psi_d > 0.5*math.pi or self.psi > 0.5*math.pi:
            if psi_d < 0:
                psi_d = psi_d +2*math.pi
            
            if self.psi < 0:
                self.psi = self.psi + 2*math.pi
        self.psi_d=psi_d


        ####
        e_psi_candidate = psi_d - self.psi
        
        if e_psi_candidate > math.pi:
            e_psi_candidate = e_psi_candidate -math.pi
        elif e_psi_candidate < -math.pi:
            e_psi_candidate = e_psi_candidate + math.pi
        self.e_psi = e_psi_candidate
        
        ###

   

        
        #5print("e_psi = {}".format(self.e_psi))
        #if self.e_psi >= 0 and self.e_psi < 3.14 or self.e_psi < -3.14 and self.e_psi > -2*3.14 or self.e_psi > 2*3.14:
        if self.e_psi >= 0:
            #MOVE RIGHT
            self.movement = "RIGHT"
        elif self.e_psi < 0:
            #MOVE LEFT*
            self.movement = "LEFT"

        eta_u= self.eta_u(self.u,u_d,self.psi,psi_d)
        tau_x = self.tau_x(eta_u,self.u)
        tau_y = 0
        eta_psi = self.eta_psi(self.v,v_d,self.psi,psi_d)
        tau_N = self.tau_N(eta_psi,self.r)

        tau_L,tau_R,tau_M = self.thruster_forcesV2(tau_x,tau_y,tau_N)
        if tau_R <= 1.1*tau_L and  tau_R >= 0.9*tau_L:
            # MOVE FORWARD
            self.movement = "FORWARD"
        
        if self.stop != True:
            tau_L,tau_R,tau_M = round(tau_L,1) ,round(tau_R,1) ,round(tau_M,1)
        else:
            tau_L,tau_R,tau_M = 0,0,0
        #pwm_L=self.pwm_L(tau_L)
        #pwm_R=self.pwm_R(tau_R)
        #pwm_M=self.pwm_M(tau_M)
        #print("forces: {},{},{}".format(tau_x,tau_y,tau_N))
        #print("thrusters: {},{},{}".format(tau_L,tau_R,tau_M))
        return tau_L,tau_R,tau_M
        #u,v,r,psi=self.jellyfishbot(t,pwm_L,pwm_R,pwm_M)

    def obtain__thruster_commands_LOS_Virtual_target(self,u_d,v_d):  #simple steering with speed control
        
        Xp = math.atan2(self.yd-self.y,self.xd-self.x)
        #Xp = math.atan2(self.yd-self.yp,self.xd-self.xp)
        print ("x,y: {},{} xd,yd: {},{}".format(self.x,self.y,self.xd,self.yd))
        print("xp,yp: {},{}".format(self.xp,self.yp))
        e = -(self.x-self.xd)*math.sin(Xp)+(self.y-self.yd)*math.cos(Xp)
        #e = -(self.x-self.xp)*math.sin(Xp)+(self.y-self.yp)*math.cos(Xp)


        self.e_integral = self.e_integral + e*self.dt
        Xr = math.atan(-self.K_P*e-self.K_I*self.e_integral)
        # if self.enable_print:
        #     print("Xp = {}".format(Xp))


        
        psi_d = Xp + Xr# - math.atan(self.v/u_d)
        print ("Xp: {}, Xr: {}".format(Xp,Xr))
        print ("psi_d before: {}".format(psi_d))
        #print("Xp, e, Xr: {},{},{}".format(round(Xp,2),round(e,2),round(Xr,2)))
        
        #psi_d = Yp  - math.atan(ye/delta) - math.atan(-self.v/u_d)
        #print("x,y: {},{}".format(self.x,self.y))
        if psi_d < -0.5*math.pi or self.psi < -0.5*math.pi or psi_d > 0.5*math.pi or self.psi > 0.5*math.pi:
            if psi_d < 0:
                psi_d = psi_d +2*math.pi
            
            if self.psi < 0:
                self.psi = self.psi + 2*math.pi
        

        self.psi_d=psi_d
        #print(psi_d)
        
        e_psi_candidate = psi_d - self.psi
        print ("psi: {}, psi_d: {}".format(self.psi,psi_d))
        print ("e_psi_candidate before: {}".format(e_psi_candidate))
        if e_psi_candidate > math.pi:
            e_psi_candidate = e_psi_candidate -math.pi
        elif e_psi_candidate < -math.pi:
            e_psi_candidate = e_psi_candidate + math.pi
        self.e_psi = e_psi_candidate
        #e_psi_candidate = e_psi_candidate - (math.ceil((e_psi_candidate + math.pi/2) / math.pi) - 1) * math.pi

        #if e_psi_candidate <= math.pi and e_psi_candidate >= -math.pi:
        #    self.e_psi = e_psi_candidate
        #else:
        #    if psi_d != 0:
        #        self.e_psi = np.sign(psi_d)*psi_d - self.psi
        #    else:
        #        self.e_psi = psi_d - self.psi
        
        if self.e_psi >= 0:
            #MOVE RIGHT
            self.movement = "RIGHT"
        elif self.e_psi < 0:
            #MOVE LEFT*
            self.movement = "LEFT"
        #print("u_d = {}".format(u_d))
        eta_u= self.eta_u(self.u,u_d,self.psi,psi_d)
        tau_x = self.tau_x(eta_u,self.u)

        
        eta_v= self.eta_v(self.v,v_d,self.psi,psi_d)
        tau_y = self.tau_y(eta_v,self.v)


        #tau_y = 0
        eta_psi = self.eta_psi(self.v,self.v_d,self.psi,psi_d)
        tau_N = self.tau_N(eta_psi,self.r)
        print ("Calculated etas: {} , {} , {}".format(eta_u,eta_v,eta_psi))
        print ("taus required: {} , {} , {}".format(tau_x,tau_y,tau_N))

        tau_L,tau_R,tau_M = self.thruster_forcesV2(tau_x,tau_y,tau_N)
        #tau_L,tau_R,tau_M = self.clip_thrusters(tau_L,tau_R,tau_M )
        print ("Calculated taus: {} , {} , {}".format(tau_L,tau_R,tau_M))
        if tau_R <= 1.1*tau_L and  tau_R >= 0.9*tau_L:
            # MOVE FORWARD
            self.movement = "FORWARD"
        
        if self.stop != True:
            tau_L,tau_R,tau_M = round(tau_L,1) ,round(tau_R,1) ,round(tau_M,1)
        else:
            tau_L,tau_R,tau_M = 0,0,0
        #pwm_L=self.pwm_L(tau_L)
        #pwm_R=self.pwm_R(tau_R)
        #pwm_M=self.pwm_M(tau_M)
        #print("forces: {},{},{}".format(tau_x,tau_y,tau_N))
        #print("thrusters: {},{},{}".format(tau_L,tau_R,tau_M))
        return tau_L,tau_R,tau_M
        #u,v,r,psi=self.jellyfishbot(t,pwm_L,pwm_R,pwm_M)
    def obtain__thruster_commands_LOS_FOSSEN(self,u_d):  #simple steering with speed control
 
        #Yp = math.atan2(self.yd-self.y,self.xd-self.x)
        Yp = math.atan2(self.yd-self.yp,self.xd-self.xp)
        #ye = -(self.x-self.xd)*math.sin(Yp)+(self.y-self.yd)*math.cos(Yp)
        ye = -(self.x-self.xp)*math.sin(Yp)+(self.y-self.yp)*math.cos(Yp)
 
        look_ahead_distance = 0.5

        
        psi_d = Yp  - math.atan(self.v/u_d) - math.atan(ye/look_ahead_distance)
        #print("Xp, e, Xr: {},{},{}".format(round(Xp,2),round(e,2),round(Xr,2)))
        
        #psi_d = Yp  - math.atan(ye/delta) - math.atan(-self.v/u_d)
        #print("x,y: {},{}".format(self.x,self.y))
        if psi_d < -0.5*math.pi or self.psi < -0.5*math.pi or psi_d > 0.5*math.pi or self.psi > 0.5*math.pi:
            if psi_d < 0:
                psi_d = psi_d +2*math.pi
            
            if self.psi < 0:
                self.psi = self.psi + 2*math.pi
        

        self.psi_d=psi_d
        #print(psi_d)
        
        e_psi_candidate = psi_d - self.psi
        if e_psi_candidate > math.pi:
            e_psi_candidate = e_psi_candidate -math.pi
        elif e_psi_candidate < -math.pi:
            e_psi_candidate = e_psi_candidate + math.pi
        self.e_psi = e_psi_candidate
        #e_psi_candidate = e_psi_candidate - (math.ceil((e_psi_candidate + math.pi/2) / math.pi) - 1) * math.pi

        #if e_psi_candidate <= math.pi and e_psi_candidate >= -math.pi:
        #    self.e_psi = e_psi_candidate
        #else:
        #    if psi_d != 0:
        #        self.e_psi = np.sign(psi_d)*psi_d - self.psi
        #    else:
        #        self.e_psi = psi_d - self.psi
        
        if self.e_psi >= 0:
            #MOVE RIGHT
            self.movement = "RIGHT"
        elif self.e_psi < 0:
            #MOVE LEFT*
            self.movement = "LEFT"
        #print("u_d = {}".format(u_d))
        eta_u= self.eta_u(self.u,u_d,self.psi,psi_d)
        tau_x = self.tau_x(eta_u,self.u)
        tau_y = 0
        eta_psi = self.eta_psi(self.v,self.v_d,self.psi,psi_d)
        tau_N = self.tau_N(eta_psi,self.r)
        #print("eta_u = {}".format(eta_u))
        tau_L,tau_R,tau_M = self.thruster_forcesV2(tau_x,tau_y,tau_N,clipping = True)
        #tau_L,tau_R,tau_M = self.clip_thrusters(tau_L,tau_R,tau_M )
        
        if tau_R <= 1.1*tau_L and  tau_R >= 0.9*tau_L:
            # MOVE FORWARD
            self.movement = "FORWARD"
        
        if self.stop != True:
            tau_L,tau_R,tau_M = round(tau_L,1) ,round(tau_R,1) ,round(tau_M,1)
        else:
            tau_L,tau_R,tau_M = 0,0,0
        #pwm_L=self.pwm_L(tau_L)
        #pwm_R=self.pwm_R(tau_R)
        #pwm_M=self.pwm_M(tau_M)
        #print("forces: {},{},{}".format(tau_x,tau_y,tau_N))
        #print("thrusters: {},{},{}".format(tau_L,tau_R,tau_M))
        return tau_L,tau_R,tau_M
        #u,v,r,psi=self.jellyfishbot(t,pwm_L,pwm_R,pwm_M)


