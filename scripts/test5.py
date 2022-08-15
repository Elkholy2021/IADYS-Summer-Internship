from scipy.optimize import fsolve
import math
import numpy as np
m = math.tan(math.pi/2)
xp = 0
yp = 1
delta = 1

a = (1+m**2)
b = -2*yp -2*m*xp
c = yp**2 +xp**2 -delta**2
a,b,c = 328280.968335,-13119690.201,131083305.009

print(b**2-4*a*c)
yd = (-b+math.sqrt(b**2-4*a*c))/(2*a)
xd = m*yd
print("yd,xd = {},{}".format(yd,xd))


from scipy.optimize import fsolve
Yp = math.pi*0.5
ry = -3.01403999172e-05
rx = -6.98165547231e-06

ax = 0
def func(x):
    return [-1/math.tan(Yp)*(ry-x[0])+x[1]-rx,
            math.tan(Yp)*x[0]+ax-x[1]]
root = fsolve(func, [0.1, 0.1])
yp = root[0]
xp = root[1]
print('yp = {}'.format(yp))
print('xp = {}'.format(xp))