import math
from sympy import symbols, Eq, solve


yp = 0
xp = 0
Yp =0.5*math.pi*0.99
m = math.tan(Yp)
delta = 1
yd0 = symbols('yd0')
eq1 = Eq(((yd0-yp)**2+(m*yd0-xp)**2)-delta**2)
sol_dict = solve((eq1), (yd0)) 
yd1 = sol_dict[0]
yd2 = sol_dict[1]
if m >= 0:
    yd = max(yd1,yd2)
else:
    yd = min(yd1,yd2)
xd = m*yd
print("yd,xd = {},{}".format(yd,xd))