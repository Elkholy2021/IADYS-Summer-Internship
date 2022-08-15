from xml.etree.ElementTree import XMLID
from xml.sax import xmlreader
from sympy import symbols, Eq, solve
import math


def projected_point(Yp,rx,ry,ax):
    a,b = symbols('a b')
    eq1 = Eq(-1/math.tan(Yp)*(ry-a)+b-rx)
    eq2 = Eq(math.tan(Yp)*a+ax-b)

    sol_dict = solve((eq1,eq2), (a,b))
    return sol_dict



Yp = math.pi/4
ry = 1
rx = 10

ax = 10
sol_dict = projected_point(Yp,rx,ry,ax)
a,b = symbols('a b')
yp = sol_dict[a]
xp = sol_dict[b]
print('yp = {}'.format(yp))
print('xp = {}'.format(xp))


delta = 1
m = math.tan(Yp)
a = (1+m**2)
b = -2*yp -2*m*xp
c = yp**2 +xp**2 -delta**2
yd = (-b+math.sqrt(b**2-4*a*c))/(2*a)
xd = m*yd
print('yp2 = {}'.format(yd))
print('xp2 = {}'.format(xd))
# print('yp2 = {}'.format(yp+0.5))
# print('xp2 = {}'.format(math.tan(Yp)*(yp+0.5)))

print("ye - FOSSEN = {}".format(-(rx-xp)*math.sin(Yp)+(ry-yp)*math.cos(Yp)))
print("ye - FOSSEN2 = {}".format(-(ry-yp)*math.sin(Yp)+(rx-xp)*math.cos(Yp)))
print("ye - HASSAN = {}".format(math.sqrt((rx-xp)**2+(ry-yp)**2)))
