# Point coordinatesProjectionPointOnLine(double x /**< [in] coordinate x of p */, double y /**< [in] coordinate y of p */, Point A, Point B)
# {
#   /*returns coordinates of projection on line*/

#   Point p = {0, 0};
from select import select


a = self.x - self.ay
b = self.y - self.ax
c = self.xf - self.ay
d = self.yf - self.ax
#   double a = x - A.x;
#   double b = y - A.y;
#   double c = B.x - A.x;
#   double d = B.y - A.y;

dot = a*c + b*d
len_sq = c * c + d * d
param = -1
if (len_sq != 0) #// in case of 0 length line
    param = dot / len_sq
#   double dot = a * c + b * d;
#   double len_sq = c * c + d * d;
#   double param = -1;
#   if (len_sq != 0) // in case of 0 length line
#     param = dot / len_sq;

#   double xx, yy;
if param < 0:
    xx = self.ay
    yy = self.ax
elif param > 1:
    xx = self.xf
    yy = self.yf
else:
    xx = self.ay + param * c
    yy = self.ax + param *d

print("xp,yp = {}".format(xx,yy))
#   if (param < 0)
#   {
#     xx = A.x;
#     yy = A.y;
#   }
#   else if (param > 1)
#   {
#     xx = B.x;
#     yy = B.y;
#   }
#   else
#   {
#     xx = A.x + param * c;
#     yy = A.y + param * d;
#   }

#   p.x = xx;
#   p.y = yy;

#   return p;
# }