import numpy as np
import math 
import matplotlib.pyplot as plt




class path_generation():
    def __init__(self):
        self.W = 20
        self.L = 8
        self.R = 3
        self.x0 = 2
        self.y0 = 2 
        self.alpha = 0
        self.overlap = False
        #self.pointsX = [2 , 10, 10, 2]
        #self.pointsY = [2 , 2 , 5 , 5]
        self.pointsX = [0]
        self.pointsY = [0]
        self.Npoints = 0

        

    def enable_overlap(self):
        self.overlap = True



    def generate_points(self):
        # if self.overlap:
        #     self.points_per_side =math.ceil(self.W/float(self.R))
        # else:
        #     self.points_per_side = math.floor(self.W/self.R)
        self.points_per_side = math.floor(self.W/self.R)

        self.x = self.x0
        self.y = self.y0
        other_side = True
        # print("Number of points is {}".format(int(self.points_per_side)*2))
       
        self.points = np.zeros((2, int(self.points_per_side)*2+1)) #add 1 here to add initial position
        self.points[0,0] = 0
        self.points[1,0] = 0
        for i in range(int(self.points_per_side)*2):
            self.pointsX.append(self.x)
            self.pointsY.append(self.y)
            self.points[0,i+1] = self.x
            self.points[1,i+1] = self.y
            if other_side == True:
                if self.x == self.x0+self.L:
                    self.x = self.x - self.L
                    self.y = self.y
                    other_side = False
                else:
                    self.x = self.x + self.L
                    self.y = self.y
                    other_side = False
            else:
                self.x = self.x
                self.y = self.y + float(self.W)/self.points_per_side
                other_side = True
        
        #self.points = np.transpose(self.points)
        self.alpha = math.radians(self.alpha)
        self.R = np.array([[math.cos(self.alpha) ,-math.sin(self.alpha)], [math.sin(self.alpha),math.cos(self.alpha) ]])
        self.points = np.matmul(self.R,self.points)
        self.points = self.points - np.array([[self.points[0,0]-self.x0], [self.points[1,0]-self.y0]])
       # print(self.points)
        #print(self.points.shape)
        self.Npoints = len(self.pointsX)
        return self.points
    def plot_path0(self):
        plt.plot(self.pointsY[0],self.pointsX[0],'X')
        plt.plot(self.pointsY[1:-1],self.pointsX[1:-1],'*')
        plt.plot(self.pointsY[-1],self.pointsX[-1],'X')
        plt.plot(self.pointsY,self.pointsX,'-')
        plt.grid()
        plt.show()

    def plot_path(self):
        plt.plot(self.points[1,0],self.points[0,0],'X')
        plt.plot(self.points[1,1: -1],self.points[0,1:-1],'*')
        plt.plot(self.points[1,-1],self.points[0,-1],'X')
        plt.plot(self.points[1,:],self.points[0,:],'-')
        plt.grid()
        plt.show()
        

