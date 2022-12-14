import csv
import matplotlib.pyplot as plt
import numpy as np
import sys
Xs=[]
Ys=[]
XPs=[]
YPs=[]
XDs=[]
YDs=[]
Yaws=[]
#fig, axs = plt.subplots(2)

#csv_name=str(sys.argv[-1])
#csv_name='posebag2.csv'
csv_name=str(sys.argv[-1])

c = 0
#if csv_name.split('.')[0]=='posebag2':
    
with open(csv_name, 'r') as file:
    reader = csv.reader(file)
    for row in reader:
        if c > 0:

    
        
            Xs.append(float(row[1]))
            Ys.append(float(row[2]))
            XPs.append(float(row[4]))
            YPs.append(float(row[5]))
            Yaws.append(float(row[3]))
            XDs.append(float(row[6]))
            YDs.append(float(row[7]))
            
            
        c = c+1
    XPs = np.array(XPs)
    YPs = np.array(YPs)
    XDs = np.array(XDs)
    YDs = np.array(YDs)
    Xs = np.array(Xs)
    Ys = np.array(Ys)
    Yaws = np.array(Yaws)
    MSE = np.sum((XPs-Xs)**2+(YPs-Ys)**2)
    print("MSE = {}".format(MSE))
 
    plt.plot(YPs,XPs,'*')
    #plt.plot(YDs,XDs,'*')
    plt.plot(Ys,Xs,'*')
    plt.axis('equal')   
    # plt.plot(t,x,label='Depth - with filter')
    plt.grid()
    plt.xlabel("x", fontsize=15)
    plt.ylabel("y'", fontsize=15)
    plt.title("Performance: {}, MSE = {}".format(csv_name.split('.')[0],MSE), fontsize=26)  
    #plt.legend(['Projected point', 'Virtual target','Robot position'])

    #plt.rcParams.update({'font.size': 26})
    plt.show()


    plt.plot(Yaws)


    #plt.axis('scaled')   
    plt.ylim(-4, 4)

    plt.grid()
    plt.xlabel("time", fontsize=15)
    plt.ylabel("heading angle", fontsize=15)
    plt.title("Performance: {}".format(csv_name.split('.')[0]), fontsize=26)  
    plt.show()
    

    # print(depth)

