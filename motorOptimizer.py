import numpy as np
print("started")
#Torque: .3098, .5163, .9300, .9588, 1.475, 1.623, 1.770, 1.844, 2.213, 6.269
#weight : .6400, 1.160, 1.500, 1.800, 2.430, 2.670, 2.646, 3.340, 3.780, 7.937
t1max = 6.269*12 #10 best
t2max, M2 = 2.213*12, 3.780 #9 best
t3max, M3 = 0.3098*12, .6400 #1 best
t4max, M4 = 0.3098*12, 0.64 #1 best
M5 = 0.12 #theoretical dynamixel ax-12a
tmax = np.array([t1max,t2max,t3max,t4max])
P = 0
bestLengths = np.array([0,0,0,0,0])
bestTorque = 0
t1Remaining = 0
t2Remaining = 0
t3Remaining = 0
t4Remaining = 0
remainingTorque = np.array([t1Remaining,t2Remaining,t3Remaining,t4Remaining])
bestTorqueSet = remainingTorque
minRemaining = 100000
counter = 0
for l1 in np.arange(0, 6.1, 0.1):
    for l2 in np.arange(4, 8.1, 0.1):
        for l3 in np.arange(2, 8.1, 0.1):
            for l5 in np.arange(3, 4.1, 0.1):
                counter += 1
                if counter % 10000000 == 0:
                    print("looped 10mil")
                if l1 + l2 + l3 + l5 >= 23.95 and l1 + l2 + l3 + l5<= 24.05:
                    for l6 in np.arange(0, 3.1, 0.1):
                        #calculate torques
                        t1 = (M2*l2)+((M3+M4)*(l2+l3))+((M5+P)*(l2+l3+l5))
                        t2 = ((M3+M4)*l3)+((M5+P)*(l3+l5))
                        t3 = (M5+P)*l5
                        t4 = P*l6
                        t = np.array([t1,t2,t3,t4])
                        
                        #check remaining available torque
                        remainingTorque = np.array([t1max-t1,t2max-t2,t3max-t3,t4max-t4])
                        if np.any(remainingTorque<=0):
                            #print("nope")
                            break
                        minRemaining = 100000
                        for i in range(4):
                            if remainingTorque[i] < minRemaining:
                                minRemaining = remainingTorque[i]
                                index = i
                        if remainingTorque[0]>bestTorque: #minRemaining
                            bestTorque = minRemaining
                            bestLengths = np.array([l1,l2,l3,l5,l6])
                            bestTorqueSet = remainingTorque    
                            strugglingMotor = index
                            #print("torque updated")     
print(bestTorque/12, "- Worst Torque Final")
print("best lengths: ", bestLengths)
print("best torques: ", bestTorqueSet/12)
print(counter, " combinations checked")


maxPayLoad = 0
l1, l2, l3, l5, l6 = bestLengths
while np.all((bestTorqueSet>=0)):
    P = maxPayLoad
    t1 = (M2*l2)+((M3+M4)*(l2+l3))+((M5+P)*(l2+l3+l5))
    t2 = ((M3+M4)*l3)+((M5+P)*(l3+l5))
    t3 = (M5+P)*l5
    t4 = P*l6
    t = np.array([t1,t2,t3,t4])
    bestTorqueSet = np.array([t1max-t1,t2max-t2,t3max-t3,t4max-t4])
    maxPayLoad += 0.01

print("max payload: ", round((maxPayLoad-0.02), 4))
P = maxPayLoad - 0.02
t1 = (M2*l2)+((M3+M4)*(l2+l3))+((M5+P)*(l2+l3+l5))
t2 = ((M3+M4)*l3)+((M5+P)*(l3+l5))
t3 = (M5+P)*l5
t4 = P*l6
t = np.array([t1,t2,t3,t4])
bestTorqueSet = np.array([t1max-t1,t2max-t2,t3max-t3,t4max-t4])/12
print("torque left: ", bestTorqueSet)
print("strugling motor: ",np.argmin(bestTorqueSet)+1)