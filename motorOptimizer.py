import numpy as np

names = ["Twotrees Nema 17", "SOULUCK Nema 23 A", "STEPPERONLINE CNC Nema 23", "SOULUCK Nema 23 B", "SOULUCK Nema 23 C", "SOULUCK Nema 23 D", "STEPPERONLINE Nema 23", "SOULUCK Nema 23 E", "SOULUCK Nema 23 F", "HobbyUnlimited Nema 34"]
torques = [0.3098, 0.5163, 0.93, 0.9588, 1.4751, 1.6226, 1.7701, 1.8439, 2.2127, 6.2693] #ft lb
weights = [0.64, 1.16, 1.5, 1.68, 2.43, 2.67, 2.64555, 3.34, 3.78, 7.93664] #lb
costs = [8.00, 19.00, 26.00, 20.00, 25.00, 26.00, 26.00, 37.00, 40.00, 58.00] #USD

#Selecting motors for simulation
def set_motors(t1, t2, t3, t4):
    #t1 = int(input("Motor 1: "))
    #t2 = int(input("Motor 2: "))
    #t3 = int(input("Motor 3: "))
    #t4 = int(input("Motor 4: "))
    t1max, t2max, t3max, t4max = torques[t1 - 1]*12, torques[t2-1]*12, torques[t3-1]*12, torques[t4-1]*12 #lb in
    M1, M2, M3, M4, M5 = weights[t1-1], weights[t2-1], weights[t3-1], weights[t4-1], 0.24 #Last one is theoretical dynamixel ax-12a (360 rotational servo)
    tmax = np.array([t1max,t2max,t3max,t4max])
    masses = [M2, M3, M4, M5]
    return tmax, masses

def find_lengths(tmax, masses):
    #Determines how to calculate the best motor set
    method = int(input("Calculation method: (1) Torque percentage (2) Max Payload: "))
    while (method != 1) and (method != 2):
        method = int(input("Please enter 1 or 2: "))
    #Initializing variables
    P = 0 #in lb
    bestLengths = np.array([0,0,0,0,0]) #in
    bestTorque = 0 #%
    remainingTorque = np.array([0,0,0,0]) #in lb
    bestTorques = remainingTorque
    counter = 0 #number
    bestMaxPayload = 0 #lb
    strugglingMotor = 0
    loadedRemainingTorques = np.array([0,0,0,0]) #in lb
    failingMotor = 0
    for l1 in np.arange(0, 6.1, 0.1):
        for l2 in np.arange(4, 8.1, 0.1):
            for l3 in np.arange(2, 8.1, 0.1):
                l5 = 24 - (l1+l2+l3)
                if 0 < l5 < 4:
                    for l6 in np.arange(0, 3.1, 0.1):
                        counter += 1
                        if counter % 10000000 == 0:
                            print("looped 10mil")
                        #calculate torques
                        lengths = [l1, l2, l3, l5, l6]
                        t = calc_torques(lengths, masses, 0) #This is in lb in
                        #Check remaining available torque
                        remainingTorque = tmax-t
                        remainingTorquePercentage = remainingTorque/tmax
                        #If any of the motors can't handle the weight, we cannot use the configuration. We can change this threshold to be more selective
                        if np.any(remainingTorquePercentage<=0):
                            break
                        #Below are two methods of finding our set. The first is finding which set leads to the best worst percentage of torque usage
                        #The second finds which set leads to the best payload before failure
                        if method == 1:
                            minRemaining = np.min(remainingTorquePercentage)
                            if minRemaining > bestTorque:
                                #Total:
                                bestLengths = lengths
                                #Pre-Payload:
                                bestTorque = minRemaining
                                bestTorques = remainingTorque
                                strugglingMotor = np.argmin(remainingTorquePercentage)+1
                                #Post-Payload:
                                bestMaxPayload, loadedRemainingTorques, failingMotor = find_max_payload(bestLengths, masses, tmax) 
                        elif method == 2:                        
                            if find_max_payload(lengths, masses, tmax)[0]>bestMaxPayload:
                                #Total:
                                bestLengths = lengths #in
                                #Pre-Payload:
                                bestTorque = np.min(remainingTorquePercentage) #%
                                bestTorques = remainingTorque #in lb
                                strugglingMotor = np.argmin(remainingTorquePercentage)+1 #index This could also be struggling by percentage if changed
                                #Post-Payload:
                                bestMaxPayload, loadedRemainingTorques, failingMotor = find_max_payload(bestLengths, masses, tmax) #lbs, in lb, index
                        
    print(counter, " combinations checked")
    return bestLengths, bestTorque, bestTorques, strugglingMotor, bestMaxPayload, loadedRemainingTorques, failingMotor

def find_max_payload(lengths, masses, tmax):
    l1, l2, l3, l5, l6 = lengths
    maxPayload = 0.0
    bestTorqueSet = tmax - calc_torques(lengths, masses, maxPayload)
    while np.all((bestTorqueSet>=0)):
        t = calc_torques(lengths, masses, maxPayload)
        bestTorqueSet = tmax - t
        maxPayload += 0.01
    
    maxPayload -= 0.02 #Technically its -0.01 but it causes a computational error leading to negative loaded torques
    tFinal = calc_torques(lengths, masses, maxPayload)
    remainingTorque = tmax - tFinal
    failingMotor = np.argmin(remainingTorque)+1
    return maxPayload, remainingTorque, failingMotor

def calc_torques(lengths, masses, payload):
    M2, M3, M4, M5 = masses
    l1, l2, l3, l5, l6 = lengths
    P = payload
    t1 = (M2*l2)+((M3+M4)*(l2+l3))+((M5+P)*(l2+l3+l5))
    t2 = ((M3+M4)*l3)+((M5+P)*(l3+l5))
    t3 = (M5+P)*l5
    t4 = P*l6
    t = np.array([t1,t2,t3,t4])
    return t

tmax, masses = set_motors(10, 9, 2, 1)
bestLengths, bestTorque, bestTorqueSet, strugglingMotor, bestMaxPayload, loadedRemainingTorques, failingMotor = find_lengths(tmax, masses)

print("PREPAYLOAD: \nWorst Torque Final (%): ", bestTorque*100)
print("Best Lengths (in): ", [float(x) for x in bestLengths])
print("Best Torques (ft lb): ", bestTorqueSet/12)
print("Best Torques (%): ", 100*bestTorqueSet/tmax)
print("Struggling Motor: ", strugglingMotor)
print("\n\n")
print("WITH PAYLOAD: \nMax Payload: ", round((bestMaxPayload), 4))
print("Max-loaded Torque (ft lb): ", loadedRemainingTorques/12)
print("Max-loaded Torque (%): ", (loadedRemainingTorques/tmax)*100)
print("Failed Motor: ",failingMotor)