import numpy as np
import pandas as pd

#MOTORS OF INTEREST
MOTORS_OF_INTEREST = (1, 1, 14, 9, 9)
#Defining the motors, torques, weights, and costs in corresponding arrays, then assigning them to an overarching dataframe
names = ["Twotrees Nema 17", "Twotrees Nema17 w20:1 gearing", "SOULUCK Nema 23 A", "STEPPERONLINE CNC Nema 23", "SOULUCK Nema 23 B", "SOULUCK Nema 23 C", "SOULUCK Nema 23 D", "STEPPERONLINE Nema 23", "SOULUCK Nema 23 E", "SOULUCK Nema 23 F", "HobbyUnlimited Nema 34", "FILFEEL Micro Gearmotor", "Greartisan 12V DC Motor 100RPM", "Greartisan 12V DC Motor 120RPM", "Greartisan 12V DC Motor 200RPM", "Greartisan 12V DC Motor 60RPM", "Fafeicy 12V DC Motor", "Greartsian 12V 30 RPM"]
torques = [0.3098, (0.3098*20) ,0.5163, 0.93, 0.9588, 1.4751, 1.6226, 1.7701, 1.8439, 2.2127, 6.2693, 0.1627, 0.2604, 0.4701, 0.1591, 0.4701, 0.405, 0.867961662145188] #ft lb
weights = [0.64, (0.64+0.71), 1.16, 1.5, 1.68, 2.43, 2.67, 2.64555, 3.34, 3.78, 7.93664, 0.02866, 0.4875, 0.4544, 0.4631, 0.4606, 0.3638, 0.675] #lb
costs = [8.00, 32.34, 19.00, 26.00, 20.00, 25.00, 26.00, 26.00, 37.00, 40.00, 58.00, 8.75, 15, 15, 15, 15, 15.7, 14.99] #USD
motors = pd.DataFrame({'Name': names, 'Torque': torques, 'Weight': weights, 'Cost': costs})

density = 0.4/12 #lbs per inch of 80X20

#Sorting the dataframe by motor Torque in decreasing order (i.e. highest Torque is first)
sortedMotors = motors.sort_values(by='Torque', ascending=False).reset_index(drop=True)
sortedMotors.index.name = 'Index'
print(sortedMotors)

#Selecting motors for simulation. Intakes motors from global list and outputs masses and max torques
def set_motors(MOTORS_OF_INTEREST, motorDataframe):
    m2, m3, m4, m5, m6 = MOTORS_OF_INTEREST
    t2max, t3max, t4max, t5max = motorDataframe['Torque'].iat[m2]*12, motorDataframe['Torque'].iat[m3]*12, motorDataframe['Torque'].iat[m4]*12, motorDataframe['Torque'].iat[m5]*12 #lb in
    M2, M3, M4, M5, M6 = motorDataframe['Weight'].iat[m2], motorDataframe['Weight'].iat[m3], motorDataframe['Weight'].iat[m4], motorDataframe['Weight'].iat[m5], motorDataframe['Weight'].iat[m6] #lb
    tmax = np.array([t2max,t3max,t4max, t5max,])
    masses = [M3, M4, M5, M6]
    return tmax, masses

def find_lengths(tmax, masses, density):
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
                    l6 = 1.5
                    counter += 1
                    if counter % 10000000 == 0:
                        print("looped 10mil")
                    #calculate torques
                    lengths = [l1, l2, l3, l5, l6]
                    t = calc_torques(lengths, masses, 0, density) #This is in lb in
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
                            strugglingMotor = np.argmin(remainingTorquePercentage)+2
                            #Post-Payload:
                            bestMaxPayload, loadedRemainingTorques, failingMotor = find_max_payload(bestLengths, masses, tmax, density) 
                    elif method == 2:                        
                        if find_max_payload(lengths, masses, tmax, density)[0]>bestMaxPayload:
                            #Total:
                            bestLengths = lengths #in
                            #Pre-Payload:
                            bestTorque = np.min(remainingTorquePercentage) #%
                            bestTorques = remainingTorque #in lb
                            strugglingMotor = np.argmin(remainingTorquePercentage)+2 #index This could also be struggling by percentage if changed
                            #Post-Payload:
                            bestMaxPayload, loadedRemainingTorques, failingMotor = find_max_payload(bestLengths, masses, tmax, density) #lbs, in lb, index
                        
    print(counter, " combinations checked")
    return bestLengths, bestTorque, bestTorques, strugglingMotor, bestMaxPayload, loadedRemainingTorques, failingMotor

def find_max_payload(lengths, masses, tmax, density):
    l1, l2, l3, l5, l6 = lengths
    maxPayload = 0.0
    bestTorqueSet = tmax - calc_torques(lengths, masses, maxPayload, density)
    while np.all((bestTorqueSet>=0)):
        t = calc_torques(lengths, masses, maxPayload, density)
        bestTorqueSet = tmax - t
        maxPayload += 0.01
    
    maxPayload -= 0.02 #Technically its -0.01 but it causes a computational error leading to negative loaded torques
    tFinal = calc_torques(lengths, masses, maxPayload, density)
    remainingTorque = tmax - tFinal
    failingMotor = np.argmin(remainingTorque)+2
    return maxPayload, remainingTorque, failingMotor

def calc_torques(lengths, masses, payload, density):
    d = density
    M3, M4, M5, M6 = masses
    l1, l2, l3, l5, l6 = lengths
    P = payload
    t1 = (M3*l2)+((M5+M4)*(l2+l3))+((M6+P)*(l2+l3+l5))+(d*l2*(l2/2))+(d*(l3)*(l2+(l3)/2))+(d*4*(l2+l3))+(d*l5*(l2+l3+(l5/2))) #IF L4 CHANGES, CHANGE THIS
    t2 = ((M5+M4)*l3)+((M6+P)*(l3+l5))+(d*l3*(l3/2))+(d*4*l3)+(d*l5*(l3+(l5/2)))
    t3 = ((M6+P)*l5)+(d*l5*(l5/2))
    t4 = P*l6
    t = np.array([t1,t2,t3,t4])
    return t


#HERE WE INPUT THE TUPLE OF MOTORS WE ARE INTERESTED IN
tmax, masses = set_motors(MOTORS_OF_INTEREST, sortedMotors)
bestLengths, bestTorque, bestTorqueSet, strugglingMotor, bestMaxPayload, loadedRemainingTorques, failingMotor = find_lengths(tmax, masses, density)

print("PREPAYLOAD: \nWorst Torque (%): ", bestTorque*100)#WORST torque from the BEST set of lengths
printedMotorNumber = ["#2", "#3", "#4", "#5"]
linkageNumber = ["1st","2nd","3rd","4th","5th"]

result = ""
for name, item in zip(linkageNumber, bestLengths):
    result += f"{name} length: {item:.2f}, "
print("Best Lengths (in): ", result)

result = ""
for name, item in zip(printedMotorNumber, bestTorqueSet/12):
    result += f"{name} torque: {item:.2f}, "
print("Best Torques (ft lb): ", result)
    
result = ""
for name, item in zip(printedMotorNumber, 100*bestTorqueSet/tmax):
    result += f"{name} torque: {item:.2f}%, "
print("Best Torques (%): ", result)


print("Struggling Motor: ", strugglingMotor)
print("\n\n")
print("WITH PAYLOAD: \nMax Payload: ", round((bestMaxPayload), 4))

result = ""
for name, item in zip(printedMotorNumber, loadedRemainingTorques/12):
    result += f"{name} torque: {item:.6f}, "
print("Leftover Torque (lb*ft): ", result)

result = ""
for name, item in zip(printedMotorNumber, (loadedRemainingTorques/tmax)*100):
    result += f"{name} torque: {item:.2f}%, "
print("Leftover Torque (%): ", result)

print("Failed Motor: ",failingMotor)