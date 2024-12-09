import numpy as np
import pandas as pd

#Defining the motors, torques, weights, and costs in corresponding arrays, then assigning them to an overarching dataframe
names = ["Twotrees Nema 17", "SOULUCK Nema 23 A", "STEPPERONLINE CNC Nema 23", "SOULUCK Nema 23 B", "SOULUCK Nema 23 C", "SOULUCK Nema 23 D", "STEPPERONLINE Nema 23", "SOULUCK Nema 23 E", "SOULUCK Nema 23 F", "HobbyUnlimited Nema 34", "FILFEEL Micro Gearmotor", "Greartisan 12V DC Motor 100RPM", "Greartisan 12V DC Motor 120RPM", "Greartisan 12V DC Motor 200RPM", "Greartisan 12V DC Motor 60RPM", "Fafeicy 12V DC Motor"]
torques = [0.3098, 0.5163, 0.93, 0.9588, 1.4751, 1.6226, 1.7701, 1.8439, 2.2127, 6.2693, 0.1627, 0.2604, 0.4701, 0.1591, 0.4701, 0.405] #ft lb
weights = [0.64, 1.16, 1.5, 1.68, 2.43, 2.67, 2.64555, 3.34, 3.78, 7.93664, 0.02866, 0.4875, 0.4544, 0.4631, 0.4606, 0.3638] #lb
costs = [8.00, 19.00, 26.00, 20.00, 25.00, 26.00, 26.00, 37.00, 40.00, 58.00, 8.75, 15, 15, 15, 15, 15.7] #USD
motors = pd.DataFrame({'Name': names, 'Torque': torques, 'Weight': weights, 'Cost': costs})

#Sorting the dataframe by motor weight in decreasing order (i.e. heaviest motor is first)
sortedMotors = motors.sort_values(by='Weight', ascending=False)
print(sortedMotors)
#Selecting motors for simulation. Intakes motors from global list and outputs masses and max torques
def set_motors(m1, m2, m3, m4, m5, motorDataframe):
    t1max, t2max, t3max, t4max = motorDataframe['Torque'].iat[m1]*12, motorDataframe['Torque'].iat[m2]*12, motorDataframe['Torque'].iat[m3]*12, motorDataframe['Torque'].iat[m4]*12 #lb in
    M1, M2, M3, M4, M5 = motorDataframe['Weight'].iat[m1], motorDataframe['Weight'].iat[m2], motorDataframe['Weight'].iat[m3], motorDataframe['Weight'].iat[m4], motorDataframe['Weight'].iat[m5] #lb
    tmax = np.array([t1max,t2max,t3max,t4max])
    masses = [M2, M3, M4, M5]
    return tmax, masses

#Intakes max torques and masses initialized by set_motors, and outputs best lengths for the set, as well as max payload and other information
def find_lengths(tmax, masses):
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
    #Looping through all lengths within specified ranges
    for l1 in np.arange(0, 6.1, 0.1):
        for l2 in np.arange(4, 8.1, 0.1):
            for l3 in np.arange(2, 8.1, 0.1):
                #Making sure the sum is 24 for a 2ft radius
                l5 = 24 - (l1+l2+l3)
                #Checking to see that l5 is within a desireable range
                if 0 < l5 < 4:
                    #Looping through final length
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
                        #Finding maximum payload of the current set for comparison
                        currentMaxPayload, currentLoadedRemainingTorques, currentFailingMotor = find_max_payload(lengths, masses, tmax, 0.1)
                        #If the current payload is the best so far, save it and all other pertinent information about the set
                        if currentMaxPayload>bestMaxPayload:
                            #Total:
                            bestLengths = lengths #in
                            #Pre-Payload:
                            bestTorque = np.min(remainingTorquePercentage) #%
                            bestTorques = remainingTorque #in lb
                            strugglingMotor = np.argmin(remainingTorquePercentage)+1 #index This could also be struggling by percentage if changed
                            #Post-Payload:
                            bestMaxPayload, loadedRemainingTorques, failingMotor = find_max_payload(lengths, masses, tmax, 0.01) #lbs, in lb, index
    #Return the best lengths, worst torque percentage, remaining (unloaded) torques, struggling (unloaded) motor, max payload, remaining torques (loaded to max), and failing (loaded) motor
    return bestLengths, bestTorque, bestTorques, strugglingMotor, bestMaxPayload, loadedRemainingTorques, failingMotor

#Intakes lengths, masses, and maximum torques of a motor configuration as well as a search granularity and outputs the maximum payload it can hold
def find_max_payload(lengths, masses, tmax, step):
    maxPayload = 0.0
    bestTorqueSet = tmax - calc_torques(lengths, masses, maxPayload)
    while np.all((bestTorqueSet>=0)):
        t = calc_torques(lengths, masses, maxPayload)
        bestTorqueSet = tmax - t
        maxPayload += step
    
    maxPayload -= 2*step #Technically its -0.01 but it causes a computational error leading to negative loaded torques
    tFinal = calc_torques(lengths, masses, maxPayload)
    remainingTorque = tmax - tFinal
    failingMotor = np.argmin(remainingTorque)+1
    return maxPayload, remainingTorque, failingMotor

#Intakes lengths, masses, and payload of a configuration and outputs current torques used in the motors
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
#Intakes the dataframe of motor information and outputs the best motors along with cost, as well as all motor combinations that supercede the payloadThreshold
def run_motor_combos(motorDataframe, payloadThreshold):
    #Initializing variables
    costs = motorDataframe["Cost"]
    count = 0
    cost = 0
    motor1 = 0
    bestPayload = 0
    bestMotors = [0, 0, 0, 0, 0]
    specSets = pd.DataFrame({})
    specMotors = []
    specNames = []
    specPayloads = []
    specCosts = []
    #Looping through all motor combinations in feasible range
    for motor2 in range(1, 3):
        for motor3 in range(3, 10):
            for motor4 in range(4, 16):
                for motor5 in range(5, 16):
                    motors = [motor1, motor2, motor3, motor4, motor5]
                    #Only analyzing motor combinations in decreasing weight (i.e. motors farther along the arm must be lighter than all previous motors)
                    if sorted(motors)==motors:
                        tmax, masses = set_motors(motor1, motor2, motor3, motor4, motor5, motorDataframe)
                        bestMaxPayload = find_lengths(tmax, masses)[4]
                        #If the max payload for the current set of motors supercedes the specified threshold, add the information to the initialized lists
                        if bestMaxPayload>=payloadThreshold:
                            specMotors.append(motors)
                            specPayloads.append(bestMaxPayload)
                        #If the max payload is the best so far, save the information
                        if round((bestMaxPayload), 4) > bestPayload:
                            bestMotors = motors
                            bestPayload = round((bestMaxPayload), 4)
                        #Progress percentages printed in console. If the numbers in the loop are changed, the denominator of count also should change
                        print(np.round(100*(count/768), decimals=1), "%")
                        print(motors)
                        count+=1
    #For all saved motors over the payload threshold, save the cost of the set and the names of the motors in saved lists
    for set in specMotors:
        cost = costs[set[0]] + costs[set[1]] + costs[set[2]] + costs[set[3]] + costs[set[4]]
        names = [motorDataframe["Name"].iat[set[0]], motorDataframe["Name"].iat[set[1]], motorDataframe["Name"].iat[set[2]], motorDataframe["Name"].iat[set[3]], motorDataframe["Name"].iat[set[4]]]
        specNames.append(names)
        specCosts.append(cost)
    #Creating the dataframe from the saved lists
    specSets["Motors"] = specNames
    specSets["Payload"] = specPayloads
    specSets["Total Cost"] = [float(x) for x in specCosts]
    specSets["Motor Numbers"] = [[i+1 for i in x] for x in specMotors]
    #Returning the best set of motors (by payload), dataframe with information of sufficient motor sets, and cost of the best set
    return bestMotors, specSets, cost


#Finding the best motors, cost, and all sufficient motor sets for the specified motor database and payload threshold
motors, specSets, cost = run_motor_combos(sortedMotors, 1)
#Defining maximum torques and masses for best motor set and all other information from find_lengths
tmax, masses = set_motors(motors[0], motors[1], motors[2], motors[3], motors[4], sortedMotors)
bestLengths, bestTorque, bestTorqueSet, strugglingMotor, bestMaxPayload, loadedRemainingTorques, failingMotor = find_lengths(tmax, masses)

#Printing all information on the best motor set as well as sending all sufficient sets to a csv file
print("\nBest Motors: \n", sortedMotors["Name"].iat[motors[0]], "\n", sortedMotors["Name"].iat[motors[1]], "\n", sortedMotors["Name"].iat[motors[2]], "\n", sortedMotors["Name"].iat[motors[3]], "\n", sortedMotors["Name"].iat[motors[4]], "\n")
print("Total Cost: $", cost)
print("\n\n")
print("PREPAYLOAD: \nWorst Torque Final (%): ", bestTorque*100)
print("Best Lengths (in): ", [float(x) for x in bestLengths])
print("Best Torques (ft lb): ", bestTorqueSet/12)
print("Best Torques (%): ", 100*bestTorqueSet/tmax)
print("Struggling Motor: ", strugglingMotor)
print("\n\n")
print("WITH PAYLOAD: \nMax Payload: ", round((bestMaxPayload), 4))
print("Max-loaded Torque (ft lb): ", loadedRemainingTorques/12)
print("Max-loaded Torque (%): ", (loadedRemainingTorques/tmax)*100)
print("Failed Motor: ",failingMotor, '\n\n')
print("Other working motor sets: ")
print(specSets)
specSets.to_csv('specSets.csv')