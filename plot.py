#!/usr/bin/python
'''
Created on Jun 3, 2013

@author: enalepa
'''

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import math, tools, os

CRAD = 180. / math.pi

class Plot(object):
    def __init__(self, my_file):
        variables_list = []
        line_num = 0
        self.my_file = my_file
        for line in open(my_file, "r"):
            line = line.rstrip('\n')
            if line[0] == '%':
                datas = line.split()
                data_num = 0
                self.type = datas[1]
                self.shortName = datas[2]
                
                if datas[3] == "True":
                        self.state = "Success"
                elif datas[3] == "False":
                        self.state = "Failure"
                else:
                        self.state = "Indeterminate"
                        
            else:
                datas = line.split()
                data_num = 0
                if line_num == 0:
                    for data in datas:
                        variables_list.append([data])
                else:
                    for data in datas:
                        variables_list[data_num].append(data)
                        data_num += 1        
                
                line_num += 1

        self.variables = {}
                
        for elt in variables_list:
            self.variables[elt[0]] = [tools.floatExcept(i) for i in elt[1:]]
            
    def Plot(self):
        plt.figure(self.type + " " + self.shortName + " : " + self.state, figsize=(23,12))
        
        if self.state == "Success":
            my_color = "green"
        else:
            my_color = "red"

        plt.suptitle(self.type + " " + self.shortName + " : " + self.state, color = my_color)

        if self.type == "Joint":
            plt.subplot(211)
            plt.grid(True)
            plt.xlabel("Time (s)")
            plt.ylabel("(Deg)")                   

            plt.plot(self.variables["Time"], self.variables["Actuator"], self.variables["Time"], self.variables["Position"])
            plt.legend(("Actuator", "Position"))
            
            plt.subplot(212)
            plt.xlabel("Time (s)")
            plt.ylabel("(Deg)")

            plt.plot(self.variables["Time"], self.variables["Error"], self.variables["Time"], self.variables["Max"], 'r', self.variables["Time"], self.variables["-Max"], 'r')
            plt.legend(("Error", "Limit"), loc = 'best')
            
        elif self.type == "CurrentSensor":
            plt.subplot(311)
            plt.grid(True)
            plt.xlabel("Time (s)")
            plt.ylabel("(Amp)")

            plt.plot(self.variables["Time"], self.variables["Current"], self.variables["Time"], self.variables["CurrentMax"],'--')
            plt.plot(self.variables["Time"], self.variables["CurrentSA"], linewidth = 2.0, color = "black")
            plt.plot(self.variables["Time"], self.variables["CurrentLimitHigh"], 'r', self.variables["Time"], self.variables["CurrentLimitLow"], 'r')
            plt.legend(("Current", "CurrentMax", "Current Sliding Average", "CurrentLimit"), loc = 'best')
 
            plt.subplot(312)
            plt.xlabel("Time (s)")
            plt.ylabel("(Deg C)")
            
            plt.plot(self.variables["Time"], self.variables["Temperature"], self.variables["Time"], self.variables["TemperatureMax"], 'r')
            plt.legend(("Temperature", "Temperature Max"), loc = 'best')
            
            plt.subplot(313)
            plt.xlabel("Time (s)")
            plt.ylabel("(Deg)")
            
            plt.plot(self.variables["Time"], self.variables["Command"], self.variables["Time"], self.variables["Minimum"], self.variables["Time"], self.variables["Position"])
            plt.legend(("Command", "Minimum", "Position"), loc = 'best')
            
        elif self.type == "Temperature":
            plt.subplot(311)
            plt.grid(True)
            plt.xlabel("Time (s)")
            plt.ylabel("(Deg C)")

            plt.plot(self.variables["Time"], self.variables["Temperature"], self.variables["Time"], self.variables["TemperatureMax"], 'g', self.variables["Time"], self.variables["TemperatureStop"], 'r')
            plt.legend(("Temperature", "Temperature Max", "Temperature Stop"), loc = "best")

            plt.subplot(312)
            plt.xlabel("Time (s)")
            plt.ylabel("(A)")

            plt.plot(self.variables["Time"], self.variables["Current"], self.variables["Time"], self.variables["CurrentMax"])
            plt.plot(self.variables["Time"], self.variables["CurrentSA"], linewidth = 2.0, color = "black")
            plt.plot(self.variables["Time"], self.variables["CurrentLimitHigh"], 'r', self.variables["Time"], self.variables["CurrentLimitLow"], 'r')
            plt.legend(("Current", "Current Max", "Current Sliding Average", "Current Limit"), loc = "best")
            
            plt.subplot(313)
            plt.xlabel("Time (s)")
            plt.ylabel("(Deg)")
            
            plt.plot(self.variables["Time"], self.variables["Command"], self.variables["Time"], self.variables["Minimum"], self.variables["Time"], self.variables["Position"])
            plt.legend(("Command", "Minimum", "Position"), loc = 'best')
            
        elif self.type == "Led":   
            plt.subplot(221)
            plt.title("LED OFF")
            img1 = mpimg.imread(self.my_file[0:-4] + "CropOFF.bmp")
            plt.imshow(img1)
            
            plt.subplot(222)
            plt.title("LED ON")
            img1 = mpimg.imread(self.my_file[0:-4] + "CropON.bmp")
            plt.imshow(img1)

            plt.subplot(223)
            plt.grid(True)
            plt.xlabel("Pixel intensity")
            plt.ylabel("Number of pixels")
            plt.yscale("log")
            plt.xlim(0,255)
            absi = range(len(self.variables["HistogramOff"]))
            plt.bar(absi, self.variables["HistogramOff"], 1)
            
            plt.subplot(224)
            plt.grid(True)
            plt.xlabel("Pixel intensity")
            plt.ylabel("Number of pixels")
            plt.yscale("log")
            plt.xlim(0,255)            
            absi = range(len(self.variables["HistogramOn"]))
            plt.bar(absi, self.variables["HistogramOn"], 1)
            
        elif self.type == "PluginSensorJointLimit:":
            firstAngleRad = tools.separate(self.variables["FirstAngleList"][0], ";")
            firstAngle = [float(item) * CRAD for item in firstAngleRad]
            
            secondAngleMinRad = tools.separate(self.variables["SecondAngleMinList"][0], ";")
            secondAngleMin = [float(item) * CRAD for item in secondAngleMinRad]

            secondAngleMinLimitLowRad = tools.separate(self.variables["SecondAngleMinLimitLowList"][0], ";")
            secondAngleMinLimitLow = [float(item) * CRAD for item in secondAngleMinLimitLowRad]
            
            secondAngleMinLimitHighRad = tools.separate(self.variables["SecondAngleMinLimitHighList"][0], ";")
            secondAngleMinLimitHigh = [float(item) * CRAD for item in secondAngleMinLimitHighRad]                        
            
            secondAngleMaxRad = tools.separate(self.variables["SecondAngleMaxList"][0], ";")
            secondAngleMax = [float(item) * CRAD for item in secondAngleMaxRad]
            
            secondAngleMaxLimitLowRad = tools.separate(self.variables["SecondAngleMaxLimitLowList"][0], ";")
            secondAngleMaxLimitLow = [float(item) * CRAD for item in secondAngleMaxLimitLowRad]
            
            secondAngleMaxLimitHighRad = tools.separate(self.variables["SecondAngleMaxLimitHighList"][0], ";")
            secondAngleMaxLimitHigh = [float(item) * CRAD for item in secondAngleMaxLimitHighRad]
            
            valuePrec = "Undefined"
            for index, value in enumerate(self.variables["Phase"]):
                if value == "None" and valuePrec != "None":
                    firstIndexNone = index
                elif value == "Up" and valuePrec != "Up":
                    firstIndexUp = index
                valuePrec = value
             
            plt.subplot(211)
            plt.grid(True)
            plt.xlabel("First Angle (deg)")
            plt.ylabel("Second Angle (deg)")
            
            plt.plot(self.variables["FirstJointCommand"], self.variables["SecondJointCommand"], 'b', self.variables["FirstJointPosition"], self.variables["SecondJointPosition"], 'g')
            plt.plot(firstAngle, secondAngleMin, 'k--')
            plt.plot(firstAngle, secondAngleMinLimitLow, 'r')
            plt.plot(firstAngle, secondAngleMax, 'k--')
            plt.plot(firstAngle, secondAngleMinLimitHigh, 'r')
            plt.plot(firstAngle, secondAngleMin, 'ko', firstAngle, secondAngleMax, 'ko')
            plt.plot(firstAngle, secondAngleMaxLimitLow, 'r', firstAngle, secondAngleMaxLimitHigh, 'r')
            
            plt.legend(("Command", "Position", "Second Angle Max.", "Second Angle Limit"), loc = "best")
            
            plt.subplot(212)
            plt.grid(True)
            plt.xlabel("Time (s)")
            plt.ylabel("Second Angle (deg)")
            
            plt.plot(self.variables["Time"], self.variables["SecondJointCommand"], 'b', self.variables["Time"], self.variables["SecondJointPosition"], 'g')
            
            # ALMemory bug bypass : Inversion in secondAngleMax / secondAngleMin for Head and Left Ankle
            if "Head" in self.shortName or "LAnkle" in self.shortName:
                plt.plot(self.variables["Time"][:firstIndexNone - 1], self.variables["SecondAngleMax"][:firstIndexNone - 1], 'k--', self.variables["Time"][:firstIndexNone - 1], self.variables["SecondAngleMaxLimitLow"][:firstIndexNone - 1], 'r', self.variables["Time"][:firstIndexNone - 1], self.variables["SecondAngleMaxLimitHigh"][:firstIndexNone - 1], 'r')
                plt.plot(self.variables["Time"][firstIndexUp:], self.variables["SecondAngleMin"][firstIndexUp:], 'k--', self.variables["Time"][firstIndexUp:], self.variables["SecondAngleMinLimitLow"][firstIndexUp:], 'r', self.variables["Time"][firstIndexUp:], self.variables["SecondAngleMinLimitHigh"][firstIndexUp:], 'r')
            else: # Normal case
                plt.plot(self.variables["Time"][:firstIndexNone - 1], self.variables["SecondAngleMin"][:firstIndexNone - 1], 'k--', self.variables["Time"][:firstIndexNone - 1], self.variables["SecondAngleMinLimitLow"][:firstIndexNone - 1], 'r', self.variables["Time"][:firstIndexNone - 1], self.variables["SecondAngleMinLimitHigh"][:firstIndexNone - 1], 'r')
                plt.plot(self.variables["Time"][firstIndexUp:], self.variables["SecondAngleMax"][firstIndexUp:], 'k--', self.variables["Time"][firstIndexUp:], self.variables["SecondAngleMaxLimitLow"][firstIndexUp:], 'r', self.variables["Time"][firstIndexUp:], self.variables["SecondAngleMaxLimitHigh"][firstIndexUp:], 'r')
            
            plt.legend(("Command", "Position", "Max.", "Limit"), loc = "best")
            
        elif self.type == "FSR":
            
            plt.subplot(311)
            plt.grid(True)
            plt.xlabel("Time (s)")
            plt.ylabel("(kg)")
            
            plt.plot(self.variables["Time"], self.variables["RearLeft"], self.variables["Time"], self.variables["RearRight"], self.variables["Time"], self.variables["FrontLeft"], self.variables["Time"], self.variables["FrontRight"])
            plt.legend(("Rear Left", "Rear Right", "Front Left", "Front Right"), loc = "best")
            
            plt.subplot(312)
            plt.grid(True)
            plt.xlabel("Time (s)")
            plt.ylabel("(kg)")
            
            plt.plot(self.variables["Time"], self.variables["TotalWeight"], self.variables["Time"], self.variables["TotalWeightTheorical"])
            plt.legend(("Total Weight", "Total Weight Theoretical"), loc = "best")
            
            plt.subplot(313)
            plt.grid(True)
            plt.xlabel("Time (s)")
            plt.ylabel("(kg)")
            
            plt.plot(self.variables["Time"], self.variables["Error"], self.variables["Time"], self.variables["Limit"], "r", self.variables["Time"], self.variables["LimitNeg"], "r")
            plt.legend(("Error", "Limit"), loc = 'best')

        elif self.type == "CenterOfForcePosition":
            plt.subplot(211)
            plt.grid(True)
            plt.xlabel("Time (s)")
            plt.ylabel("(mm)")
            
            plt.plot(self.variables["Time"], self.variables["CenterOfPressure"], self.variables["Time"], self.variables["CenterOfPressureTheorical"])
            plt.legend(("Center Of Pressure", "Center Of Pressure Theoretical"), loc = "best")
            
            plt.subplot(212)
            plt.grid(True)
            plt.xlabel("Time (s)")
            plt.ylabel("(mm)")
            
            plt.plot(self.variables["Time"], self.variables["Error"], self.variables["Time"], self.variables["Limit"],"r", self.variables["Time"], self.variables["LimitNeg"], "r")
            plt.legend(("Error", "Limit"), loc = 'best')
            
        plt.show()

def listFilesFolders (path, prompt, my_filter = None):
    filesFolders = os.listdir(path)
    filesFolders.sort()
    num = 0
    for fileFolder in filesFolders:
        if num < 10:
            pattern = "{}  - {}"
        else:
            pattern =  "{} - {}"

        if my_filter == None:
            print pattern.format(num, fileFolder)
        else:
            if my_filter in fileFolder:
                print pattern.format(num, fileFolder)
        num += 1

    print
    user = raw_input(prompt)
    if user == "":
        return None
    elif user == "*":
        if my_filter == None:           
            return filesFolders
        else:
            filesFoldersFiltered = []
            for fileFolder in filesFolders:
                if my_filter in fileFolder:
                    filesFoldersFiltered.append(fileFolder)
                    
            return filesFoldersFiltered
    else:
        return [filesFolders [int(userSplitted)] for userSplitted in user.split()]
    
def log10Expt(my_list):
    logList = []
    for elt in my_list:
        if elt != 0:
            logElt = math.log10(elt)
        else:
            logElt = elt
        
        logList.append(logElt)
    return logList

while True:
    os.system("clear")
    print "*****************************************"
    print "* Robot Firmware Automatic Test Results *"
    print "*****************************************"
    print
    trial = listFilesFolders("Results", "Choose your test (Enter to quit) : ")
    if trial is None:
        break
    
    trialPath = "Results" + "/" + trial[0]

    folder = 0
    while True:
        os.system("clear")
        print "***************"
        print "* Sub-devices *"
        print "***************"
        print
        folder = listFilesFolders(trialPath, "Choose your type of sub-device (Enter to go back) : ")

        if folder is None:
            break
        elif folder[0] == "Synthesis":
            print "*************"
            print "* Synthesis *"
            print "*************"
            print
            os.system("cat " + trialPath + "/Synthesis")
            raw_input("Press Enter to continue ...")
        else:
            folderPath = trialPath + "/" + folder[0]

            while True:
                os.system("clear")
                print "**********"
                print "* Figures *"
                print "***********"
                print
                my_files = listFilesFolders(folderPath, "Choose your file(s) (Enter to go back, * to select all) : ", "res")
                if my_files is None:
                    break

                for my_file in my_files:
                    filePath = folderPath + "/" + my_file
                    fig = Plot(filePath)
                    fig.Plot()

