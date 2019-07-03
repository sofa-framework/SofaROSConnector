#!/usr/bin/env python
# -*- coding: utf-8 -*-

import Sofa

import copy

import sys
import os
import time

from os.path import isfile, join

sys.path.append(os.path.abspath("C:\\sofa\\mergeBranch\\git\\sofa\\applications\\trupython"))
from accessDB import *

import psutil


def transformTableInString(Table):
	sizeT =  len(Table);
	strOut= ' ';
	for p in range(sizeT):
		strOut = strOut+ str(Table[p])+' '

		
	return strOut


def transformDoubleTableInSimpleTable(Table):
    size0 =  len(Table);
    # count the size
    size=0;
    for i in range(size0):
        size = size+len(Table[i]);

    
    TableOut=[0]*size;
    s=0;
    for i in range(size0):
        for j in range(len(Table[i])):
            TableOut[s] = Table[i][j];
            s=s+1;

        
    return TableOut


def getForceFromVelocityViaTable(velocity, vToFTable, tolerance = 0.001):

    closestVelocityIndex = 0
    while vToFTable[closestVelocityIndex][0] < velocity and closestVelocityIndex < (len(vToFTable) - 1):
        closestVelocityIndex = closestVelocityIndex + 1

    # check if velocity was found directly
    if abs(vToFTable[closestVelocityIndex][0] - velocity) < tolerance:
        return vToFTable[closestVelocityIndex][0]

    # three cases
    # case 1: velocity is somewhere in the middle of the table
    #   velocity is between vToFTable[closestVelocityIndex] and vToFTable[closestVelocityIndex-1]
    if (closestVelocityIndex > 0) and (closestVelocityIndex < (len(vToFTable) - 1)):
        lowVtoF = vToFTable[closestVelocityIndex-1]
        highVtoF = vToFTable[closestVelocityIndex]

        vDiff = highVtoF[0] - lowVtoF[0]
        fDiff = highVtoF[1] - lowVtoF[1]

        #print("case 1")
        #print("closestVelocityIndex: {0}".format(closestVelocityIndex))
        #print("len(vToFTable): {0}".format(len(vToFTable)))
        #print("velocity: {0}".format(velocity))
        #print("lowVtoF: {0}".format(lowVtoF))
        #print("highVtoF: {0}".format(highVtoF))
        #print("vDiff: {0}".format(vDiff))
        #print("fDiff: {0}".format(fDiff))
        #return lowVtoF[1] + ( ( (highVtoF[0] - velocity) / vDiff ) * fDiff )
        return lowVtoF[1] + ( ( (velocity - lowVtoF[0]) / vDiff ) * fDiff )

    # case 2: velocity is before the beginning of the table
    #   velocity is smaller than vToFTable[0]
    if (closestVelocityIndex == 0):
        lowVtoF = vToFTable[0]
        highVtoF = vToFTable[1]

        vDiff = highVtoF[0] - lowVtoF[0]
        fDiff = highVtoF[1] - lowVtoF[1]

        #print("case 2")
        #print("closestVelocityIndex: {0}".format(closestVelocityIndex))
        #print("len(vToFTable): {0}".format(len(vToFTable)))
        #print("velocity: {0}".format(velocity))
        #print("lowVtoF: {0}".format(lowVtoF))
        #print("highVtoF: {0}".format(highVtoF))
        #print("vDiff: {0}".format(vDiff))
        #print("fDiff: {0}".format(fDiff))

        #return lowVtoF[1] + ( ( (highVtoF[0] - velocity) / vDiff ) * fDiff )
        return lowVtoF[1] + ( ( (velocity - lowVtoF[0]) / vDiff ) * fDiff )

    # case 3: velocity is after the end of the table
    #   velocity is larger than vToFTable[len(vToFTable)-1]
    if (closestVelocityIndex == (len(vToFTable) - 1)):
        lowVtoF = vToFTable[closestVelocityIndex-1]
        highVtoF = vToFTable[closestVelocityIndex]

        vDiff = highVtoF[0] - lowVtoF[0]
        fDiff = highVtoF[1] - lowVtoF[1]

        #print("case 3")
        #print("closestVelocityIndex: {0}".format(closestVelocityIndex))
        #print("len(vToFTable): {0}".format(len(vToFTable)))
        #print("velocity: {0}".format(velocity))
        #print("lowVtoF: {0}".format(lowVtoF))
        #print("highVtoF: {0}".format(highVtoF))
        #print("vDiff: {0}".format(vDiff))
        #print("fDiff: {0}".format(fDiff))

        return lowVtoF[1] + ( ( (velocity - highVtoF[0]) / vDiff ) * fDiff )

    print("ERROR: Never go here")
    return -1.0




# class controllerRobot(Sofa.PythonScriptController):
#
#     def initGraph(self, node):
#         self.MechanicalState = node.getObject('banane')
#         #self.rootNode = node.getRoot()
#         # self.armatureEnd = node.getRoot()\
#         #     .getChild("DAE_scene")\
#         #     .getChild("root_POWERBALL")\
#         #     .getChild("Gelenk_6.1_8")\
#         #     .getChild("rigid_6")\
#         #     .getObject("Base_geo6")
#         self.armatureEnd = node.getRoot()\
#             .getChild("DAE_scene")\
#             .getChild("FlanscheBasis_23")\
#             .getChild("rigid_7")\
#             .getObject("Base_geo14")
#         self.oldPos = self.armatureEnd.position
#         print('End of kinematics chain (tool attachment point): %s' % self.armatureEnd)
#
#     # called on each animation step
#     total_time = 0
#     def onEndAnimationStep(self,dt):
#     #def onBeginAnimationStep(self,dt):
#         self.total_time += dt
#         print('onBeginAnimatinStep (python) dt=%f total time=%f'%(dt,self.total_time))
#         print('I am %s'%(self.MechanicalState.name))
#
#         currPos = self.MechanicalState.position
#         newPos = self.armatureEnd.position
#
#         if (len(newPos) != 1):
#             print("ERROR: Something is wrong with the length of %s.position, should be 1 (it should be a rigid with a single 7 element position and orientation vector)." % self.armatureEnd.name)
#
#         posChange = [newPos[0][i] - self.oldPos[0][i] for i in range(len(newPos[0]))]
#
#         #print('oldPos: %s'% self.oldPos )
#         #print('newPos: %s'% newPos )
#         print('Moving tool by: %s'%posChange)
#
#         #print('Current position is %s' % currPos)
#         #print('Setting position to %s' % newPos)
#
#         numNode=len(currPos);
#         for i in range(numNode):
#             for dim in range(len(currPos[i])):
#                 currPos[i][dim] += posChange[dim];
#
#         self.MechanicalState.position=currPos
#
#         #print('New position is %s' % currPos)
#         self.oldPos = newPos
#         return 0


class robotJointController(Sofa.PythonScriptController):

    def initGraph(self, node):
        self.firstSteps = 2

        self.executeMotion = False
        self.steps=0

        self.arbitraryControl = node.getRoot()\
         .getChild("DAE_blendfix_scene")\
         .getChild("KinematicsModels")\
         .getChild("ur10_kinematics_model.ur10_kinematics_model.")\
         .getObject("ToolControl")
        if (self.arbitraryControl is not None):
            print('(controller_manuell.py::robotJointController) ArbitraryController found: %s' % self.arbitraryControl.getName())
            self.arbitraryControl.findData('ControlIndex').value = [[3,9]]
            print('(controller_manuell.py::robotJointController) %s joints found.' % (len(self.arbitraryControl.findData('JointNames').value)))
            print('(controller_manuell.py::robotJointController) Joint names: %s' % (self.arbitraryControl.findData('JointNames').value))
            print('(controller_manuell.py::robotJointController) Control Index: %s' % (self.arbitraryControl.findData('ControlIndex').value))
            #print('(controller_manuell.py::robotJointController) Initial joint angles: %s' % (self.arbitraryControl.findData('KinematicValues').value))

        self.objectManager = node.getRoot().getObject("truPhysicsObjectManager")
        #self.objectManager = None

        if self.objectManager is not None:
        #if 'self.objectManager' in locals():
            print('(controller_manuell.py::robotJointController) self.objectManager: %s' % self.objectManager)
            #self.dba = accessDB('schenketest', 'testpassBanane123', 'truserver', 'velocityTestDB', 'VELOCITIES_TEST', True)
            #self.dba = accessDB('schenketest', 'testpassBanane123', '192.168.179.254', 'velocityTestDB', 'VELOCITIES_TEST', True)
            #AAAAAAAAAAAAAAAAAAAAAAA#self.dba = accessDB('developer1', 'truPhysics123', 'truephysicrunsafe.cjtfzb1mil5b.us-west-2.rds.amazonaws.com', 'truephysicrunsafe', 'punkt_geschwindigkeiten', True)
            #AAAAAAAAAAAAAAAAAAAAAAA#self.dba.connectDB()
            ###careful###self.dba.createTable()
            self.lastWrittenTime=-1.0
            self.sTime = time.time()

            # lookup table for velocity -> force data
            self.vToFTable = []
            vToFFile = file("C:\\sofa\\mergeBranch\\git\\sofa\\applications\\trupython\\velocityToForceTable.dat","r")
            for vToFLine in vToFFile:
                vToFData = vToFLine.split(" ")
                self.vToFTable.append([ float(vToFData[0]), float(vToFData[1]) ])
            vToFFile.close()
            #print(self.vToFTable)

            #AAAAAAAAAAAAAAAAAAAAAAA# von hier
            self.outString = ""
            self.velAvgCnt = 1
            #self.velAvgCnt = 1
            self.velAvg =[]
            self.inFileDir = "C:\\tmp\\hannover\\"
            self.inFileName = "out.dat"
            self.writeAtEnd = False
            self.createImage = True

            if not self.writeAtEnd:
                datafile = file(join(self.inFileDir, self.inFileName),"w")
                datafile.write("#TruPhysics simulation result\n")
                datafile.write("#simTime, velMag, posX, posY, posZ\n")
                datafile.close()

            #AAAAAAAAAAAAAAAAAAAAAAA# bis hier

    def cleanup(self):
        print('Python script finishes')
        #AAAAAAAAAAAAAAAAAAAAAAA#if self.objectManager is not None:
        #AAAAAAAAAAAAAAAAAAAAAAA#    self.dba.commitAndDisconnectDB()
        if self.writeAtEnd:
            #AAAAAAAAAAAAAAAAAAAAAAA# von hier
            #outfile = file("C:\\Users\\MesseUser\\Documents\\out.dat","w")
            datafile = file(join(self.inFileDir, self.inFileName),"w")
            datafile.write(self.outString)
            datafile.close()

        if self.createImage:
            # HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH
            tRange = '[]'
            xRange = '[]'
            dataRangesString = tRange + xRange

            gnuplotMaxTime = 120

            gnuplotGeneralSettings = 'set key;' \
                             'set term png size 1200,800;'

            gnuplotTitle = 'set title "TruPhysics simulation output" noenhanced;'

            outDir = "C:\\Users\\Schenke\\Documents\\"
            outFile = "out.png"

            dataFileString = '"' + join(self.inFileDir, self.inFileName).replace("\\","/") + '"'

            gnuplotOutput = 'set out "' + join(outDir, outFile).replace("\\","/") + '";'

            gnuplotCommandString = 'plot' + dataRangesString + ' '\
                             + dataFileString + ' using 1:2 w l lc rgb "blue" title "velocity", ' \
                             + dataFileString + ' using 1:2 w p lc rgb "blue" notitle , ' \
                             + dataFileString + ' using 1:3 w l lc rgb "gree" title "x", ' \
                             + dataFileString + ' using 1:3 w p lc rgb "green" notitle , ' \
                             + dataFileString + ' using 1:4 w l lc rgb "red" title "y", ' \
                             + dataFileString + ' using 1:4 w p lc rgb "red" notitle , ' \
                             + dataFileString + ' using 1:5 w l lc rgb "orange" title "z", ' \
                             + dataFileString + ' using 1:5 w p lc rgb "orange" notitle , ' \
                             ';'

            try:
                #p1 = psutil.Popen([join(logDir, "gnuplot"),gnuplotString],env=env_main,stdout=outf)
                p1 = psutil.Popen(['gnuplot','-e', gnuplotGeneralSettings + gnuplotTitle + gnuplotOutput + gnuplotCommandString])
                p1.wait(gnuplotMaxTime)
            except psutil.TimeoutExpired:
                print("WARNING: gnuplot has taken longer than {0} seconds trying to create {1}. "
                      "Something is probably wrong, unless the datafile is very big.".format(gnuplotMaxTime, outFile))
                p1.wait()
            # HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH
            #AAAAAAAAAAAAAAAAAAAAAAA# bis hier
        sys.stdout.flush()
        return 0

    # called on each animation step
    def onBeginAnimationStep(self, dt):
        kinematicValues = (self.arbitraryControl.findData('KinematicValues').value);

        speedFactor = 1 # 0..1; 1 means maximum speed the real robot can do

        #robotIncrementValue=72 * dt * speedFactor; # values for SCHUNK lwa4p
        #maxStep = ((360 / 72) * (1/dt) * (1/speedFactor)) + 1; # values for SCHUNK lwa4p

        robotIncrementValue=120 * dt * speedFactor;
        maxStep = ((360 / 120) * (1/dt) * (1/speedFactor)) + 1;

        if self.executeMotion and self.steps==0:
            # circle trajectory
            kinematicValues[4][0]  = 0;
            kinematicValues[5][0]  = 0;
            kinematicValues[6][0]  = 0;
            kinematicValues[7][0]  = -90;
            kinematicValues[8][0]  = 0;
            kinematicValues[9][0] = 0;

            self.steps = self.steps + 1
        else:
            if self.executeMotion and self.steps < maxStep:
                # circle trajectory
                kinematicValues[4][0]-=robotIncrementValue;

                self.steps = self.steps + 1

        (self.arbitraryControl.findData('KinematicValues').value) = transformTableInString( transformDoubleTableInSimpleTable(kinematicValues) )

        return 0

    def onEndAnimationStep(self, dt):
        #AAAAAAAAAAAAAAAAAAAAAAA#if self.firstStep:
        #AAAAAAAAAAAAAAAAAAAAAAA#    self.firstStep = False
        #AAAAAAAAAAAAAAAAAAAAAAA#    return 0
        #AAAAAAAAAAAAAAAAAAAAAAA# von hier
        if self.firstSteps > 0:
            self.firstSteps = self.firstSteps - 1
            return 0
        #AAAAAAAAAAAAAAAAAAAAAAA# bis hier

        #AAAAAAAAAAAAAAAAAAAAAAA#if self.objectManager is not None:
        if True:
            #identifierData = self.objectManager.findData('currentVelocityAdditionalData').value
            #simDataVec = self.objectManager.findData('currentVelocityApproximationData').value
            simDataVec = copy.deepcopy( self.objectManager.findData('currentVelocityApproximationData').value )

            #print(simDataVec)

            simTime = -1
            while len(simDataVec) > 0:
                #print(simDataVec)
                simData = simDataVec[0]
                simDataVec = simDataVec[1:]
                #print("simData: {0}\n".format(simData))
                #velocityDataString = "Velocity data: {0}".format(simData)
                #print(velocityDataString)

                uniqueIdentifier = simData[0]
                meshName = simData[1]
                selectedVertex = simData[2]
                pointIndex = simData[3]
                testrunIndex = simData[4]

                #cF=0.1
                cF=1

                simTime = simData[5]
                velX = float(simData[6])*cF
                velY = float(simData[7])*cF
                velZ = float(simData[8])*cF

                #AAAAAAAAAAAAAAAAAAAAAAA#velMag = float(simData[9])*cF
                #AAAAAAAAAAAAAAAAAAAAAAA# von hier
                self.velAvg.append(float(simData[9])*cF)
                if len(self.velAvg) > self.velAvgCnt:
                    self.velAvg = self.velAvg[1:len(self.velAvg)]
                tmp = 0
                #print("AAAAAAAAAAAAAAAAAAAAAAAA: {0}".format(self.velAvg))
                for bla in self.velAvg:
                    tmp = tmp + bla
                velMag = tmp/len(self.velAvg)
                #AAAAAAAAAAAAAAAAAAAAAAA# bis hier
                posX = simData[10]
                posY = simData[11]
                posZ = simData[12]
                forMag = getForceFromVelocityViaTable(velMag,self.vToFTable)

                if float(simTime) > self.sTime:
                    simTime = str(float(simTime) - self.sTime)

                if (self.lastWrittenTime == simTime):
                    break

                #print("SQL write: {0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14}".format(0, uniqueIdentifier, meshName, selectedVertex, pointIndex, simTime, velX, velY, velZ, velMag, posX, posY, posZ, forMag, testrunIndex))

                #AAAAAAAAAAAAAAAAAAAAAAA# von hier
                ####self.outString = self.outString + "{0} {1} {2}\n".format(simTime, velMag, forMag)

                dataOutLine = "{0} {1} {2} {3} {4}\n".format(simTime, velMag, posX, posY, posZ)
                if self.writeAtEnd:
                    self.outString = self.outString + dataOutLine #WITHGNUPLOT#
                else:
                    print("AAAAAAAAAAAAAAAAAAAAAAAAAA")
                    datafile = file(join(self.inFileDir, self.inFileName),"a")
                    datafile.write(dataOutLine)
                    datafile.close()


                #AAAAAAAAAAAAAAAAAAAAAAA# bis hier
                #self.dba.writeDBEntry(selectedVertex, uniqueIdentifier, meshName, selectedVertex, pointIndex, simTime, velX, velY, velZ, velMag, posX, posY, posZ, testrunIndex)

                #self.dba.writeDBEntry(0, uniqueIdentifier, meshName, selectedVertex, 222, simTime, velX, velY, velZ, velMag, posX, posY, posZ, 444) # with pointIndex and testrunIndex set to values that fit the dummy entries I made in the AWS-DB
                #AAAAAAAAAAAAAAAAAAAAAAA#self.dba.writeDBEntry(0, uniqueIdentifier, meshName, selectedVertex, pointIndex, simTime, velX, velY, velZ, velMag, posX, posY, posZ, forMag, testrunIndex)

                #outFile = open('b:\\banane.txt', 'a')
                #outFile.write(velocityDataString + '\n')
                #outFile.close()

            self.lastWrittenTime=simTime
        print("python sql writing done")
        #self.objectManager.findData('currentVelocityData').value = ' '

        return 0

    def onKeyPressed(self,c):
        kinematicValues = (self.arbitraryControl.findData('KinematicValues').value);
        jointControlledByROS = (self.arbitraryControl.findData('controlledByROS').value);
        print('(controller_manuell.py::robotJointController) Current joint angles: %s' % (kinematicValues))

        robotIncrementValue=0.5

        if (c == "C"):
            self.executeMotion = not self.executeMotion

        # toggle robot
        if (c == "Y"):
            jointControlledByROS[4][0] = 1 - jointControlledByROS[5][0];
            jointControlledByROS[5][0] = 1 - jointControlledByROS[6][0];
            jointControlledByROS[6][0] = 1 - jointControlledByROS[7][0];
            jointControlledByROS[7][0] = 1 - jointControlledByROS[8][0];
            jointControlledByROS[8][0] = 1 - jointControlledByROS[9][0];
            jointControlledByROS[9][0] = 1 - jointControlledByROS[10][0];

        if (c == "K"):
            kinematicValues[4][0]  = 165;
            kinematicValues[5][0]  = -85;
            kinematicValues[6][0]  = 65;
            kinematicValues[7][0]  = -5;
            kinematicValues[8][0]  = 95;
            kinematicValues[9][0]  = 0;

        if (c == "L"):
            kinematicValues[4][0]  = 212;
            kinematicValues[5][0]  = -21;
            kinematicValues[6][0]  = -77;
            kinematicValues[7][0]  = 81;
            kinematicValues[8][0]  = 43;
            kinematicValues[9][0]  = 10;

        ### robot
        # first joint
        if (c == "1"):
            kinematicValues[4][0]+=robotIncrementValue;

        if (c == "A"):
            kinematicValues[4][0]-=robotIncrementValue;

        # second joint
        if (c == "2"):
            kinematicValues[5][0]+=robotIncrementValue;

        if (c == "W"):
            kinematicValues[5][0]-=robotIncrementValue;

        # third joint
        if (c == "3"):
            kinematicValues[6][0]+=robotIncrementValue;

        if (c == "D"):
            kinematicValues[6][0]-=robotIncrementValue;

        # fourth joint
        if (c == "4"):
            kinematicValues[7][0]+=robotIncrementValue;

        if (c == "F"):
            kinematicValues[7][0]-=robotIncrementValue;

        # fifth joint
        if (c == "5"):
            kinematicValues[8][0]+=robotIncrementValue;

        if (c == "G"):
            kinematicValues[8][0]-=robotIncrementValue;

        # sixth joint
        if (c == "6"):
            kinematicValues[9][0]+=robotIncrementValue;

        if (c == "H"):
            kinematicValues[9][0]-=robotIncrementValue;

        # useful position
        #if (c == "M"):
        #    kinematicValues[0][0]=0;
        #    kinematicValues[0][1]=-20;
        #    kinematicValues[0][2]=-120;
        #    kinematicValues[0][3]=0;
        #    kinematicValues[0][4]=50;
        #    kinematicValues[0][5]=0;
        # if (c == "K"): # careful, 'K' is currently used above for the hand starting position
        #     kinematicValues[0][0]=0;
        #     kinematicValues[0][1]=-30;
        #     kinematicValues[0][2]=-70;
        #     kinematicValues[0][3]=0;
        #     kinematicValues[0][4]=-60;
        #     kinematicValues[0][5]=0;

        # if (c == "M"):
        #     kinematicValues[0][0]=0;
        #     kinematicValues[0][1]=-50;
        #     kinematicValues[0][2]=-70;
        #     kinematicValues[0][3]=0;
        #     kinematicValues[0][4]=-60;
        #     kinematicValues[0][5]=0;

        (self.arbitraryControl.findData('KinematicValues').value) = transformTableInString( transformDoubleTableInSimpleTable(kinematicValues) )
        (self.arbitraryControl.findData('controlledByROS').value) = transformTableInString( transformDoubleTableInSimpleTable(jointControlledByROS) )
        return 0


class objectController(Sofa.PythonScriptController):

    def initGraph(self, node):
        self.rigidMap = node.getObject('falling_1_Object');


    def onKeyPressed(self,c):
        objPos = self.rigidMap.findData('position').value;

        numNode=len(objPos);

        positionChange = 2

        # if (c == "8"):
        #     for i in range(numNode):
        #         objPos[i][1]+=positionChange;
        #
        # if (c == "2"):
        #     for i in range(numNode):
        #         objPos[i][1]-=positionChange;
        #
        # if (c == "4"):
        #     for i in range(numNode):
        #         objPos[i][0]+=positionChange;
        #
        # if (c == "6"):
        #     for i in range(numNode):
        #         objPos[i][0]-=positionChange;
        #
        # if (c == "/"):
        #     for i in range(numNode):
        #         objPos[i][2]+=positionChange;
        #
        # if (c == "5"):
        #     for i in range(numNode):
        #         objPos[i][2]-=positionChange;

        # UP key############################## NO more necessary ??? ######################
        #if ord(c)==19:
        #    for i in range(numNode):
        #        restPos[i][2]+=0.005;

        # DOWN key
        #if ord(c)==21:
        #    for i in range(numNode):
        #        restPos[i][2]-=0.005;

        # LEFT key
        #if ord(c)==18:
        #    for i in range(numNode):
        #        restPos[i][0]-=0.005;

        # RIGHT key
        #if ord(c)==20:
        #    for i in range(numNode):
        #        restPos[i][0]+=0.005;
        #########################################################

        self.rigidMap.findData('position').value = transformTableInString( transformDoubleTableInSimpleTable(objPos) )
        return 0



class controllerGrasper1(Sofa.PythonScriptController):

    def initGraph(self, node):
        # now we will change the values in the mapping !!!
        self.rigidMap = node.getObject('map');
    

    def onKeyPressed(self,c):
        restPos = self.rigidMap.findData('initialPoints').value;

        
        numNode=len(restPos);
            
        if (c == "+"):
            for i in range(numNode):
                restPos[i][1]+=0.1;

        if (c == "-"):
            for i in range(numNode):
                restPos[i][1]-=0.1;
            
        if (c == "/"):
            for i in range(numNode):
                restPos[i][1]+=1;

        if (c == "*"):
            for i in range(numNode):
                restPos[i][1]-=1;

        # UP key############################## NO more necessary ??? ######################
        #if ord(c)==19:
        #    for i in range(numNode):
        #        restPos[i][2]+=0.005;

        # DOWN key
        #if ord(c)==21:
        #    for i in range(numNode):
        #        restPos[i][2]-=0.005;

        # LEFT key
        #if ord(c)==18:
        #    for i in range(numNode):
        #        restPos[i][0]-=0.005;

        # RIGHT key
        #if ord(c)==20:
        #    for i in range(numNode):
        #        restPos[i][0]+=0.005;
        #########################################################

        self.rigidMap.findData('initialPoints').value = transformTableInString( transformDoubleTableInSimpleTable(restPos) )
        return 0


 


class controllerGrasper2(Sofa.PythonScriptController):
    
    def initGraph(self, node):
        # now we will change the values in the mapping !!!
        self.rigidMap = node.getObject('map');
    
    
    def onKeyPressed(self,c):
        restPos = self.rigidMap.findData('initialPoints').value;
        numNode=len(restPos);
            
        if (c == "+"):
            for i in range(numNode):
                restPos[i][1]-=0.1;

        if (c == "-"):
            for i in range(numNode):
                restPos[i][1]+=0.1;
            
        if (c == "/"):
            for i in range(numNode):
                restPos[i][1]-=1;

        if (c == "*"):
            for i in range(numNode):
                restPos[i][1]+=1;

        # UP key############################## NO more necessary ??? ######################
        #if ord(c)==19:
        #    for i in range(numNode):
        #        restPos[i][2]+=0.005;

        # DOWN key
        #if ord(c)==21:
        #    for i in range(numNode):
        #        restPos[i][2]-=0.005;

        # LEFT key
        #if ord(c)==18:
        #    for i in range(numNode):
        #        restPos[i][0]-=0.005;

        # RIGHT key
        #if ord(c)==20:
        #    for i in range(numNode):
        #        restPos[i][0]+=0.005;
        #########################################################

        self.rigidMap.findData('initialPoints').value = transformTableInString( transformDoubleTableInSimpleTable(restPos) )

        return 0



