#!/usr/bin/env python
# -*- coding: utf-8 -*-

import Sofa

import copy

import sys
import os
import time

from os.path import isfile, join


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
            
    def cleanup(self):
        print('Python script finishes')
        sys.stdout.flush()
        return 0

    # called on each animation step
    def onBeginAnimationStep(self, dt):
        print('onBeginAnimationStep(' + str(dt) + ')')
        kinematicValues = (self.arbitraryControl.findData('KinematicValues').value);

        speedFactor = 1 # 0..1; 1 means maximum speed the real robot can do

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

        return 0

    def onEndAnimationStep(self, dt):
        print('onEndAnimationStep(' + str(dt) + ')')
        return 0

    def onKeyPressed(self,c):
        print('onKeyPressed(' + str(c) + ')')
        kinematicValues = (self.arbitraryControl.findData('KinematicValues').value);
        jointControlledByROS = (self.arbitraryControl.findData('controlledByROS').value);
        print('(controller_manuell.py::robotJointController) Current joint angles: %s' % (self.arbitraryControl.findData('KinematicValues').value))

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



