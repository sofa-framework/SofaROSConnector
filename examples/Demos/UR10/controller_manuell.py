import Sofa
import sys

############################################################################################
# this is a PythonScriptController example script
############################################################################################




############################################################################################
# following defs are used later in the script
############################################################################################


def testNodes(node):
	node.findData('name').value = 'god'

	# Node creation
	adam = node.createChild('Adam')
	eve = node.createChild('Eve')
	abel = eve.createChild('Abel')

	#you can animate simulation directly by uncommenting the following line:
	#node.animate=true

	return 0


# Python version of the "oneParticleSample" in cpp located in applications/tutorials/oneParticle
def oneParticleSample(node):
	node.findData('name').value='oneParticleSample'
	node.findData('gravity').value=[0.0, -9.81, 0.0]
	solver = node.createObject('EulerSolver',printLog='false')
	particule_node = node.createChild('particle_node')
	particle = particule_node.createObject('MechanicalObject')
	particle.resize(1)
	mass = particule_node.createObject('UniformMass',totalmass=1)

	return 0


def transformTableInString(Table):
	sizeT =  len(Table);
	strOut= ' ';
	for p in range(sizeT):
		strOut = strOut+ str(Table[p])+' '

		
	return strOut


def transformDoubleTableInSimpleTable(Table):
        size0 =  len(Table);
        # count the size
        size=0
        for i in range(size0):
            size = size+len(Table[i]);

    
        TableOut=[0]*size;
        s=0
        for i in range(size0):
            for j in range(len(Table[i])):
                TableOut[s] = Table[i][j];
                s=s+1;

        return TableOut


############################################################################################
# following defs are optionnal entry points, called by the PythonScriptController component;
############################################################################################


class ExampleController(Sofa.PythonScriptController):
	# called once the script is loaded
	def onLoaded(self,node):
		self.counter = 0
		print 'Controller script loaded from node %s'%node.findData('name').value
		return 0

	# optionnally, script can create a graph...
	def createGraph(self,node):
		print 'createGraph called (python side)'

		#uncomment to create nodes
		#testNodes(node)

		#uncomment to create the "oneParticle" sample
		oneParticleSample(node)

		return 0



	# called once graph is created, to init some stuff...
	def initGraph(self,node):
		print 'initGraph called (python side)'
		self.arbitraryControl = node.getRoot().getChild("DAE_blendfix_scene")\
                    .getChild("KinematicsModels")\
                    .getChild("ur10_kinematics_model.ur10_kinematics_model.")\
                    .getObject("ToolControl")
                if (self.arbitraryControl is not None):
                    print('(controller_manuell.py::robotJointController) ArbitraryController found: %s' % self.arbitraryControl.getName())
                    self.arbitraryControl.findData('ControlIndex').value = [[3,9]]
                    print('(controller_manuell.py::robotJointController) %s joints found.' % (len(self.arbitraryControl.findData('JointNames').value)))
                    print('(controller_manuell.py::robotJointController) Joint names: %s' % (self.arbitraryControl.findData('JointNames').value))
                    print('(controller_manuell.py::robotJointController) Control Index: %s' % (self.arbitraryControl.findData('ControlIndex').value))
		return 0

	def bwdInitGraph(self,node):
		print 'bwdInitGraph called (python side)'
		sys.stdout.flush()
		return 0

	def onIdle(self):
		sys.stdout.flush()
		return 0
		
	def onRecompile(self):
		print("The source ["+__file__+"] has changed and is reloaded.")
		
	# called on each animation step
	total_time = 0
	def onBeginAnimationStep(self,dt):
		self.total_time += dt
		#print 'onBeginAnimatinStep (python) dt=%f total time=%f'%(dt,self.total_time)
		return 0

	def onEndAnimationStep(self,dt):
		sys.stdout.flush()
		return 0

	# called when necessary by Sofa framework... 
	def storeResetState(self):
		print 'storeResetState called (python side)'
		sys.stdout.flush()
		return 0

	def reset(self):
		print 'reset called (python side)'
		sys.stdout.flush()
		return 0

	def cleanup(self):
		print 'cleanup called (python side)'
		sys.stdout.flush()
		return 0


	# called when a GUIEvent is received
	def onGUIEvent(self,controlID,valueName,value):
		print 'GUIEvent received: controldID='+controlID+' valueName='+valueName+' value='+value
		sys.stdout.flush()
		return 0 

	# key and mouse events; use this to add some user interaction to your scripts 
	def onKeyPressed(self,c):
		print 'onKeyPressed '+ str(c)
		sys.stdout.flush()
		
		kinematicValues = self.arbitraryControl.findData('KinematicValues').value
		jointControlledByROS = (self.arbitraryControl.findData('controlledByROS').value);
                print('(controller_manuell.py::robotJointController) Current joint angles: %s' % (kinematicValues))
                print('(controller_manuell.py::robotJointController) Controlled by ROS   : %s' % (jointControlledByROS))

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

                (self.arbitraryControl.findData('KinematicValues').value) = transformTableInString( transformDoubleTableInSimpleTable(kinematicValues) )
                (self.arbitraryControl.findData('controlledByROS').value) = transformTableInString( transformDoubleTableInSimpleTable(jointControlledByROS) )
                        
                return 0 

	def onKeyReleased(self,k):
		print 'onKeyReleased '+k
		sys.stdout.flush()
		return 0 

	def onMouseButtonLeft(self,x,y,pressed):
		print 'onMouseButtonLeft x='+str(x)+' y='+str(y)+' pressed='+str(pressed)
		sys.stdout.flush()
		return 0

	def onMouseButtonRight(self,x,y,pressed):
		print 'onMouseButtonRight x='+str(x)+' y='+str(y)+' pressed='+str(pressed)
		sys.stdout.flush()
		return 0

	def onMouseButtonMiddle(self,x,y,pressed):
		print 'onMouseButtonMiddle x='+str(x)+' y='+str(y)+' pressed='+str(pressed)
		sys.stdout.flush()
		return 0

	def onMouseWheel(self,x,y,delta):
		print 'onMouseButtonWheel x='+str(x)+' y='+str(y)+' delta='+str(delta)
		sys.stdout.flush()
		return 0


	# called at each draw (possibility to use PyOpenGL)
	def draw(self):
		if self.counter < 10:
			print 'draw ('+str(self.counter+1)+' / 10)'
			self.counter += 1
		sys.stdout.flush()

 
