from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from dynamic_graph.sot.core.meta_tasks_kine import *
from dynamic_graph.sot.core.meta_tasks import generic6dReference, Task, GainAdaptive
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate,matrixToRPY
from dynamic_graph.sot.core.meta_tasks import *
from dynamic_graph.sot.core.meta_task_posture import MetaTaskPosture, MetaTaskKinePosture
from dynamic_graph import plug
from numpy import *
from dynamic_graph.sot.application.velocity.precomputed_tasks import Application
from dynamic_graph.sot.tools import Oscillator, Seqplay
from dynamic_graph.sot.dynamics.zmp_from_forces import ZmpFromForces

toList = lambda sot: map(lambda x: x[2:],sot.display().split('\n')[3:-2])

def degToRad(deg):
    rad=(deg*pi)/180
    return rad

def change6dPositionReference(task,feature,gain,position,selec=None,ingain=None,resetJacobian=True):
    M=generic6dReference(position)
    if selec!=None:
        if isinstance(selec,str):  feature.selec.value = selec
        else: feature.selec.value = toFlags(selec)
    feature.reference.value = matrixToTuple(M)
    if gain!=None:  setGain(gain,ingain)
    if 'resetJacobianDerivative' in task.__class__.__dict__.keys() and resetJacobian:
        task.resetJacobianDerivative()

class Hrp2Bike(Application):

    tracesRealTime = True

    bikeSitting = (
        # Free flyer
        0., 0., 0.648702, 0., 0. , 0.,

        # Legs
        0., 0., -0.453786, 0.872665, -0.418879, 0.,
        0., 0., -0.453786, 0.872665, -0.418879, 0.,

        # Chest and head
        0., 0., 0., 0.,

        # Arms
        0.261799, -0.17453, 0., -0.523599, 0., 0., 0.1,
        0.261799, 0.17453,  0., -0.523599, 0., 0., 0.1,
        )

    def __init__(self,robot,hands=True):#, forceSeqplay=True):
        Application.__init__(self,robot)

        self.sot=self.solver.sot
        self.hands=hands
#        self.robot=robot
#        self.seq=Seqplay('seqplay')
        self.createTasks(robot)
        self.initTasks()
        self.initTaskGains()
        self.initOscillator()
        self.initSolver()
        self.initialStack()

    def printSolver(self): #show tasks in solver controling the robot
        print self.solver


    #----------TASKS-------------------------------
    #------------------CREATION--------------------

    def createTrunkTask (self, robot, taskName, ingain=1.0):
        task = Task (taskName)
        task.add (self.features['chest'].name)
        task.add (self.features['waist'].name)
        task.add (self.features['gaze'].name)
        gain = GainAdaptive(taskName+'gain')
        gain.setConstant(ingain)
        plug(gain.gain, task.controlGain)
        plug(task.error, gain.error)
        return (task, gain)

    def createTasks(self,robot):
        self.taskHalfSitting = MetaTaskKinePosture(self.robot.dynamic,'halfsitting')
        self.taskBikeSitting = MetaTaskKinePosture(self.robot.dynamic,'bike-sitting')
        self.taskGripper     = MetaTaskKinePosture(self.robot.dynamic,'gripper')

        (self.tasks['trunk'],self.gains['trunk'])= self.createTrunkTask(robot, 'trunk')

        self.taskRF          = self.tasks['left-ankle']
        self.taskLF          = self.tasks['right-ankle']
        self.taskCom         = self.tasks['com']
        self.taskRH          = self.tasks['right-wrist']
        self.taskLH          = self.tasks['left-wrist']
        self.taskTrunk       = self.tasks['trunk']
        self.taskPosture     = self.tasks['posture']
        self.taskBalance     = self.tasks['balance']

    def openGripper(self):
        self.taskGripper.gotoq(None,rhand=(self.gripperOpen,),lhand=(self.gripperOpen,))

    def closeGripper(self):
        self.taskGripper.gotoq(None,rhand=(self.gripperClose,),lhand=(self.gripperClose,))

    #------------------INIT-TASK------------------
    def initTasks(self):
        self.initTaskBalance()
        if self.hands:
            self.initTaskGripper()
        self.initTaskHalfSitting()
        #self.initTaskBikeSitting()


    def initTaskBalance(self):
        # --- BALANCE ---
        self.features['chest'].frame('desired')
        self.features['waist'].frame('desired')
        self.features['chest'].selec.value = '111000'
        self.features['waist'].selec.value = '111000'
        self.featureCom.selec.value = '111'

    def initTaskHalfSitting(self):
        self.taskHalfSitting.gotoq(None,\
                                    rleg =(self.robot.halfSitting[6:12]),   \
                                    lleg =(self.robot.halfSitting[12:18]),  \
                                    chest=(self.robot.halfSitting[18:20]),  \
                                    head =(self.robot.halfSitting[20:22]),  \
                                    rarm =(self.robot.halfSitting[22:28]),  \
                                    larm =(self.robot.halfSitting[29:35]))

    def initTaskBikeSitting(self):
        self.taskBikeSitting.gotoq(None,\
                                    rleg =(self.bikeSitting[6:12]),   \
                                    lleg =(self.bikeSitting[12:18]),  \
                                    chest=(self.bikeSitting[18:20]),  \
                                    head =(self.bikeSitting[20:22]),  \
                                    rarm =(self.bikeSitting[22:28]),  \
                                    larm =(self.bikeSitting[29:35]))

    def initTaskGripper(self):
        self.gripperOpen = degToRad(30)
        self.gripperClose = degToRad(3)
        self.openGripper()

    def setOsciFreq(self,f):
        self.oscillatorRoll.omega.value=f*pi
        self.oscillatorPitch.omega.value=f*pi

    def setOsciMagni(self,m):
        self.oscillatorRoll.magnitude.value=m*pi
        self.oscillatorPitch.magnitude.value=m*pi

    def initOscillator(self):
        self.oscillatorRoll=Oscillator('oscillatorRoll')
        self.oscillatorRoll.setContinuous(True)
        self.oscillatorRoll.setActivated(True)
        self.oscillatorRoll.setTimePeriod(self.robot.timeStep)
        self.oscillatorRoll.setActivated(False)
        self.oscillatorRoll.magnitude.value = 0.1
        self.oscillatorRoll.phase.value = 0.0
        self.oscillatorRoll.omega.value = 0.75

        self.oscillatorPitch = Oscillator('oscillatorPitch')
        self.oscillatorPitch.setContinuous(True)
        self.oscillatorPitch.setActivated(True)
        self.oscillatorPitch.setTimePeriod(self.robot.timeStep)
        self.oscillatorPitch.setActivated(False)
        self.oscillatorPitch.magnitude.value = 0.1
        self.oscillatorPitch.phase.value = 1.57
        self.oscillatorPitch.omega.value = 0.75

        self.stackRP = Stack_of_vector('StackOscRollPitch')
        plug ( self.oscillatorRoll.vectorSout, self.stackRP.sin1 )  
        plug ( self.oscillatorPitch.vectorSout, self.stackRP.sin2 )
        self.stackRP.selec1(0,1)
        self.stackRP.selec2(0,1)
                
        self.stackRPY = Stack_of_vector('StackOscRollPitchYaw')
        plug ( self.stackRP.sout, self.stackRPY.sin1 )  
        self.stackRPY.sin2.value = (0.0,)
        self.stackRPY.selec1(0,2)
        self.stackRPY.selec2(0,1)
        
        self.stackPoseRPY = Stack_of_vector('StackOscPoseRollPitchYaw')
        self.stackPoseRPY.sin1.value = (0.0,0.0,0.0)
        plug ( self.stackRPY.sout, self.stackPoseRPY.sin2 )
        self.stackPoseRPY.selec1(0,3)
        self.stackPoseRPY.selec2(0,3)
        
        self.poseRPYaw2Homo = PoseRollPitchYawToMatrixHomo('OscPoseRPYaw2Homo')
        plug ( self.stackPoseRPY.sout , self.poseRPYaw2Homo.sin)

        self.headRef = Multiply_of_matrixHomo('headRef')
        self.headRef.sin1.value = self.robot.dynamic.signal('gaze').value
        plug( self.poseRPYaw2Homo.sout, self.headRef.sin2)
        plug( self.headRef.sout, self.features['gaze'].reference)
    
        self.chestRef = Multiply_of_matrixHomo('chestRef')
        self.chestRef.sin1.value = self.robot.dynamic.signal('chest').value
        plug( self.poseRPYaw2Homo.sout, self.chestRef.sin2)
        plug( self.chestRef.sout, self.features['chest'].reference)

        self.waistRef = Multiply_of_matrixHomo('waistRef')
        self.waistRef.sin1.value = self.robot.dynamic.signal('waist').value
        plug( self.poseRPYaw2Homo.sout, self.waistRef.sin2)
        plug( self.waistRef.sout, self.features['waist'].reference)

        self.setOsciFreq(1)
        self.setOsciMagni(1)


    #------------------INIT-GAIN------------------
    def initTaskGains(self):
        self.taskHalfSitting.gain.setByPoint(2,0.2,0.01,0.8)


    #----------SOLVER------------------------------
    def initSolver(self):
        plug(self.sot.control,self.robot.device.control)

    def push(self,task): #push task in solver 
        if isinstance(task,str): taskName=task
        elif "task" in task.__dict__:  taskName=task.task.name
        else: taskName=task.name
        if taskName not in toList(self.sot):
            self.sot.push(taskName)
        if taskName!="posture" and "posture" in toList(self.sot):
            self.sot.down("posture")

    def removeTasks(self,task) : #remove task from solver
            if isinstance(task,str): taskName=task
            elif "task" in task.__dict__:  taskName=task.task.name
            else: taskName=task.name
            if taskName in toList(self.sot):
                self.sot.remove(taskName)

    def printSolver(self): #show tasks in solver controlling the robot
        print self.solver

    def showTasks(self) : #show library of precomputed tasks
        self.tasks

    # --- TRACES -----------------------------------------------------------
    def withTraces(self):
        if self.tracesRealTime:
            self.robot.initializeTracer()
        else:
            self.robot.tracer = Tracer('trace')
            self.robot.device.after.addSignal('{0}.triger'.format(self.robot.tracer.name))
        self.robot.tracer.open('/tmp/','','.dat')
        self.robot.startTracer()

    def stopTraces(self):
        self.robot.stopTracer()

    def trace(self):
        self.robot.tracer.dump()

    #----------RUN---------------------------------

    def initialStack(self):
        self.push(self.taskBalance)
        self.push(self.taskTrunk)
        self.push(self.taskPosture)
        if self.hands:
            self.push(self.taskGripper)

    def goHalfSitting(self):
        self.sot.clear()
        self.push(self.taskBalance)
        self.push(self.taskPosture)
        if self.hands:
            self.push(self.taskGripper)
        self.push(self.taskHalfSitting)


    def goBikeSitting(self):
        self.sot.clear()
        self.push(self.taskBalance)
        self.push(self.tasks['chest'])
#        self.push(self.taskPosture)
        if self.hands:
            self.push(self.taskGripper)
        change6dPositionReference(self.taskRH,self.features['right-wrist'],\
                                    self.gains['right-wrist'],\
                                    #(0.3,-0.3,1.1,-pi/2,0,pi/2),'111111')
                                    (0.3785,-0.1365,1.07,-pi/2,0,pi/2),'111111')
        self.push(self.taskRH)
        change6dPositionReference(self.taskLH,self.features['left-wrist'],\
                                    self.gains['left-wrist'],\
                                     #(0.3,0.3,1.1,pi/2,0,-pi/2),'111111')
                                     (0.3785,0.1365,1.07,pi/2,0,-pi/2),'111111')
        self.push(self.taskLH)
        change6dPositionReference(self.taskRF,self.features['right-ankle'],\
                                    self.gains['right-ankle'],\
                                    #(0.015,-0.25,0.2,0,0,0),'111111')
                                    (0.0,-0.1125,0.10,0,0,0),'111111')
        self.push(self.taskRF)
        change6dPositionReference(self.taskLF,self.features['left-ankle'],\
                                    self.gains['left-ankle'],\
                                    #(0.015,0.25,0.2,0,0,0),'111111')
                                    (0.0,0.1125,0.44,0,0,0),'111111')
        self.push(self.taskLF)

    def startOcillation(self):
        self.oscillatorRoll.setActivated(True)
        self.oscillatorPitch.setActivated(True)

    def stopOcillation(self):
        self.oscillatorRoll.setActivated(False)
        self.oscillatorPitch.setActivated(False)

    # --- SEQUENCER ---
    step=0
    def sequencer(self,stepSeq=None):
        if stepSeq!=None:
            self.step=stepSeq
        #-----initial position------
        if self.step==0:
            print "Step : ", self.step
            self.initialStack()
            print('Initial Stack')
            self.step+=1
        elif self.step==1:
            print "Step : ", self.step
            self.goBikeSitting()
            print('Bike Sitting')
            self.step+=1
        elif self.step==2:
            print "Step : ", self.step
            if self.hands:
                self.closeGripper()
            print('Close Gripper')
            self.step+=1
        #-----move start------
        elif self.step==3:
            print "Step : ", self.step
            self.startOcillation()
            print('Start oscillation')
            self.step+=1
        #-----end of move-----
        elif self.step==4:
            print "Step : ", self.step
            self.stopOcillation()
            print('Stop oscillation')
            self.step+=1
        elif self.step==5:
            print "Step : ", self.step
            if self.hands:
                self.openGripper()
            print('Open Gripper')
            self.step+=1
        else:
            print "Step : ", self.step
            self.goHalfSitting()
            print('Half-Sitting')
            self.step+=1
    def __call__(self):
        self.sequencer()
    def __repr__(self): 
        self.sequencer()
        return str()
