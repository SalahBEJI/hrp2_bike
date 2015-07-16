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
#        0.10, -0.25, 0.85, 0., -pi/2 , 0., 0.1,
#        0.10,  0.25, 0.85, 0., -pi/2 , 0., 0.1,
        0.261799, -0.17453, 0., -0.523599, 0., 0., 0.1,
        0.261799, 0.17453,  0., -0.523599, 0., 0., 0.1,
        )

    def __init__(self,robot):#, forceSeqplay=True):
        Application.__init__(self,robot)

        self.sot=self.solver.sot
#        self.robot=robot
        self.seq=Seqplay('seqplay')
#        self.forceSeqplay=forceSeqplay
#        if self.forceSeqplay:
#            self.zmpRef=ZmpFromForces('zmpRef')
#        else:
#            self.zmpRef=self.seq
        self.createTasks(robot)
        self.initTasks()
        self.initTaskGains()
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
        self.initTaskGripper()
        self.initTaskHalfSitting()
        self.initTaskBikeSitting()
#        self.initSeqplay()

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

#    def initSeqplay(self):
#        if self.forceSeqplay:
#            plug(self.seq.forceLeftFoot, self.zmpRef.force_0)
#            plug(self.seq.forceRightFoot, self.zmpRef.force_1)
#            plug (self.robot.frames['leftFootForceSensor'].position , self.zmpRef.sensorPosition_0)
#            plug (self.robot.frames['rightFootForceSensor'].position, self.zmpRef.sensorPosition_1)

#        plug (self.zmpRef.zmp , self.robot.device.zmp)

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
        self.push(self.taskGripper)

    def goHalfSitting(self):
        self.sot.clear()
        self.push(self.taskBalance)
        self.push(self.taskPosture)
        self.push(self.taskGripper)
        self.push(self.taskHalfSitting)


    def goBikeSitting(self):
        self.sot.clear()
        self.push(self.taskBalance)
        self.push(self.taskPosture)
        self.push(self.taskGripper)
        self.push(self.taskBikeSitting)

    def prepareSeqplay(self):
        self.seq.leftAnkle.recompute(2)  
        self.seq.rightAnkle.recompute(2) 
        self.seq.com.recompute(2) 
        self.seq.comdot.recompute(2) 
        self.seq.leftAnkleVel.recompute(2) 
        self.seq.rightAnkleVel.recompute(2)
        self.seq.posture.recompute(2)

        
        plug (self.seq.leftAnkle, self.leftAnkle.reference)
        plug (self.seq.rightAnkle, self.rightAnkle.reference)

        plug (self.seq.com, self.comRef)
        plug (self.seq.com, self.featureComDes.errorIN)

        plug (self.seq.comdot, self.comdot)
        plug (self.seq.comdot, self.featureComDes.errordotIN)

        plug (self.seq.leftAnkleVel, self.leftAnkle.velocity)
        plug (self.seq.rightAnkleVel, self.rightAnkle.velocity)
        plug (self.seq.posture, self.featurePostureDes.errorIN)

    def runSeqplay(self):
        self.seq.start ()

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
            self.closeGripper()
            print('Close Gripper')
            self.step+=1
        #-----seqplay preparation------
        elif self.step==3:
            print "Step : ", self.step
            self.prepareSeqplay()
            print ('Seqplay prepared')
        #-----move start------
        elif self.step==4:
            print "Step : ", self.step
            self.runSeqplay()
            print('Seqplay run')
            self.step+=1
        #-----end of move-----
        elif self.step==5:
            print "Step : ", self.step
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
