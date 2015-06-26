from dynamic_graph.sot.core.meta_tasks import generic6dReference, Task, GainAdaptive
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_tasks import *
from dynamic_graph.sot.core.meta_task_posture import MetaTaskPosture
from dynamic_graph import plug
from numpy import *
from dynamic_graph.sot.application.velocity.precomputed_tasks import Application
from dynamic_graph.sot.tools import Oscillator, Seqplay

toList = lambda sot: map(lambda x: x[2:],sot.display().split('\n')[3:-2])

class Hrp2Bike(Application):

    def __init__(self,robot):
        Application.__init__(self,robot)

        self.sot=self.solver.sot
        self.robot=robot

        self.initTasks(robot)
        self.initTaskGains()
        self.initialStack()

    def printSolver(self): #show tasks in solver controling the robot
        print self.solver


    #----------TASKS-------------------------------
    def initTasks(self,robot):
        self.initTaskBalance()
        self.createBikeSittingTask(robot, 'bike-sitting')
        self.createTasks(robot)

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

    def createBikeSittingTask (self, robot, taskName, ingain=1.0):
        task = Task (taskName)
        task.add (self.features['left-ankle'].name)
        task.add (self.features['right-ankle'].name)
        task.add (self.features['com'].name)
        task.add (self.features['right-wrist'].name)
        task.add (self.features['left-wrist'].name)
        gain = GainAdaptive(taskName+'gain')
        gain.setConstant(ingain)
        plug(gain.gain, task.controlGain)
        plug(task.error, gain.error)
        return (task, gain)

    def initTaskBalance(self):
        # --- BALANCE ---
        self.features['chest'].frame('desired')
        self.features['waist'].frame('desired')
        self.features['gaze'].frame('desired')
        self.features['chest'].selec.value = '111000'
        self.features['waist'].selec.value = '111000'
        self.features['gaze'].selec.value = '111000'
        self.featureCom.selec.value = '111'

    def createTasks(self,robot):
        (self.tasks['trunk'],self.gains['trunk'])= self.createTrunkTask(robot, 'trunk')
        (self.tasks['bike-sitting'],self.gains['bike-sitting'])= self.createBikeSittingTask(robot, 'bike-sitting')
        self.taskRF      = self.tasks['left-ankle']
        self.taskLF      = self.tasks['right-ankle']
        self.taskCom     = self.tasks['com']
        self.taskRH      = self.tasks['right-wrist']
        self.taskLH      = self.tasks['left-wrist']
        self.taskTrunk   = self.tasks['trunk']
        self.taskHalfSitting = MetaTaskPosture(self.robot.dynamic,'halfsitting')
        self.taskPosture = self.tasks['posture']
        self.taskBalance = self.tasks['balance']
        self.taskBikeSitting = self.tasks['bike-sitting']


    def initTaskGains(self):
        self.taskHalfSitting.gain.setByPoint(2,0.2,0.01,0.8)


    #----------SOLVER------------------------------

    def push(self,task,feature=None): #push task in solver 
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
            self.sot.remove(taskName)

    def printSolver(self): #show tasks in solver controling the robot
        print self.solver

    def showTasks(self) : #show library of precomputed tasks
        self.tasks


    #----------RUN---------------------------------

    def initialStack(self):
        self.sot.clear()
        self.push(self.taskBalance)
        self.push(self.taskTrunk)
        self.push(self.taskPosture)

    def goHalfSitting(self):
        self.featurePostureDes.errorIN.value = self.robot.halfSitting
        self.sot.clear()
        self.push(self.taskBalance)
        self.push(self.taskPosture)

    def bikeSitting(self):
        self.sot.clear()
        self.push(self.taskBikeSitting)


    # --- SEQUENCER ---
class Sequencer:
    step=0
    def __init__(self,hrp2Bike):
        self.hrp2Bike = hrp2Bike

    def nextStep(self,step=None):
        if self.step==0:
            self.hrp2Bike.goHalfSitting
            self.step+=1
            print self.step
        if self.step==1:
            self.hrp2Bike.bikeSitting
            self.step+=1
            print self.step
        else:
            self.hrp2Bike.goHalfSitting
            self.step+=1
            print self.step

    def __call__(self):
        self.nextStep()
    def __repr__(self): 
        self.nextStep()
        return str()
