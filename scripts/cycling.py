#!/usr/bin/env python
#coding:utf-8
from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from dynamic_graph.sot.core.meta_tasks_kine import *
from dynamic_graph.sot.core.meta_tasks import generic6dReference, Task, GainAdaptive
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate,matrixToRPY
from dynamic_graph.sot.core.meta_tasks import *
from dynamic_graph.sot.core.meta_task_posture import MetaTaskPosture, MetaTaskKinePosture
from dynamic_graph import plug
from numpy import *
from dynamic_graph.sot.application.velocity.precomputed_meta_tasks import *
from dynamic_graph.sot.tools import Oscillator, Seqplay
import time
import math
import os
import sys
#toList = lambda sot: map(lambda x: x[2:],sot.display().split('\n')[3:-2])



def degToRad(deg):
    rad=(deg*pi)/180
    return rad

def change6dPositionReference(task,position,selec=None,ingain=None,resetJacobian=True):
    M=generic6dReference(position)
    if selec!=None:
        if isinstance(selec,str):  
            task.feature.selec.value = selec
        else: 
            task.feature.selec.value = toFlags(selec)
    task.featureDes.position.value = matrixToTuple(M)
    
   
    
    setGain(task.gain.setConstant(10),ingain)
    
    if 'resetJacobianDerivative' in task.__class__.__dict__.keys() and resetJacobian:
        task.resetJacobianDerivative()
    
class Hrp2Bike():

    tracesRealTime = True

    initialPose = (
        # Free flyer
        degToRad(0), degToRad(0), degToRad(0), degToRad(0), degToRad(0), degToRad(0),

        # Chest and head
        degToRad(0), degToRad(20), degToRad(0), degToRad(0), 

        #LArms
        degToRad(-70), degToRad(0), degToRad(0), degToRad(0), degToRad(90), degToRad(0), 

        #Rgripper
        degToRad(-40), 

        #RArms
        degToRad(-70),degToRad(0), degToRad(0), degToRad(0), degToRad(-90),degToRad(0),
        
        #LGipper 
        degToRad(-40), 
        
        #LLeg
        degToRad(0), degToRad(0), degToRad(-30), degToRad(60), degToRad(0),degToRad(0),
         
        #RLeg 
        degToRad(0), degToRad(0), degToRad(-70), degToRad(60), degToRad(10), degToRad(0),
        )

    def __init__(self,robot,hands=True):#, forceSeqplay=True):
        self.solver = Solver(robot)
        self.sot= self.solver.sot
        self.hands=hands
        self.robot=robot
#        self.seq=Seqplay('seqplay')
        self.createTasks(robot)
#        self.initTasks()
        self.initTaskGains()
        self.initSolver()
        #self.initialStack()
        self.taskInitialPose = MetaTaskKinePosture(self.robot.dynamic,'initial-pose')
        self.taskHalfSitting = MetaTaskKinePosture(self.robot.dynamic,'halfsitting')
        self.taskGripper     = MetaTaskKinePosture(self.robot.dynamic,'gripper')
        self.taskHalfSitting.gain.setByPoint(2,0.2,0.01,0.8)
        self.taskGripper.gain.setConstant(3)
        self.taskInitialPose.gain.setByPoint(3,3.2,3.01,3.8)
        
        """
        robot.mTasks['com'] = MetaTaskKineCom(robot.dynamic)
        """
        
        """
        self.robot.contactLF = MetaTaskKine6d('contactLF',self.robot.dynamic,'LF',self.robot.OperationalPointsMap['left-ankle'])
        self.contactRF = MetaTaskKine6d('contactRF',self.robot.dynamic,'RF',self.robot.OperationalPointsMap['right-ankle'])
        """

        """
        self.contactRF = MetaTaskKine6d('contactRF',self.robot.dynamic,'RF',self.robot.OperationalPointsMap['right-ankle'])
        self.features = self.robot.contactRF.feature.frame('desired')
        self.gains = self.robot.contactRF.gain.setConstant(10)
        """

    def printSolver(self): #show tasks in solver controling the robot
        print self.solver
       
    def toList(self):
        self.toList()
        
    

    #----------TASKS-------------------------------
    #------------------CREATION--------------------

    def createBalance(self,robot,solver):
        createBalance(robot,solver)   
    
    def createTasks(self,robot):
        createTasks(robot)
        """
     
        (self.tasks['trunk'],self.gains['trunk'])= self.createTrunkTask(robot, 'trunk')
        self.taskRF          = self.tasks['left-ankle']
        self.taskLF          = self.tasks['right-ankle']
        self.taskCom         = self.tasks['com']
        self.taskRH          = self.tasks['right-wrist']
        self.taskLH          = self.tasks['left-wrist']
        self.taskTrunk       = self.tasks['trunk']
        self.taskPosture     = self.tasks['posture']
        self.taskBalance     = self.tasks['balance']
        self.taskWaist       = self.tasks['waist']
        """
    def openGripper(self):
        self.taskGripper.gotoq(None,rhand=(self.gripperOpen,),lhand=(self.gripperOpen,))

    def closeGripper(self):
        if self.hands:
            self.solver.sot.push(self.taskGripper.task.name)
        #self.taskGripper.gotoq(None,rhand=(self.gripperClose,),lhand=(self.gripperClose,))

    #------------------INIT-TASK------------------
    def initTasks(self):
        self.sot.initTaskBalance()
        self.sot.initTaskPosture()
        #if self.sot.hands:
        self.sot.initTaskGripper()
        self.sot.initTaskHalfSitting()
        self.sot.initTaskInitialPose()


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
    
    def initTaskInitialPose(self):
        self.taskInitialPose.gotoq(None,\
                                    rleg =(self.initialPose[6:12]),   \
                                    lleg =(self.initialPose[12:18]),  \
                                    chest=(self.initialPose[18:20]),  \
                                    head =(self.initialPose[20:22]),  \
                                    rarm =(self.initialPose[22:28]),  \
                                    larm =(self.initialPose[29:35]))

    def initTaskPosture(self):
        # --- LEAST NORM
        weight_ff        = 0
        weight_leg       = 3
        weight_knee      = 5
        weight_chest     = 1
        weight_chesttilt = 12
        weight_head      = 0.3
        weight_arm       = 0.8

        weight = diag( (weight_ff,)*6 + (weight_leg,)*12 + (weight_chest,)*2 + (weight_head,)*2 + (weight_arm,)*14)
        weight[9,9] = weight_knee
        weight[15,15] = weight_knee
        weight[19,19] = weight_chesttilt
        #weight = weight[6:,:]

        self.featurePosture.jacobianIN.value = matrixToTuple(weight)
        self.featurePostureDes.errorIN.value = self.robot.halfSitting
        mask = '1'*36
        # mask = 6*'0'+12*'0'+4*'1'+14*'0'
        # mask = '000000000000111100000000000000000000000000'
        # robot.dynamic.displaySignals ()
        # robot.dynamic.Jchest.value
        self.features['posture'].selec.value = mask

    def initTaskGripper(self):
        """
        self.gripperOpen = degToRad(5)
        self.gripperClose = degToRad(3)
        self.closeGripper()
        """
        self.taskGripper.gotoq(None,rhand=(degToRad(-40)),lhand=(degToRad(-40)))

    #------------------INIT-GAIN------------------
    def initTaskGains(self):
        pass
        """
        self.taskHalfSitting.gain.setByPoint(2,0.2,0.01,0.8)
        self.taskGripper.gain.setConstant(3)
        self.taskInitialPose.gain.setByPoint(5,5.2,5.01,5.8)
        """
    #----------SOLVER------------------------------
    def initSolver(self):
        plug(self.sot.control,self.robot.device.control)


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
    def goInitialPose(self):
        self.sot.clear()
#        self.push(self.taskBalance)
#        self.push(self.taskLF)
#        self.push(self.taskRF)
#        self.push(self.taskPosture)
#        if self.hands:
#            self.push(self.taskGripper)
        self.sot.addContact(self.robot.contactRF)
        self.robot.contactRF.featureDes.selec.value = '111111'
        self.robot.contactRF.featureDes.position.value=matrixToTuple(generic6dReference((0.33400191731161255, -0.095, 0.25065344860126393, 0, 0, 0)))

        self.sot.addContact(self.robot.contactLF)
        self.robot.contactLF.featureDes.selec.value = '111111'
        self.robot.contactLF.featureDes.position.value=matrixToTuple(generic6dReference((0, 0.095, 0.1290868250498592, 0, 0, 0)))


        
        self.sot.addContact(self.robot.mTasks['waist'])
        self.robot.contactLF.featureDes.selec.value = '111111'
        self.robot.mTasks['waist'].featureDes.position.value=matrixToTuple(generic6dReference((0, 0, 0.648702, 0, 0, 0)))

        self.sot.addContact(self.robot.mTasks['rh'])
        self.robot.contactLF.featureDes.selec.value = '111111'
        self.robot.mTasks['rh'].featureDes.position.value=matrixToTuple(generic6dReference((0.4844452323115148, -0.25, 0.8453561482158212, 0, 0, 0)))

        self.sot.addContact(self.robot.mTasks['lh'])
        self.robot.contactLF.featureDes.selec.value = '111111'
        self.robot.mTasks['lh'].featureDes.position.value=matrixToTuple(generic6dReference((0.4844452323115148, 0.25, 0.8453561482158212, 0, 0, 0)))
        







#self.solver.sot.push(self.taskInitialPose.task.name)

    def initialStack(self):
        self.sot.clear()
        #self.sot.push(self.taskBalance)
        #self.sot.push(self.taskTrunk)
        #self.sot.push(self.taskPosture)
        if self.hands:
            self.solver.sot.push(self.taskGripper)

    def goHalfSitting(self):
        self.sot.clear()
        self.sot.push(self.taskBalance.task.name)
        self.sot.push(self.taskPosture.task.name)
        if self.hands:
            self.sot.push(self.taskGripper.task.name)
        self.sot.push(self.taskHalfSitting.name)

    #------------parameters of bike-------
    xc=0.21; zc=-0,121566
     #center of crank gear
    R=1.5 #pedal-center of crank gear


    def taskRightFoot(self):
        
        self.sot.addContact(self.robot.contactRF)
        self.robot.contactRF.featureDes.selec.value = '111111'
        self.robot.contactRF.featureDes.position.value=matrixToTuple(generic6dReference((0.5,0,0,0,0,0)))
        
        """
        self.robot.dynamic.com.recompute(0)
        self.robot.mTasks['com'].featureDes.errorIN.value = self.robot.dynamic.com.value
        self.robot.mTasks['com'].task.controlGain.value = 10
        self.robot.mTasks['com'].feature.selec.value = '011'
        self.solver.sot.push(self.taskCom.task.name)
        """
    """
    def taskLeftFoot(self):
        self.sot.addContact(self.robot.contactLF)
        self.robot.contactLF.feature.position.value=matrixToTuple(generic6dReference((0.6,0.9,0,0,0,0)))
        self.robot.contactLF.feature.selec.value = '111111'
    """

    def goBikeSitting(self):
        pass
        #self.clear()
#        self.push(self.taskBalance)
#        self.push(self.tasks['chest'])

        """
        change6dPositionReference(self.taskWaist,self.features['waist'],\
                                    self.gains['waist'],\
                                    (-0.22,0.0,0.58,0,0,0),'111111')
        self.push(self.taskWaist)
        if self.hands:
            self.push(self.taskGripper)
          
        change6dPositionReference(self.taskRH,self.features['right-wrist'],\
                                    self.gains['right-wrist'],\
                                    (0.25,-0.255,0.90,-pi/2,0,pi/2),'111111')
        self.push(self.taskRH)
        
        change6dPositionReference(self.taskLH,self.features['left-wrist'],\
                                    self.gains['left-wrist'],\
                                     (0.25,0.255,0.90,pi/2,0,-pi/2),'111111')
        self.push(self.taskLH)
        change6dPositionReference(self.taskRF,self.features['right-ankle'],\
                                    self.gains['right-ankle'],\
                                    (0.0,-0.1125,0.12,0,0,degToRad(-8)),'111111')
        self.push(self.taskRF)
        change6dPositionReference(self.taskLF,self.features['left-ankle'],\
                                    self.gains['left-ankle'],\
                                    (0.0,0.1125,0.46,0,0,degToRad(8)),'111111')
        self.push(self.taskLF)
        self.push(self.taskPosture)
        
    def stopMove(self):
        self.rotation=False
        """

    def InitCircle(self,nbPoint=16,rotation=True): #nbPoint=number of point to dicretise the circle
           
        
        
        self.rotation=rotation
        self.stepCircle=(2*pi)/nbPoint
        circleL=[]
        circleR=[]

        for j in range(0,nbPoint):
            #circleL.append(-(j*self.stepCircle)+(pi/2))
            #circleR.append(-(j*self.stepCircle)+(3*pi/2))
            
            circleL.append(-(j*(2*pi)/nbPoint)-5*pi/6)
            circleR.append(-(j*(2*pi)/nbPoint)+pi/6)

        #self.sot.addContact(self.robot.contactRF)
        #self.robot.contactRF.featureDes.selec.value = '111111'
        #self.sot.addContact(self.robot.contactLF)
        #self.robot.contactLF.featureDes.selec.value = '111111'
        
        xc=0.15
        zc=0.2
        R=0.13
        
        for i in range(0,nbPoint):
            xpR=((cos(circleR[i])*R)+xc)
            zpR=((sin(circleR[i])*R)+zc)
            xpL=((cos(circleL[i])*R)+xc)
            zpL=((sin(circleL[i])*R)+zc)
            print ('la valeur de xpL est ' ,format(xpL))
            print ('la valeur de zpL est ' ,format(zpL))
            self.robot.contactRF.featureDes.position.value=matrixToTuple(generic6dReference((xpR,-0.095,zpR,0,0,0)))
            self.robot.contactLF.featureDes.position.value=matrixToTuple(generic6dReference((xpL,0.095,zpL,0,0,0)))
            time.sleep(1)    
        
        

        """
        change6dPositionReference(self.taskWaist,self.features['waist'],\
                                    self.gains['waist'],\
                                    (-0.22,0.0,0.58,0,0,0),'111111')
        
        self.push(self.taskWaist)
        if self.hands:
            self.push(self.taskGripper)

        """
        #change6dPositionReference(self.robot.contactLF,x,'111111')
        #self.solver.sot.addContact(self.robot.contactLF)
        #self.solver.sot.push(self.robot.contactLF.task.name)
        

    def Circle(self):

        #self.solver.sot.addContact(self.robot.contactLF)
        #self.solver.sot.addContact(self.robot.contactRF)
        #self.sot.clear()
        #self.solver.sot.push(self.robot.contactLF.task.name)
        #self.solver.sot.push(self.robot.contactRF.task.name)
        
        
        """
        change6dPositionReference(self.sot.contactLF,self.sot.features['left-ankle'],\
                                        self.gains['left-ankle'],\
                                        (0.7,0.1125,-0.8,0,0,degToRad(8)),'111111')
        
        self.solver.sot.push(self.contactLF.task.name)
        
        
        
        #--------rotation--------
        self.push(self.taskRF)
        self.push(self.taskLF)
        #while self.rotation:
        for i in range(0,nbPoint):
            xpL=(cos(circleL[i])*self.R)+self.xg
            zpL=(sin(circleL[i])*self.R)+self.zg
            xpR=(cos(circleR[i])*self.R)+self.xg
            zpR=(sin(circleR[i])*self.R)+self.zg
            change6dPositionReference(self.taskRF,self.features['right-ankle'],\
                                        self.gains['right-ankle'],\
                                        (xpR,-0.1125,zpR,0,0,degToRad(-8)),'111111')
            change6dPositionReference(self.taskLF,self.features['left-ankle'],\
                                        self.gains['left-ankle'],\
                                        (xpL,0.1125,zpL,0,0,degToRad(8)),'111111')
            time.sleep(5)
        """   
    #--- SEQUENCER ---
        
    step = 0
    def sequencer(self,stepSeq=None):
        if stepSeq!=None:
            self.step=stepSeq
        #-----initial position------
        if self.step==0:
            print "Step : ", self.step
            self.initTaskInitialPose()
            self.goInitialPose()
            print('Initial Pose')
            self.step+=4
        elif self.step==1:
            print "Step : ", self.step
            self.goBikeSitting()
            print('Bike Sitting')
            self.step+=0
            
        elif self.step==2:
            print "Step : ", self.step
            if self.hands:
                self.initTaskGripper()
                self.closeGripper()
            print('Close Gripper')
            
            self.step+=0
        #-----move start------
        elif self.step==3:
            print "Step : ", self.step
            self.taskRightFoot()
            #self.taskLeftFoot()
            #self.InitCircle()
            #self.Circle()
            print('Start movement Foot')
            self.step+=0

        elif self.step==4:
            print "Step : ", self.step
            self.InitCircle()
            #self.taskLeftFoot()
            #self.InitCircle()
            #self.Circle()
            print('Start Cycling')
            self.step+=0
        #-----end of move-----
        #elif self.step==4:
            #print "Step : ", self.step
            #self.stopMove()
            #print('Stop movement')
           # self.step+=1
        elif self.step==7:#5:
            print "Step : ", self.step
            #if self.hands:
            #    self.openGripper()
            #print('Open Gripper')
            self.step+=1
        else:
            print "Step : ", self.step
            #self.initTaskHalfSitting()
            #self.goHalfSitting()
            print('Half-Sitting')
            self.step+=1
            
    def __call__(self):
        self.sequencer()
    def __repr__(self): 
        self.sequencer()
        return str()
