from dynamic_graph.sot.hrp2_14.robot import Robot
from dynamic_graph.sot.application.cycling import Hrp2Bike

robot = Robot( 'robot' )
hrp2Bike = Hrp2Bike(robot)
robot.initializeTracer()
# --- SHORTCUTS

push              = hrp2Bike.push
taskHalfStitting  = hrp2Bike.taskHalfStitting
sot               = hrp2Bike.sot
taskBikeSitting   = hrp2Bike.taskBikeSitting
















