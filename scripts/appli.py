from dynamic_graph.sot.application.cycling import Hrp2Bike
from dynamic_graph.script_shortcuts import optionalparentheses

hrp2Bike = Hrp2Bike(robot)

hrp2Bike.withTraces()

# --- SHORTCUTS

#push              = hrp2Bike.push
sot               = hrp2Bike.sot
taskInitialPose   = hrp2Bike.taskInitialPose
taskHalfSitting   = hrp2Bike.taskHalfSitting
removeTasks       = hrp2Bike.removeTasks

tr           = hrp2Bike.robot.tracer
goip         = optionalparentheses(hrp2Bike.goInitialPose)
gclose       = optionalparentheses(hrp2Bike.closeGripper)
gohs         = optionalparentheses(hrp2Bike.goHalfSitting)
gopen        = optionalparentheses(hrp2Bike.openGripper)
gobs         = optionalparentheses(hrp2Bike.goBikeSitting)


s = hrp2Bike.sequencer
t = optionalparentheses(hrp2Bike.trace)
