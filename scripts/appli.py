from dynamic_graph.sot.application.cycling import Hrp2Bike
from dynamic_graph.script_shortcuts import optionalparentheses

hrp2Bike = Hrp2Bike(robot)

hrp2Bike.withTraces()

# --- SHORTCUTS

push              = hrp2Bike.push
sot               = hrp2Bike.sot
taskBikeSitting   = hrp2Bike.taskBikeSitting
taskHalfSitting   = hrp2Bike.taskHalfSitting
removeTasks       = hrp2Bike.removeTasks

tr           = hrp2Bike.robot.tracer
gopen        = optionalparentheses(hrp2Bike.openGripper)
gclose       = optionalparentheses(hrp2Bike.closeGripper)
gohs         = optionalparentheses(hrp2Bike.goHalfSitting)
gobs         = optionalparentheses(hrp2Bike.goBikeSitting)

s = hrp2Bike.sequencer
t = optionalparentheses(hrp2Bike.trace)
