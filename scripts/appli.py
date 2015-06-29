from dynamic_graph.sot.application.cycling import Hrp2Bike, Sequencer
from dynamic_graph.script_shortcuts import optionalparentheses

hrp2Bike = Hrp2Bike(robot)
sequencer = Sequencer(hrp2Bike)

# --- SHORTCUTS

push              = hrp2Bike.push
sot               = hrp2Bike.sot
taskBikeSitting   = hrp2Bike.taskBikeSitting
taskHalfSitting   = hrp2Bike.taskHalfSitting
removeTasks       = hrp2Bike.removeTasks

gopen        = optionalparentheses(appli.openGripper)
gclose       = optionalparentheses(appli.closeGripper)
gohs         = optionalparentheses(appli.goHalfSitting)
gobs         = optionalparentheses(appli.goBikeSitting)

s = sequencer
