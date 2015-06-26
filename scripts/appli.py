from dynamic_graph.sot.application.cycling import Hrp2Bike, Sequencer

hrp2Bike = Hrp2Bike(robot)
sequencer = Sequencer(hrp2Bike)

robot.initializeTracer()
# --- SHORTCUTS

push              = hrp2Bike.push
taskHalfSitting   = hrp2Bike.taskHalfSitting
sot               = hrp2Bike.sot
taskBikeSitting   = hrp2Bike.taskBikeSitting


s = sequencer













