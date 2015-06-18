#!/usr/bin/env python

from time import sleep
import subprocess
import sys

terminal = ['gnome-terminal']

# Start roscore in a terminal
terminal.extend(['--tab-with-profile=HoldOnExit', '-e','''
bash -c '
echo "launch roscore"
roscore
'
''' % locals(), '-t', '''roscore'''])

# Start Rviz
terminal.extend(['--tab-with-profile=HoldOnExit', '-e','''
bash -c '
echo "Start Rviz"
rosrun rviz rviz
'
''' % locals(), '-t', '''Start Rviz'''])

# Launch geometric simu
terminal.extend(['--tab-with-profile=HoldOnExit', '-e','''
bash -c '
echo "geometric_simu, add a robot model"
sleep 10
roslaunch hrp2_bringup geometric_simu.launch
'
''' % locals(), '-t', '''Launch geometric simu'''])


# Start dynamic_graph_bridge run_command terminal
terminal.extend(['--tab-with-profile=HoldOnExit', '-e','''
bash -c '
echo "run command :"
echo "from dynamic_graph.sot.application.velocity.precomputed_tasks import Application"
echo "appli =Application(robot)"
echo "robot.initializeTracer()"
sleep 20
rosrun dynamic_graph_bridge run_command
'
''' % locals(), '-t', '''Run Command'''])


# Start dynamic graph
terminal.extend(['--tab-with-profile=HoldOnExit', '-e','''

bash -c '
echo "start dynamic graph"
sleep 60
rosservice call /start_dynamic_graph
'
''' % locals(), '-t', '''Start dynamic graph'''])

# Set the size of the terminal
terminal.extend(['''--geometry=195x50+0+0'''])

subprocess.call(terminal)






