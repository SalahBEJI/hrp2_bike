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


# Launch geometric simu
terminal.extend(['--tab-with-profile=HoldOnExit', '-e','''
bash -c '
echo "RVIZ, geometric_simu"
sleep 10
roslaunch hrp2_bike hrp2_bike.launch
'
''' % locals(), '-t', '''Launch geometric simu'''])


# Set the size of the terminal
terminal.extend(['''--geometry=195x50+0+0'''])

subprocess.call(terminal)






