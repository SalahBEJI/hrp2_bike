#!/usr/bin/python2.7
# -*- coding: utf-8 -*
import os
from shutil import move



def main():
	print("-----------Dimensions of the ergocycle--------------")
	height_saddle=0
	height_handlebar=0
	lenght_handlebar=0
	while (65 > height_saddle or height_saddle > 440):
		height_saddle = raw_input("Height of the saddle in mm [65-440]:")
		height_saddle=int(height_saddle)

	while (155 > height_handlebar or height_handlebar > 370):
		height_handlebar = raw_input("Height of the handlebar in mm [155-370]:")
		height_handlebar=int(height_handlebar)

	while (0 > lenght_handlebar or lenght_handlebar > 300):
		lenght_handlebar = raw_input("Lenght of the handlebar in mm [0-300]:")
		lenght_handlebar = int(lenght_handlebar)

	height_saddle=-float(440-height_saddle)/1000
	height_handlebar=-float(370-height_handlebar)/1000
	lenght_handlebar=float(300-lenght_handlebar)/1000
	
	i=0
	new_file=open('ergocycle_temp.txt','w')
	old_file=open('ergocycle_main.wrl','r')
	while i<386:
		for i in range(0,308):
			i=i+1
			line=old_file.readline()
			new_file.write(str(line))
		line=old_file.readline()
		string='        translation '+str(-height_saddle/2.2)+' 0.0 '+str(height_saddle)+' #modif origin of saddle'+'\n'
		new_file.write(string)
		for i in range(309,327):
			i=i+1
			line=old_file.readline()
			new_file.write(str(line))
		line=old_file.readline()
		string='        translation '+str(height_handlebar/4.3)+' 0.0 '+str(height_handlebar)+' #modif origin of supp_handlebar'+'\n'
		new_file.write(string)
		for i in range(328,344):
			i=i+1
			line=old_file.readline()
			new_file.write(str(line))
		line=old_file.readline()
		string='		translation '+str(lenght_handlebar)+' 0.0 0.0 #modif origin of handlebar (x=[0.0,0.3])'+'\n'
		new_file.write(string)
		for i in range(345,386):
			i=i+1
			line=old_file.readline()
			new_file.write(str(line))



	old_file.close()
	os.remove('ergocycle_main.wrl')
	move("ergocycle_temp.txt", "ergocycle_main.wrl")

if __name__=='__main__':
	main()






















