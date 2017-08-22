# Launch Model Loader corba server
rm /tmp/openhrp-model-loader.log
export HRPMODEL_CUSTOMIZER_PATH=/opt/openrobots/lib/customizer
export BUSH_CUSTOMIZER_CONFIG_PATH=/opt/openrobots/share/customizer/HRP2_bush_customizer_param.conf
echo "HRPMODEL_CUSTOMIZER_PATH=" $HRPMODEL_CUSTOMIZER_PATH
echo "BUSH_CUSTOMIZER_CONFIG_PATH=" $BUSH_CUSTOMIZER_CONFIG_PATH
/opt/ros/indigo/bin/openhrp-model-loader &> /tmp/openhrp-model-loader.log &

# Launch Collision detector corba server
rm /tmp/openhrp-collision-detector.log
/opt/ros/indigo/bin/openhrp-collision-detector &> /tmp/openhrp-collision-detector.log &

# Launch Dynamics simulator corba server
rm /tmp/openhrp-aist-dynamics-simulator.log
/opt/openrobots/bin/openhrp-aist-dynamics-simulator &> /tmp/openhrp-aist-dynamics-simulator.log &

# Launch Online viewer corba server
rm /tmp/gepetto-online-viewer.log
/opt/ros/indigo/bin/hrpsys-viewer &> /tmp/hrpsys-online-viewer.log &

# Launch SoT-controller corba server
/opt/openrobots/bin/controller-hrp2 /opt/openrobots/lib/libsot-hrp2-14-controller.so /opt/openrobots/example/openhrp3-hrp2/scheduler/etc/PIDgains.dat > /tmp/controller-hrp2.log &
/bin/sleep 5

# Launch Scheduler of the simulation
/opt/openrobots/example/openhrp3-simulator-wo-rtm/scheduler/schedulerproject  -url /home/sbeji/ergocycle_modelisation/ergocycle_dimention/simulationErgocycle.xml 


