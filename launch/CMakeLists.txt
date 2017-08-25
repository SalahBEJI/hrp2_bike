# Generate the configure file for openhrp2_launch
	
configure_file( ${CMAKE_SOURCE_DIR}/launch/openhrp3_hrp2_bike_nobush.launch.in ${CMAKE_BINARY_DIR}/launch/openhrp3_hrp2_bike_nobush.launch)
install(FILES 
  ${CMAKE_BINARY_DIR}/launch/openhrp3_hrp2_bike_nobush.launch 
  DESTINATION 
  ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)