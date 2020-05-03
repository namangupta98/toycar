cmake_minimum_required(VERSION 2.8.3)
project(toycar)

#add_compile_options(-std=c++11)

# Find catkin packages
find_package(catkin REQUIRED COMPONENTS
	roslaunch
	tf
	nav_msgs
	roscpp
	rospy
	std_msgs
	geometry_msgs 
)

# Declare a catkin package
catkin_package(
	INCLUDE_DIRS
	CATKIN_DEPENDS rospy geometry_msgs
	DEPENDS
)

# Build target library
include_directories( 
  include
  ${catkin_INCLUDE_DIRS})

foreach(dir config launch meshes urdf)
	
install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(PROGRAMS
  scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Specify libraries to link a library or executable target against

