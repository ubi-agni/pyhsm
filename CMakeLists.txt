cmake_minimum_required(VERSION 3.1.3)
project(pyhsm)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rostest
  pyhsm_msgs
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS pyhsm_msgs
)

# Install targets
install(PROGRAMS scripts/viewer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  ## Add folders to be run by python nosetests
  catkin_add_nosetests(test)
endif()
