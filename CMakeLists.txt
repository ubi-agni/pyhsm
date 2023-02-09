cmake_minimum_required(VERSION 3.1.3)
project(pyhsm)

find_package(catkin REQUIRED)

# See https://docs.ros.org/en/melodic/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

catkin_package()

# Install targets
install(PROGRAMS scripts/viewer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  ## Add folders to be run by python nosetests
  catkin_add_nosetests(test)
endif()
