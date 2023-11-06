cmake_minimum_required(VERSION 3.10.2)
project(pyhsm)

find_package(catkin REQUIRED)

# See https://docs.ros.org/en/melodic/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

catkin_package()

# Install targets
catkin_install_python(PROGRAMS scripts/viewer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  ## Add folders to be run by python nosetests
  catkin_add_nosetests(test)
endif()
