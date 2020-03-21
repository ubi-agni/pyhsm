#!/usr/bin/env python
"""
Connect to ROS and start the HSM viewer.
"""

import sys
import rospy
from hsm.viewer import *
from hsm.viewer.gui import Gui

rospy.init_node('hsm_viewer', anonymous=False, disable_signals=True, log_level=rospy.INFO)
sys.argv = rospy.myargv()

gui = Gui(width=720, height=480)
Gtk.main()
