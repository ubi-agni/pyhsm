#!/usr/bin/env python
"""
Connect to ROS and start the HSM viewer.
"""

import signal
import sys
import rospy

from hsm.viewer.gtk_wrap import Gtk
from hsm.viewer.main_window import MainWindow


rospy.init_node('hsm_viewer', anonymous=False, disable_signals=True, log_level=rospy.INFO)
sys.argv = rospy.myargv()

win = MainWindow(width = 720, height = 480)
win.connect('destroy', Gtk.main_quit)

if sys.platform != 'win32':
    # Reset KeyboardInterrupt SIGINT handler,
    # so that glib loop can be stopped by it
    signal.signal(signal.SIGINT, signal.SIG_DFL)

Gtk.main()
