#!/usr/bin/env python3

import os, sys, threading
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
# -- ROS modules
import rospy, actionlib, roslaunch
from std_msgs.msg import String, Float32
from actionlib_msgs.msg import GoalStatus
from std_srvs.srv import Trigger, TriggerResponse
# -- project modules
from pi_monitor.pi_tools import PIUTIL
from pi_monitor.system_config import SystemConfig, SystemConfigNames

class PIMonitor():
    def __init__(self, CONFIG):

        # topic for publishing the temperature of the host, only works on rasberry pi
        self.pi_temperature_pub = rospy.Publisher(CONFIG.get(SystemConfigNames.ROS_PI_TEMP_TOPIC), Float32, queue_size=1)
        # start a timer
        self.timer = rospy.Timer(rospy.Duration(CONFIG.get(SystemConfigNames.SYSTEM_TIMER)), self.cb_timer)

    # -- the callback for the shutdown signal
    def on_shutdown(self):
        rospy.loginfo('[PIMonitor] shutdown the camera agent')

    def cb_timer(self, event): 
        # obtain temperature if availabel
        temperature = PIUTIL.get_cpu_temperature()
        if temperature is not None and type(temperature) == float:
            self.pi_temperature_pub.publish(Float32(temperature))   

# ----- the main program
if __name__ == '__main__':
    rospy.init_node('pi_monitor', anonymous=False)
    try:
        CONFIG:SystemConfig = SystemConfig(os.path.join(os.path.dirname(__file__), '../config/system_config.yaml'))
        pi_monitor_agent = PIMonitor(CONFIG)
        rospy.loginfo('[pi_monitor] the node is running')
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)