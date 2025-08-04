#!/usr/bin/env python3

# Copyright 2023 - Andrew Kwok Fai LUI, Centre for Robotics
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2023, The CGRAS Project'
__license__ = 'GPL'
__version__ = '0.0.1'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

# general modules
import os, json, random, math, threading, signal, sys, time, yaml
from enum import Enum
import cv2
import numpy as np
import rospy, actionlib
import message_filters
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String, Bool, Int8, Int32
from cv_bridge import CvBridge
# project modules
from cgras_pi_monitor.msg import TestSnapFeedback, TestSnapGoal, TestSnapResult, TestSnapAction
from cgras_datatools.logging_tools import logger
from cgras_datatools.opencv_tools import GeneralTools 


class CGRASTestImageServer():

    def __init__(self, **kwargs):
        # create the stop signal handler
        signal.signal(signal.SIGINT, self.stop)
        rospy.on_shutdown(self.cb_shutdown)
        # topic naming configuration
        self.camera_snap_topic = '/cgras/test/camera/snap'
        # topic for liveview
        self.liveview_topic = rospy.get_param('~camera_liveview_topic', kwargs.get('camera_liveview_topic', '/cgras/test/camera/liveview'))
        # set constant
        self.TIMEOUT_DURATION = 90.0 # seconds

        # load sample image
        try:
            image_path = os.path.expanduser('~/cgras_data/RealTileImage.jpg')
            self.sample_image = cv2.imread(image_path)
            self.scaled_sample_image = cv2.resize(self.sample_image, (0, 0), fx=0.2, fy=0.2)

        except Exception as e:
            logger.error(f'CameraAgent: failed to load sample image at {image_path}')
            return    
        
        # synchronization lock
        self.task_lock = threading.RLock()
        # setup cv bridge
        self.cv_bridge = CvBridge()
        # create action server: Move.action
        self.action_server_snap = actionlib.SimpleActionServer(self.camera_snap_topic, 
                            TestSnapAction, execute_cb=self.received_snap_goal, auto_start=False)
        self.action_server_snap.start()    
        
        self.liveview_pub = rospy.Publisher(self.liveview_topic, Image, queue_size=1)
        # starts the timer at the end
        self.timer = rospy.Timer(rospy.Duration(1), self.cb_timer)

    # callback function for shutdown
    def cb_shutdown(self):
        logger.info('CameraAgent: the ros node is being shutdown')
        # sys.exit(0)

    # callback function for the interrupt signal SIGINT
    def stop(self, *args, **kwargs):
        logger.info('CameraAgent: the ros node is being stopped')
        sys.exit(0)

    # callbact function for the ros timer function
    def cb_timer(self, event):
        ...
        # publish the liveview
        self.liveview_pub.publish(self.cv_bridge.cv2_to_imgmsg(self.scaled_sample_image, encoding='bgr8'))
        # self.liveview_pub.publish(self.create_image_msg(self.image))

    # callback for receiving the goal of action
    def received_snap_goal(self, goal:TestSnapGoal):
        result = TestSnapResult()
        with self.task_lock:
            # differentiate the goal types and create the corresponding tasks with parameters obtained from the goal
            if goal.command == 'capture':
                tank_id = goal.tank_id
                tile_x, tile_y = goal.tile_x, goal.tile_y
                capture_x, capture_y = goal.grid_x, goal.grid_y
                logger.warning(f'CGRASTestImageServer: received capture goal {tank_id} tile ({tile_x}, {tile_y}) catpure ({capture_x},{capture_y})')
                # publish feedback that the snap is done
                feedback_msg = TestSnapFeedback()
                self.action_server_snap.publish_feedback(feedback_msg) 
                
                USE_COMPRESSED = True
                if USE_COMPRESSED:
                    msg = CompressedImage()
                    msg.header.stamp = rospy.Time.now()
                    msg.format = 'jpeg'
                    # msg.data = np.array(cv2.imencode('.jpg', self.sample_image)[1]).tostring()
                    msg.data = np.array(cv2.imencode('.jpg', self.sample_image)[1]).tobytes()
                    result.cimage = msg
                else:
                    result.image = self.cv_bridge.cv2_to_imgmsg(self.sample_image, encoding='bgr8')
                
                # build metadata
                metadata = {'success': True, 'path': '/test/path', 'original_filename': f'big_prawn_{capture_x}_{capture_y}.jpg', 
                            'suffix': 'jpg', 'length': 0, 'f-number': 8.0, 'shutter-speed': '1/1000', 'iso': 2000,
                            'tank_id': tank_id, 'tile_x': goal.tile_x, 'tile_y': goal.tile_y, 'capture_x': capture_x, 'capture_y': capture_y,
                            'camera_model': 'Simulated Camera', 'camera_id': 'A1234', 'device_overheating_state': 'NO Overheating',
                            }
                # gemerate a random focus_position_current_value
                focus_position_list = [15, 24, 23, 24, 25, 26, 25, 24, 25, 45]
                metadata['focus_position_current_value'] = 1 / random.choice(focus_position_list) * 1e6
                # notify the client
                result.metadata = json.dumps(metadata)
                if self.action_server_snap.is_preempt_requested():
                    logger.warning(f'CGRASTestImageServer (snap goal): preemption received')
                    self.action_server_snap.set_preempted(result)
                else:
                    logger.warning(f'CGRASTestImageServer: (snap goal): succeed')
                    self.action_server_snap.set_succeeded(result)
            else:
                logger.warning(f'Test snap action command ({goal.command}) is not recognized')
     

# ----- the main program
if __name__ == '__main__':
    rospy.init_node('cgras_test_image_server_node', anonymous=False)
    try:
        rospy.loginfo('cgras_test_image_server_node is running')
        robot_agent = CGRASTestImageServer()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)