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
from datetime import datetime
from enum import Enum
import cv2
import numpy as np
import rospy, actionlib
import message_filters
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Int8, Int32
from cv_bridge import CvBridge
# project modules
from cgras_pi_monitor.msg import TestSnapFeedback, TestSnapGoal, TestSnapResult, TestSnapAction
from cgras_datatools.logging_tools import logger
from cgras_datatools.opencv_tools import GeneralTools 


class CGRASTestImageClient():

    def __init__(self, **kwargs):
        # create the stop signal handler
        signal.signal(signal.SIGINT, self.stop)
        rospy.on_shutdown(self.cb_shutdown)
        # topic naming configuration
        self.camera_snap_topic = '/cgras/test/camera/snap'
        # topic for liveview
        self.liveview_topic = rospy.get_param('~camera_liveview_topic', kwargs.get('camera_liveview_topic', '/cgras/test/camera/liveview'))
        # output folder for images
        self.output_image_folder = os.path.expanduser('~/cgras_data/ros_image_test')
        os.makedirs(self.output_image_folder, exist_ok=True)
        # set constant
        self.TIMEOUT_DURATION = 30.0 # seconds
        # load sample image
        try:
            image_path = os.path.expanduser('~/cgras_data/RealTileImage.jpg')
            self.sample_image = cv2.imread(image_path)
            self.scaled_sample_image = cv2.resize(self.sample_image, (0, 0), fx=0.2, fy=0.2)
        except Exception as e:
            logger.error(f'CameraAgent: failed to load sample image at {image_path}')
            return    
        # initialize timing variables for run_experiment
        self.start_time = self.finished_call_time = self.feedback_time = self.done_time = self.ready_time = self.saved_time = None
        self.has_done = False
        # synchronization lock
        self.task_lock = threading.RLock()
        # setup cv bridge
        self.cv_bridge = CvBridge()
        # create action server: Move.action
        self.snap_action_client = actionlib.SimpleActionClient(self.camera_snap_topic, TestSnapAction)
        result = self.snap_action_client.wait_for_server(timeout=rospy.Duration(5))
        logger.warning(f'wait for server {result}')
        # mode
        self.USE_COMPRESSED = True
        # run experiment
        self.run_experiment()
        
    def run_experiment(self, repeats=1):
        for exp_n in range(repeats):
            logger.info(f'Running experiment {exp_n}')
            self.feedback_time = self.done_time = self.ready_time = self.saved_time = None
            self.start_time = time.time()
            self.call_camera_to_capture(0, 0, 0, 0, exp_n)
            self.has_done = False
            self.finished_call_time = time.time()
            while not self.has_done:
                rospy.sleep(rospy.Duration(secs=0, nsecs=10000))
                current_time = time.time()
                if current_time - self.finished_call_time > self.TIMEOUT_DURATION:
                    logger.warning(f'timeout: {current_time - self.finished_call_time} secs')
                    break
            if not self.has_done:
                continue
            logger.info(f'From start to finished calling action server: {self.finished_call_time - self.start_time} secs')
            if self.feedback_time is not None:
                logger.info(f'           to received feedback from action server: {self.feedback_time - self.start_time} secs')
            logger.info(f'           to received done call from action server: {self.done_time - self.start_time} secs')
            logger.info(f'           to image ready: {self.ready_time - self.start_time} secs')
            logger.info(f'           to image saved to file system: {self.saved_time - self.start_time} secs')
            
    def cb_camera_done(self, status, result):
        if status == GoalStatus.PREEMPTED:
            # nothing needs to be done because the goal return is due to an abort call to the action server
            logger.warning(f'Test.cb_camera_done: captured image discarded as the action ABORTED')
            return 
        elif status == GoalStatus.SUCCEEDED:
            self.done_time = time.time()
            metadata = json.loads(result.metadata)
            metadata = self.extract_properties_in_metadata(metadata)
            filename = f'{datetime.now().strftime("%m%d%Y%H%M")}'
            filepath = os.path.join(self.output_image_folder, f'{filename}.jpg')
            if self.USE_COMPRESSED:
                np_arr = np.frombuffer(result.cimage.data, np.uint8)
                image_cv = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                self.ready_time = time.time()
                cv2.imwrite(filepath, image_cv)
                self.saved_time = time.time()
            else:
                image_cv = self.cv_bridge.imgmsg_to_cv2(result.image, desired_encoding='bgr8') 

                self.ready_time = time.time()
                cv2.imwrite(filepath, image_cv)
                self.saved_time = time.time()
            self.has_done = True

    def cb_camera_feedback(self, feedback):
        self.feedback_time = time.time()

    def call_camera_to_capture(self, tank_id, placement_x, placement_y, capture_x, capture_y):
        goal = TestSnapGoal('capture', tank_id, placement_x, placement_y, capture_x, capture_y)
        self.snap_action_client.send_goal(goal, done_cb=self.cb_camera_done, feedback_cb=self.cb_camera_feedback)

    def extract_properties_in_metadata(self, metadata:dict) -> dict:
        if metadata is None:
            logger.warning(f'cb_camera_done: metadata is None')
            return None
        properties = metadata.get('properties', None)
        if properties is None:
            return metadata
        for key in properties.keys():
            setting = properties[key]
            value = setting['value_readable']
            metadata[key] = value
        del metadata['properties']
        return metadata

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
        the_task = None
        result = TestSnapResult()
        with self.task_lock:
            # differentiate the goal types and create the corresponding tasks with parameters obtained from the goal
            if goal.command == 'capture':
                tank_id = goal.tank_id
                tile_x, tile_y = goal.tile_x, goal.tile_y
                capture_x, capture_y = goal.grid_x, goal.grid_y
                logger.warning(f'CGRASTestImageServer: received capture goal {tank_id} tile ({tile_x}, {tile_y}) catpure ({capture_x},{capture_y})')
                image_size = self.sample_image.shape[:2][::-1]
                image_to_send_size = [int(image_size[0] / self.capture_grid_dim[0]), int(image_size[1] / self.capture_grid_dim[1])]
                image_to_send = self.sample_image[capture_y * image_to_send_size[1]:capture_y * image_to_send_size[1] + image_to_send_size[1],
                                           capture_x * image_to_send_size[0]:capture_x * image_to_send_size[0] + image_to_send_size[0]]
                # simulate attempt to focus and snap
                time.sleep(2.0)
                # publish feedback that the snap is done
                feedback_msg = TestSnapFeedback()
                self.action_server_snap.publish_feedback(feedback_msg) 
                # simulate time taken for the image to transfer as this is a small image
                time.sleep(1.0)

                result.image = self.cv_bridge.cv2_to_imgmsg(image_to_send, encoding='bgr8')
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
     

# ----- the main program
if __name__ == '__main__':
    rospy.init_node('cgras_test_image_client_node', anonymous=False)
    try:
        rospy.loginfo('cgras_test_image_client_node is running')
        robot_agent = CGRASTestImageClient()
        # rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)