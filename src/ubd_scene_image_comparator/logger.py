#!/usr/bin/env python

import rospy
import datetime
import message_filters
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from scipy.spatial.distance import euclidean
from sensor_msgs.msg import Image, JointState
from vision_people_logging.msg import LoggingUBD
from simple_change_detector.msg import ChangeDetectionMsg
from ubd_scene_image_comparator.msg import UbdSceneImgLog
from mongodb_store.message_store import MessageStoreProxy


class DetectionImageLogger(object):

    def __init__(self, wait_time=30):
        self._counter = 0
        self.img = Image()
        self.ubd = list()
        self.change = list()
        self._db = MessageStoreProxy(collection="ubd_scene_log")
        # ptu
        self._max_dist = 0.1
        self._wait_time = wait_time
        rospy.loginfo("Subcribe to /ptu/state...")
        self._ptu = JointState()
        self._ptu.position = [0, 0]
        self._ptu_counter = 0
        self._is_ptu_changing = [True for i in range(wait_time)]
        rospy.Subscriber("/ptu/state", JointState, self._ptu_cb, None, 1)
        # robot pose
        rospy.loginfo("Subcribe to /robot_pose...")
        self._robot_pose = Pose()
        self._robot_pose_counter = 0
        self._is_robot_moving = [True for i in range(wait_time)]
        rospy.Subscriber("/robot_pose", Pose, self._robot_cb, None, 1)
        # logging stuff
        subs = [
            message_filters.Subscriber(
                rospy.get_param(
                    "~scene_topic", "/change_detection/detections"
                ), ChangeDetectionMsg
            ),
            message_filters.Subscriber(
                rospy.get_param(
                    "~ubd_topic", "/vision_logging_service/log"
                ), LoggingUBD
            ),
            message_filters.Subscriber(
                rospy.get_param(
                    "~image_topic", "/head_xtion/rgb/image_rect_color"
                ), Image
            )
        ]
        ts = message_filters.ApproximateTimeSynchronizer(
            subs, queue_size=1, slop=0.15
        )
        ts.registerCallback(self.cb)
        rospy.Timer(rospy.Duration(60), self.to_log)

    def _ptu_cb(self, ptu):
        dist = euclidean(ptu.position, self._ptu.position)
        self._is_ptu_changing[self._ptu_counter] = dist >= self._max_dist
        self._ptu_counter = (self._ptu_counter+1) % self._wait_time
        self._ptu = ptu
        rospy.sleep(1)

    def _robot_cb(self, pose):
        dist = euclidean(
            [
                pose.position.x, pose.position.y,
                pose.orientation.z, pose.orientation.w
            ],
            [
                self._robot_pose.position.x, self._robot_pose.position.y,
                self._robot_pose.orientation.z, self._robot_pose.orientation.w
            ]
        )
        self._is_robot_moving[self._robot_pose_counter] = dist >= self._max_dist
        self._robot_pose_counter = (
            self._robot_pose_counter+1
        ) % self._wait_time
        self._robot_pose = pose
        rospy.sleep(1)

    def cb(self, cd, ubd, img):
        self.ubd.append(ubd.ubd_pos)
        self.change.append(cd.object_centroids)
        self.img = img

    def to_log(self, timer_event):
        not_moving = True not in (self._is_robot_moving+self._is_ptu_changing)
        if not_moving:
            now = rospy.Time.now()
            date_now = datetime.datetime.fromtimestamp(now.secs)
            rospy.loginfo(
                "Got to log at %s with %d ubd detections and %d scene detections" % (
                    str(date_now), len(self.ubd), len(self.change)
                )
            )
            header = Header(self._counter, now, "")
            self._counter += 1
            log = UbdSceneImgLog(header, self.img, self.ubd, self.change, False)
            self._db.insert(log)
            self.img = Image()
            self.ubd = list()
            self.change = list()
        else:
            rospy.loginfo("Robot is moving, ignore logging data...")


if __name__ == '__main__':
    rospy.init_node("ubd_scene_img_logger")
    DetectionImageLogger()
    rospy.spin()
