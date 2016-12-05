#!/usr/bin/env python

import rospy
import datetime
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from mongodb_store.message_store import MessageStoreProxy
from ubd_scene_image_comparator.msg import UbdSceneImgLog, UbdSceneAccuracy


class DetectionImageAnnotator(object):

    def __init__(self):
        self._stop = False
        self._img = Image()
        self.activity = {"Present": 0, "Absent": 0}
        self.ubd = {"TP": 0, "FN": 0, "FP": 0, "TN": 0}
        self.change = {"TP": 0, "FN": 0, "FP": 0, "TN": 0}
        self._db = MessageStoreProxy(collection="ubd_scene_log")
        self._db_store = MessageStoreProxy(collection="ubd_scene_accuracy")
        # visualisation stuff
        self._cd = list()
        self._cd_len = 0
        self._ubd = list()
        self._ubd_len = 0
        self._pub = rospy.Publisher(rospy.get_name(), Image, queue_size=10)
        self._pub_cd_marker = rospy.Publisher(
            rospy.get_name()+"/cd_marker", MarkerArray, queue_size=10
        )
        self._pub_ubd_marker = rospy.Publisher(
            rospy.get_name()+"/ubd_marker", MarkerArray, queue_size=10
        )
        self.load_accuracy()
        rospy.Timer(rospy.Duration(0.1), self.pub_img)

    def load_accuracy(self):
        log = self._db_store.query(
            UbdSceneAccuracy._type, sort_query=[("header.stamp.secs", -1)],
            limit=1
        )
        if len(log):
            log = log[0][0]
            self.activity["Present"] = log.activity_present
            self.activity["Absent"] = log.activity_absent
            self.ubd["TP"] = log.ubd_tp
            self.ubd["FN"] = log.ubd_fn
            self.ubd["FP"] = log.ubd_fp
            self.ubd["TN"] = log.ubd_tn
            self.change["TP"] = log.cd_tp
            self.change["FN"] = log.cd_fn
            self.change["FP"] = log.cd_fp
            self.change["TN"] = log.cd_tn
        rospy.loginfo("Loading...")
        rospy.loginfo("Activity: %s" % str(self.activity))
        rospy.loginfo("UBD: %s" % str(self.ubd))
        rospy.loginfo("Scene: %s" % str(self.change))

    def save_accuracy(self):
        header = Header(1, rospy.Time.now(), "")
        rospy.loginfo("Storing...")
        rospy.loginfo("Activity: %s" % str(self.activity))
        rospy.loginfo("UBD: %s" % str(self.ubd))
        rospy.loginfo("Scene: %s" % str(self.change))
        log = UbdSceneAccuracy(
            header, self.activity["Present"], self.activity["Absent"],
            self.ubd["TP"], self.ubd["FN"], self.ubd["FP"], self.ubd["TN"],
            self.change["TP"], self.change["FN"],
            self.change["FP"], self.change["TN"]
        )
        self._db_store.insert(log)

    def pub_img(self, timer_event):
        self._pub.publish(self._img)
        cd_markers = self._draw_detections(self._cd, "cd", self._cd_len)
        self._pub_cd_marker.publish(cd_markers)
        self._cd_len = len(self._cd)
        ubd_markers = self._draw_detections(self._ubd, "ubd", self._ubd_len)
        self._pub_ubd_marker.publish(ubd_markers)
        self._ubd_len = len(self._ubd)

    def _draw_detections(self, centroids, type="cd", length=0):
        markers = MarkerArray()
        for ind, centroid in enumerate(centroids):
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = rospy.get_name() + "_marker"
            marker.action = Marker.ADD
            marker.pose.position = centroid
            marker.pose.orientation.w = 1.0
            marker.id = ind
            if type == "cd":
                marker.type = Marker.SPHERE
                marker.color.b = 1.0
            else:
                marker.type = Marker.CUBE
                marker.color.g = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            markers.markers.append(marker)
        if len(centroids) == 0:
            for ind in range(length):
                marker = Marker()
                marker.header.frame_id = "/map"
                marker.header.stamp = rospy.Time.now()
                marker.ns = rospy.get_name() + "_marker"
                marker.action = Marker.DELETE
                marker.id = ind
                if type == "cd":
                    marker.type = Marker.SPHERE
                else:
                    marker.type = Marker.CUBE
                marker.color.a = 0.0
                markers.markers.append(marker)
        return markers

    def annotate(self):
        while not rospy.is_shutdown() and not self._stop:
            logs = self._db.query(
                UbdSceneImgLog._type, {"annotated": False}, limit=10
            )
            rospy.loginfo(
                "Getting %d logs from ubd_scene_log collection" % len(logs)
            )
            # projection_query={"robot_data": 0, "skeleton_data": 0}
            for log in logs:
                self._img = log[0].image
                self._ubd = log[0].ubd_pos
                self._cd = log[0].cd_pos
                rospy.loginfo(
                    "Please wait until the new image appears before answering"
                )
                timestamp = log[0].header.stamp
                datestamp = datetime.datetime.fromtimestamp(timestamp.secs)
                text = "Could you see if there is an activity going on "
                text += "within a minute around %s " % str(datestamp)
                text += "in image topic %s? [1/0/-1]" % rospy.get_name()
                inpt = raw_input(text)
                while not (inpt == "1" or inpt == "0" or inpt == "-1"):
                    inpt = raw_input(
                        "Please, answer 1 for an activity, 0 for not activity, and -1 for invalid data"
                    )
                if int(inpt) == 1:
                    self.activity["Present"] += 1
                    text = "Could you see if the upper body detections"
                    text += "are in the activity area in marker topic "
                    text += "%s? [1/0]" % (rospy.get_name() + "/ubd_marker")
                    inpt = raw_input(text)
                    while not (inpt == "1" or inpt == "0"):
                        inpt = raw_input(
                            "Please, answer 1 for yes, 0 for no"
                        )
                    if int(inpt):
                        self.ubd["TP"] += 1
                    else:
                        self.ubd["FN"] += 1
                    text = "Could you see if the scene detections"
                    text += "are in the activity area in marker topic "
                    text += "%s? [1/0]" % (rospy.get_name() + "/cd_marker")
                    inpt = raw_input(text)
                    while not (inpt == "1" or inpt == "0"):
                        inpt = raw_input(
                            "Please, answer 1 for yes, 0 for no"
                        )
                    if int(inpt):
                        self.change["TP"] += 1
                    else:
                        self.change["FN"] += 1
                elif int(inpt) == 0:
                    self.activity["Absent"] += 1
                    if len(log[0].ubd_pos):
                        self.ubd["FP"] += 1
                    else:
                        self.ubd["TN"] += 1
                    if len(log[0].cd_pos):
                        self.change["FP"] += 1
                    else:
                        self.change["TN"] += 1
                self._cd = list()
                self._ubd = list()
                log[0].annotated = True
                self._db.update(
                    log[0], message_query={"header.stamp.secs": timestamp.secs}
                )
                rospy.sleep(1)
            inpt = raw_input("Stop now? [1/0]")
            try:
                inpt = int(inpt)
            except:
                self.save_accuracy()
                self._stop = True
            if inpt:
                self.save_accuracy()
                self._stop = True
        rospy.loginfo("Thanks!")


if __name__ == '__main__':
    rospy.init_node("ubd_scene_annotator")
    dia = DetectionImageAnnotator()
    dia.annotate()
    rospy.spin()
