#!/usr/bin/env python

import yaml
import rospy
import datetime
import argparse
import getpass
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from region_observation.util import get_soma_info
from visualization_msgs.msg import Marker, MarkerArray
from mongodb_store.message_store import MessageStoreProxy
from human_trajectory.visualisation import TrajectoryVisualisation
from region_observation.util import create_line_string, is_intersected
from ubd_scene_image_comparator.msg import UbdSceneImgLog, UbdSceneAccuracy


class DetectionImageAnnotator(object):

    def __init__(self, config, path="/home/%s/Pictures" % getpass.getuser()):
        self.path = path
        self.regions, self.map = get_soma_info(config)
        self._stop = False
        self._img = Image()
        self.activity = {
            roi: {"Present": 0, "Absent": 0} for roi in self.regions.keys()
        }
        self.ubd = {
            roi: {"TP": 0, "FN": 0, "FP": 0, "TN": 0} for roi in self.regions.keys()
        }
        self.cd = {
            roi: {"TP": 0, "FN": 0, "FP": 0, "TN": 0} for roi in self.regions.keys()
        }
        self.leg = {
            roi: {"TP": 0, "FN": 0, "FP": 0, "TN": 0} for roi in self.regions.keys()
        }
        self.roi_activity = {roi: list() for roi in self.regions.keys()}
        self.roi_non_activity = {roi: list() for roi in self.regions.keys()}
        self._db = MessageStoreProxy(collection="ubd_scene_log")
        self._db_store = MessageStoreProxy(collection="ubd_scene_accuracy")
        self._trajvis = TrajectoryVisualisation(rospy.get_name()+"/leg")
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
        logs = self._db_store.query(
            UbdSceneAccuracy._type,
            message_query={"map": self.map},
            sort_query=[("header.stamp.secs", -1)],
            limit=len(self.regions.keys())
        )
        used_rois = list()
        for log in logs:
            log = log[0]
            if log.region_id in used_rois:
                continue
            self.activity[log.region_id]["Present"] = log.activity_present
            self.activity[log.region_id]["Absent"] = log.activity_absent
            self.ubd[log.region_id]["TP"] = log.ubd_tp
            self.ubd[log.region_id]["FN"] = log.ubd_fn
            self.ubd[log.region_id]["FP"] = log.ubd_fp
            self.ubd[log.region_id]["TN"] = log.ubd_tn
            self.cd[log.region_id]["TP"] = log.cd_tp
            self.cd[log.region_id]["FN"] = log.cd_fn
            self.cd[log.region_id]["FP"] = log.cd_fp
            self.cd[log.region_id]["TN"] = log.cd_tn
            self.leg[log.region_id]["TP"] = log.leg_tp
            self.leg[log.region_id]["FN"] = log.leg_fn
            self.leg[log.region_id]["FP"] = log.leg_fp
            self.leg[log.region_id]["TN"] = log.leg_tn
            used_rois.append(log.region_id)
        rospy.loginfo("Loading...")
        rospy.loginfo("Activity: %s" % str(self.activity))
        rospy.loginfo("UBD: %s" % str(self.ubd))
        rospy.loginfo("Scene: %s" % str(self.cd))
        rospy.loginfo("Leg: %s" % str(self.leg))
        try:
            self.roi_activity = yaml.load(open("%s/roi_activity.yaml" % self.path, "r"))
            self.roi_non_activity = yaml.load(open("%s/roi_non_activity.yaml" % self.path, "r"))
        except IOError:
            self.roi_activity = {roi: list() for roi in self.regions.keys()}
            self.roi_non_activity = {roi: list() for roi in self.regions.keys()}

    def save_accuracy(self):
        header = Header(1, rospy.Time.now(), "")
        rospy.loginfo("Storing...")
        rospy.loginfo("Activity: %s" % str(self.activity))
        rospy.loginfo("UBD: %s" % str(self.ubd))
        rospy.loginfo("Scene: %s" % str(self.cd))
        rospy.loginfo("Leg: %s" % str(self.leg))
        for roi in self.regions.keys():
            log = UbdSceneAccuracy(
                header, self.activity[roi]["Present"], self.activity[roi]["Absent"],
                self.ubd[roi]["TP"], self.ubd[roi]["FN"],
                self.ubd[roi]["FP"], self.ubd[roi]["TN"],
                self.cd[roi]["TP"], self.cd[roi]["FN"],
                self.cd[roi]["FP"], self.cd[roi]["TN"],
                self.leg[roi]["TP"], self.leg[roi]["FN"],
                self.leg[roi]["FP"], self.leg[roi]["TN"],
                roi, self.map
            )
            self._db_store.insert(log)
        with open("%s/roi_activity.yaml" % self.path, 'w') as f:
            f.write(yaml.dump(self.roi_activity))
        with open("%s/roi_non_activity.yaml" % self.path, 'w') as f:
            f.write(yaml.dump(self.roi_non_activity))

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

    def _ubd_annotation(self, log, activity_rois=list(), nonactivity_rois=list()):
        ubd_rois = list()
        for centroid in log[0].ubd_pos:
            point = create_line_string([centroid.x, centroid.y])
            for roi, region in self.regions.iteritems():
                if is_intersected(region, point):
                    ubd_rois.append(roi)
        for roi in activity_rois:
            if roi in ubd_rois:
                self.ubd[roi]["TP"] += 1
            else:
                self.ubd[roi]["FN"] += 1
        for roi in nonactivity_rois:
            if roi in ubd_rois:
                self.ubd[roi]["FP"] += 1
            else:
                self.ubd[roi]["TN"] += 1

    def _cd_annotation(self, log, activity_rois=list(), nonactivity_rois=list()):
        cd_rois = list()
        for centroid in log[0].cd_pos:
            point = create_line_string([centroid.x, centroid.y])
            for roi, region in self.regions.iteritems():
                if is_intersected(region, point):
                    cd_rois.append(roi)
        for roi in activity_rois:
            if roi in cd_rois:
                self.cd[roi]["TP"] += 1
            else:
                self.cd[roi]["FN"] += 1
        for roi in nonactivity_rois:
            if roi in cd_rois:
                self.cd[roi]["FP"] += 1
            else:
                self.cd[roi]["TN"] += 1

    def _leg_vis(self, log):
        start_time = log[0].header.stamp - rospy.Duration(60)
        end_time = log[0].header.stamp
        query = {"$or": [
            {"end_time.secs": {
                "$gte": start_time.secs, "$lt": end_time.secs
            }},
            {"start_time.secs": {
                "$gte": start_time.secs, "$lt": end_time.secs

            }},
            {
                "start_time.secs": {"$lt": start_time.secs},
                "end_time.secs": {"$gte": end_time.secs}
            }
        ]}
        self._trajvis.update_query(query=query)
        for traj in self._trajvis.trajs.traj.itervalues():
            self._trajvis.visualize_trajectory(traj)

    def _leg_annotation(self, activity_rois=list(), nonactivity_rois=list()):
        leg_rois = list()
        for traj in self._trajvis.trajs.traj.itervalues():
            trajectory = traj.get_trajectory_message()
            points = [
                [
                    pose.pose.position.x, pose.pose.position.y
                ] for pose in trajectory.trajectory
            ]
            points = create_line_string(points)
            for roi, region in self.regions.iteritems():
                if is_intersected(region, points):
                    leg_rois.append(roi)
        for roi in activity_rois:
            if roi in leg_rois:
                self.leg[roi]["TP"] += 1
            else:
                self.leg[roi]["FN"] += 1
        for roi in nonactivity_rois:
            if roi in leg_rois:
                self.leg[roi]["FP"] += 1
            else:
                self.leg[roi]["TN"] += 1

    def _activity_annotation(self, log):
        act_rois = list()
        rospy.logwarn(
            "Please wait until the new image appears before answering..."
        )
        rospy.logwarn(
            "All questions are based on the image topic %s" % rospy.get_name()
        )
        timestamp = log[0].header.stamp
        datestamp = datetime.datetime.fromtimestamp(timestamp.secs)
        # listing all regions which activity happened from the image
        text = "Which regions did the activity happen within a minute around %s? \n" % str(datestamp)
        text += "Please select from %s, " % str(self.regions.keys())
        text += "and write in the following format '[reg_1,...,reg_n]'\n"
        text += "(Press ENTER if no activity is observed): "
        inpt = raw_input(text)
        inpt = inpt.replace(" ", "")
        inpt = inpt.replace("[", "")
        inpt = inpt.replace("]", "")
        act_rois = inpt.split(",")
        if '' in act_rois and len(act_rois) == 1:
            act_rois = list()
        for roi in act_rois:
            self.activity[roi]["Present"] += 1
            self.roi_activity[roi].append((timestamp.secs / 60) * 60)
        # listing all regions which no activity happening from the image
        text = "Which regions with no activity within a minute around %s? \n" % str(datestamp)
        text += "Please select from %s, " % str(self.regions.keys())
        text += "and write in the following format '[reg_1,...,reg_n]'\n"
        text += "(Press ENTER if all regions had an activity): "
        inpt = raw_input(text)
        inpt = inpt.replace(" ", "")
        inpt = inpt.replace("[", "")
        inpt = inpt.replace("]", "")
        nact_rois = inpt.split(",")
        if '' in nact_rois and len(nact_rois) == 1:
            nact_rois = list()
        for roi in nact_rois:
            self.activity[roi]["Absent"] += 1
            self.roi_non_activity[roi].append((timestamp.secs / 60) * 60)
        return act_rois, nact_rois

    def annotate(self):
        while not rospy.is_shutdown() and not self._stop:
            logs = self._db.query(
                UbdSceneImgLog._type, {"annotated": False}, limit=10
                # UbdSceneImgLog._type, {"annotated": False}, limit=3  # TESTING PURPOSE
            )
            rospy.loginfo(
                "Getting %d logs from ubd_scene_log collection" % len(logs)
            )
            # projection_query={"robot_data": 0, "skeleton_data": 0}
            for log in logs:
                self._img = log[0].image
                self._ubd = log[0].ubd_pos
                self._cd = log[0].cd_pos
                self._leg_vis(log)
                # annotation part
                act_rois, nact_rois = self._activity_annotation(log)
                self._ubd_annotation(log, act_rois, nact_rois)
                self._cd_annotation(log, act_rois, nact_rois)
                self._leg_annotation(act_rois, nact_rois)
                # resetting
                self._cd = list()
                self._ubd = list()
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
            for log in logs:
                log[0].annotated = True
                self._db.update(
                    log[0], message_query={
                        "header.stamp.secs": log[0].header.stamp.secs
                    }
                )
        rospy.loginfo("Thanks!")


if __name__ == '__main__':
    rospy.init_node("ubd_scene_annotator")
    parser = argparse.ArgumentParser(prog=rospy.get_name())
    parser.add_argument("soma_config", help="Soma configuration")
    args = parser.parse_args()
    dia = DetectionImageAnnotator(args.soma_config)
    dia.annotate()
