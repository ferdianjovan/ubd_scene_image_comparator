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

import pymongo
from geometry_msgs.msg import Point
from shapely.geometry import Polygon
from strands_navigation_msgs.msg import TopologicalMap
from simple_change_detector.msg import ChangeDetectionMsg


class DetectionImageAnnotator(object):

    def __init__(self, config, path="/home/%s/Pictures" % getpass.getuser()):
        self.path = path
        self.regions, self.map = get_soma_info(config)
        self.topo_map = None
        self._get_waypoints()
        self.topo_map = {
            wp.name: Polygon([[wp.pose.position.x+i.x, wp.pose.position.y+i.y] for i in wp.verts]) for wp in self.topo_map.nodes
        }
        self._stop = False
        self._img = Image()
        self.activity = {
            roi: {
                wp: {"Present": 0, "Absent": 0} for wp in self.topo_map
            } for roi in self.regions.keys()
        }
        self.ubd = {
            roi: {
                wp: {"TP": 0, "FN": 0, "FP": 0, "TN": 0} for wp in self.topo_map
            } for roi in self.regions.keys()
        }
        self.cd = {
            roi: {
                wp: {"TP": 0, "FN": 0, "FP": 0, "TN": 0} for wp in self.topo_map
            } for roi in self.regions.keys()
        }
        self.leg = {
            roi: {
                wp: {"TP": 0, "FN": 0, "FP": 0, "TN": 0} for wp in self.topo_map
            } for roi in self.regions.keys()
        }
        self.wp_history = dict()
        self.roi_activity = {roi: list() for roi in self.regions.keys()}
        self.roi_non_activity = {roi: list() for roi in self.regions.keys()}
        self.roi_cd = {roi: list() for roi in self.regions.keys()}
        # self.roi_non_cd = {roi: list() for roi in self.regions.keys()}
        self.roi_ubd = {roi: list() for roi in self.regions.keys()}
        # self.roi_non_ubd = {roi: list() for roi in self.regions.keys()}
        self.roi_leg = {roi: list() for roi in self.regions.keys()}
        # self.roi_non_leg = {roi: list() for roi in self.regions.keys()}
        self._db = MessageStoreProxy(collection="ubd_scene_log")
        self._db_store = MessageStoreProxy(collection="ubd_scene_accuracy")
        self._db_change = MessageStoreProxy(collection="change_detection")
        self._db_ubd = pymongo.MongoClient(
            rospy.get_param("mongodb_host", "localhost"),
            rospy.get_param("mongodb_port", 62345)
        ).message_store.upper_bodies
        self._trajvis = TrajectoryVisualisation(rospy.get_name()+"/leg")
        # visualisation stuff
        self._cd = list()
        self._cd_len = 0
        self._ubd = list()
        self._ubd_len = 0
        self._robot_pos = list()
        self._robot_pos_len = 0
        self._pub = rospy.Publisher(rospy.get_name(), Image, queue_size=10)
        self._pub_cd_marker = rospy.Publisher(
            rospy.get_name()+"/cd_marker", MarkerArray, queue_size=10
        )
        self._pub_ubd_marker = rospy.Publisher(
            rospy.get_name()+"/ubd_marker", MarkerArray, queue_size=10
        )
        self._pub_robot_marker = rospy.Publisher(
            rospy.get_name()+"/robot_marker", MarkerArray, queue_size=10
        )
        self.load_accuracy()
        rospy.Timer(rospy.Duration(0.1), self.pub_img)

    def _topo_map_cb(self, topo_map):
        self.topo_map = topo_map

    def _get_waypoints(self):
        topo_sub = rospy.Subscriber(
            "/topological_map", TopologicalMap, self._topo_map_cb, None, 10
        )
        rospy.loginfo("Getting information from /topological_map...")
        while self.topo_map is None:
            rospy.sleep(0.1)
        topo_sub.unregister()

    def load_accuracy(self):
        logs = self._db_store.query(
            UbdSceneAccuracy._type,
            message_query={"map": self.map},
            sort_query=[("header.stamp.secs", -1)],
            limit=len(self.regions.keys())*len(self.topo_map)
        )
        used_rois = list()
        for log in logs:
            log = log[0]
            if (log.region_id, log.region_config) in used_rois:
                continue
            self.activity[log.region_id][log.region_config]["Present"] = log.activity_present
            self.activity[log.region_id][log.region_config]["Absent"] = log.activity_absent
            self.ubd[log.region_id][log.region_config]["TP"] = log.ubd_tp
            self.ubd[log.region_id][log.region_config]["FN"] = log.ubd_fn
            self.ubd[log.region_id][log.region_config]["FP"] = log.ubd_fp
            self.ubd[log.region_id][log.region_config]["TN"] = log.ubd_tn
            self.cd[log.region_id][log.region_config]["TP"] = log.cd_tp
            self.cd[log.region_id][log.region_config]["FN"] = log.cd_fn
            self.cd[log.region_id][log.region_config]["FP"] = log.cd_fp
            self.cd[log.region_id][log.region_config]["TN"] = log.cd_tn
            self.leg[log.region_id][log.region_config]["TP"] = log.leg_tp
            self.leg[log.region_id][log.region_config]["FN"] = log.leg_fn
            self.leg[log.region_id][log.region_config]["FP"] = log.leg_fp
            self.leg[log.region_id][log.region_config]["TN"] = log.leg_tn
            used_rois.append((log.region_id, log.region_config))
        # rospy.loginfo("Loading...")
        # rospy.loginfo("Activity: %s" % str(self.activity))
        # rospy.loginfo("UBD: %s" % str(self.ubd))
        # rospy.loginfo("Scene: %s" % str(self.cd))
        # rospy.loginfo("Leg: %s" % str(self.leg))
        try:
            self.roi_activity = yaml.load(open("%s/roi_activity.yaml" % self.path, "r"))
            self.roi_non_activity = yaml.load(open("%s/roi_non_activity.yaml" % self.path, "r"))
            self.roi_cd = yaml.load(open("%s/roi_cd.yaml" % self.path, "r"))
            # self.roi_non_cd = yaml.load(open("%s/roi_non_cd.yaml" % self.path, "r"))
            self.roi_ubd = yaml.load(open("%s/roi_ubd.yaml" % self.path, "r"))
            # self.roi_non_ubd = yaml.load(open("%s/roi_non_ubd.yaml" % self.path, "r"))
            self.roi_leg = yaml.load(open("%s/roi_leg.yaml" % self.path, "r"))
            # self.roi_non_leg = yaml.load(open("%s/roi_non_leg.yaml" % self.path, "r"))
            self.wp_history = yaml.load(open("%s/wp_history.yaml" % self.path, "r"))
        except IOError:
            self.roi_activity = {roi: list() for roi in self.regions.keys()}
            self.roi_non_activity = {roi: list() for roi in self.regions.keys()}
            self.roi_cd = {roi: list() for roi in self.regions.keys()}
            # self.roi_non_cd = {roi: list() for roi in self.regions.keys()}
            self.roi_ubd = {roi: list() for roi in self.regions.keys()}
            # self.roi_non_ubd = {roi: list() for roi in self.regions.keys()}
            self.roi_leg = {roi: list() for roi in self.regions.keys()}
            # self.roi_non_leg = {roi: list() for roi in self.regions.keys()}
            self.wp_history = dict()

    def save_accuracy(self):
        header = Header(1, rospy.Time.now(), "")
        rospy.loginfo("Storing...")
        # rospy.loginfo("Activity: %s" % str(self.activity))
        # rospy.loginfo("UBD: %s" % str(self.ubd))
        # rospy.loginfo("Scene: %s" % str(self.cd))
        # rospy.loginfo("Leg: %s" % str(self.leg))
        for roi in self.regions.keys():
            for wp in self.topo_map:
                log = UbdSceneAccuracy(
                    header, self.activity[roi][wp]["Present"], self.activity[roi][wp]["Absent"],
                    self.ubd[roi][wp]["TP"], self.ubd[roi][wp]["FN"],
                    self.ubd[roi][wp]["FP"], self.ubd[roi][wp]["TN"],
                    self.cd[roi][wp]["TP"], self.cd[roi][wp]["FN"],
                    self.cd[roi][wp]["FP"], self.cd[roi][wp]["TN"],
                    self.leg[roi][wp]["TP"], self.leg[roi][wp]["FN"],
                    self.leg[roi][wp]["FP"], self.leg[roi][wp]["TN"],
                    roi, wp, self.map
                )
                self._db_store.insert(log)
        with open("%s/roi_activity.yaml" % self.path, 'w') as f:
            f.write(yaml.dump(self.roi_activity))
        with open("%s/roi_non_activity.yaml" % self.path, 'w') as f:
            f.write(yaml.dump(self.roi_non_activity))
        with open("%s/roi_cd.yaml" % self.path, 'w') as f:
            f.write(yaml.dump(self.roi_cd))
        with open("%s/roi_ubd.yaml" % self.path, 'w') as f:
            f.write(yaml.dump(self.roi_ubd))
        with open("%s/roi_leg.yaml" % self.path, 'w') as f:
            f.write(yaml.dump(self.roi_leg))
        with open("%s/wp_history.yaml" % self.path, 'w') as f:
            f.write(yaml.dump(self.wp_history))

    def pub_img(self, timer_event):
        self._pub.publish(self._img)
        cd_markers = self._draw_detections(self._cd, "cd", self._cd_len)
        self._pub_cd_marker.publish(cd_markers)
        self._cd_len = len(self._cd)
        ubd_markers = self._draw_detections(self._ubd, "ubd", self._ubd_len)
        self._pub_ubd_marker.publish(ubd_markers)
        self._ubd_len = len(self._ubd)
        robot_markers = self._draw_detections(self._robot_pos, "robot", self._robot_pos_len)
        self._pub_robot_marker.publish(robot_markers)
        self._robot_pos_len = len(self._robot_pos)

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
            elif type == "ubd":
                marker.type = Marker.CUBE
                marker.color.g = 1.0
            else:
                marker.type = Marker.CYLINDER
                marker.color.r = 1.0
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

    def _ubd_annotation(self, log, activity_rois=list(), nonactivity_rois=list(), wp=""):
        ubd_rois = list()
        timestamp = log[0].header.stamp
        for centroid in self._ubd:
            point = create_line_string([centroid.x, centroid.y])
            for roi, region in self.regions.iteritems():
                if is_intersected(region, point):
                    ubd_rois.append(roi)
                    self.roi_ubd[roi].append((timestamp.secs / 60) * 60)
        for roi in activity_rois:
            if roi in ubd_rois:
                self.ubd[roi][wp]["TP"] += 1
            else:
                self.ubd[roi][wp]["FN"] += 1
        for roi in nonactivity_rois:
            if roi in ubd_rois:
                self.ubd[roi][wp]["FP"] += 1
            else:
                self.ubd[roi][wp]["TN"] += 1

    def _cd_annotation(self, log, activity_rois=list(), nonactivity_rois=list(), wp=""):
        cd_rois = list()
        timestamp = log[0].header.stamp
        for centroid in self._cd:
            point = create_line_string([centroid.x, centroid.y])
            for roi, region in self.regions.iteritems():
                if is_intersected(region, point):
                    cd_rois.append(roi)
                    self.roi_cd[roi].append((timestamp.secs / 60) * 60)
        for roi in activity_rois:
            if roi in cd_rois:
                self.cd[roi][wp]["TP"] += 1
            else:
                self.cd[roi][wp]["FN"] += 1
        for roi in nonactivity_rois:
            if roi in cd_rois:
                self.cd[roi][wp]["FP"] += 1
            else:
                self.cd[roi][wp]["TN"] += 1

    def _leg_vis(self, log):
        start_time = (log[0].header.stamp.secs / 60) * 60
        end_time = start_time + 60
        query = {"$or": [
            {"end_time.secs": {
                "$gte": start_time, "$lt": end_time
            }},
            {"start_time.secs": {
                "$gte": start_time, "$lt": end_time

            }},
            {
                "start_time.secs": {"$lt": start_time},
                "end_time.secs": {"$gte": end_time}
            }
        ]}
        self._trajvis.update_query(query=query)
        for traj in self._trajvis.trajs.traj.itervalues():
            self._trajvis.visualize_trajectory(traj)

    def _leg_annotation(self, log, activity_rois=list(), nonactivity_rois=list(), wp=""):
        leg_rois = list()
        timestamp = log[0].header.stamp
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
                    self.roi_leg[roi].append((timestamp.secs / 60) * 60)
        for roi in activity_rois:
            if roi in leg_rois:
                self.leg[roi][wp]["TP"] += 1
            else:
                self.leg[roi][wp]["FN"] += 1
        for roi in nonactivity_rois:
            if roi in leg_rois:
                self.leg[roi][wp]["FP"] += 1
            else:
                self.leg[roi][wp]["TN"] += 1

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
        wp = list()
        for pose in self._robot_pos:
            point = create_line_string([pose.x, pose.y])
            for wp_name, wp_region in self.topo_map.iteritems():
                if is_intersected(wp_region, point):
                    wp.append(wp_name)
        if wp == list():
            inpt = ""
            while inpt not in self.topo_map.keys():
                text = "Manual, From which region did the robot observed around %s? \n" % str(datestamp)
                text += "Please select from %s:" % str(self.topo_map.keys())
                inpt = raw_input(text)
            wp = inpt
        elif len(list(set(wp))) > 1:
            inpt = ""
            while inpt not in self.topo_map.keys():
                text = "From which region did the robot observed around %s? \n" % str(datestamp)
                text += "Please select from %s:" % str(list(set(wp)))
                inpt = raw_input(text)
            wp = inpt
        else:
            wp = wp[0]
        self.wp_history.update({(timestamp.secs/60)*60: wp})
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
            self.activity[roi][wp]["Present"] += 1
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
            self.activity[roi][wp]["Absent"] += 1
            self.roi_non_activity[roi].append((timestamp.secs / 60) * 60)
        return act_rois, nact_rois, wp

    def get_cd_pos(self, stamp):
        robot = list()
        detections = list()
        start_time = (stamp.secs / 60) * 60
        end_time = start_time + 60
        logs = self._db_change.query(
            ChangeDetectionMsg._type,
            {"header.stamp.secs": {"$gte": start_time, "$lt": end_time}}
        )
        for log in logs:
            detections.extend(log[0].object_centroids)
            robot.append(log[0].robot_pose.position)
        return detections, robot

    def get_ubd_pos(self, stamp):
        robot = list()
        detections = list()
        start_time = (stamp.secs / 60) * 60
        end_time = start_time + 60
        query = {
            "header.stamp.secs": {"$gte": start_time, "$lt": end_time},
            "$where": "this.ubd_pos.length > 0"
        }
        project = {
            "header.stamp.secs": 1, "ubd_pos": 1, "robot.position.x": 1,
            "robot.position.y": 1, "robot.position.z": 1
        }
        # logs = self._db.query(
        logs = self._db_ubd.find(query, project).sort(
            "header.stamp.secs", pymongo.ASCENDING
        )
        for log in logs:
            temp = list()
            for i in log["ubd_pos"]:
                temp.append(Point(x=i["x"], y=i["y"], z=i["z"]))
            detections.extend(temp)
            robot.append(
                Point(
                    x=log["robot"]["position"]["x"],
                    y=log["robot"]["position"]["y"],
                    z=log["robot"]["position"]["z"]
                )
            )
        return detections, robot

    def annotate(self):
        while not rospy.is_shutdown() and not self._stop:
            logs = self._db.query(
                UbdSceneImgLog._type, {"annotated": False}, limit=30
                # UbdSceneImgLog._type, {"annotated": False}, limit=3  # TESTING PURPOSE
            )
            rospy.loginfo(
                "Getting %d logs from ubd_scene_log collection" % len(logs)
            )
            # projection_query={"robot_data": 0, "skeleton_data": 0}
            for log in logs:
                self._img = log[0].image
                self._ubd, ubd_robot = self.get_ubd_pos(log[0].header.stamp)
                self._cd, cd_robot = self.get_cd_pos(log[0].header.stamp)
                self._robot_pos = ubd_robot + cd_robot
                # self._ubd = log[0].ubd_pos
                # self._cd = log[0].cd_pos
                self._leg_vis(log)
                # annotation part
                act_rois, nact_rois, wp = self._activity_annotation(log)
                self._ubd_annotation(log, act_rois, nact_rois, wp)
                self._cd_annotation(log, act_rois, nact_rois, wp)
                self._leg_annotation(log, act_rois, nact_rois, wp)
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
