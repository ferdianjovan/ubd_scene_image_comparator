#!/usr/bin/env python

import yaml
import rospy
import argparse
from region_observation.util import get_soma_info
from mongodb_store.message_store import MessageStoreProxy
from ubd_scene_image_comparator.msg import UbdSceneAccuracy

from strands_navigation_msgs.msg import TopologicalMap


class ViewAccuracy(object):

    def __init__(self, config):
        self.config = config
        self.regions, self.map = get_soma_info(config)
        self.topo_map = None
        self._get_waypoints()
        self.topo_map = [wp.name for wp in self.topo_map.nodes]
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
        self._db_store = MessageStoreProxy(collection="ubd_scene_accuracy")
        self.load_accuracy()

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

    def view_accuracy(self, region_id):
        # portion of activity in region_id
        for wp in self.topo_map:
            try:
                p_present = self.activity[region_id][wp]["Present"] / float(
                    self.activity[region_id][wp]["Absent"] + self.activity[region_id][wp]["Present"]
                )
            except ZeroDivisionError:
                continue
            data = self.calculate_accuracy_percentage(region_id, wp)
            rospy.loginfo("Region Config: %s" % wp)
            text = "Probability of activity happening: %.3f \n" % p_present
            text += "UBD True Positive Rate: %.3f, UBD True Negative Rate: %.3f \n" % (
                data["upper_body"]["tp"], data["upper_body"]["tn"]
            )
            text += "CD True Positive Rate: %.3f, CD True Negative Rate: %.3f \n" % (
                data["scene"]["tp"], data["scene"]["tn"]
            )
            text += "LEG True Positive Rate: %.3f, LEG True Negative Rate: %.3f \n" % (
                data["trajectory"]["tp"], data["trajectory"]["tn"]
            )
            rospy.loginfo(text)

    def calculate_accuracy_percentage(self, region_id, region_config):
        data = {"scene": dict(), "upper_body": dict(), "trajectory": dict()}
        # TPR
        assert self.activity[region_id][region_config]["Present"] == (
            self.ubd[region_id][region_config]["TP"] + self.ubd[region_id][region_config]["FN"]
        ), "Mismatch between total activity present and TP + FN for UBD"
        assert self.activity[region_id][region_config]["Present"] == (
            self.cd[region_id][region_config]["TP"] + self.cd[region_id][region_config]["FN"]
        ), "Mismatch between total activity present and TP + FN for CD"
        assert self.activity[region_id][region_config]["Present"] == (
            self.leg[region_id][region_config]["TP"] + self.leg[region_id][region_config]["FN"]
        ), "Mismatch between total activity present and TP + FN for LEG"
        if self.activity[region_id][region_config]["Present"] > 0:
            data["upper_body"]["tp"] = self.ubd[region_id][region_config]["TP"] / float(
                self.activity[region_id][region_config]["Present"]
            )
            data["scene"]["tp"] = self.cd[region_id][region_config]["TP"] / float(
                self.activity[region_id][region_config]["Present"]
            )
            data["trajectory"]["tp"] = self.leg[region_id][region_config]["TP"] / float(
                self.activity[region_id][region_config]["Present"]
            )
        else:
            data["upper_body"]["tp"] = 0.0
            data["scene"]["tp"] = 0.0
            data["trajectory"]["tp"] = 0.0
        # TNR
        assert self.activity[region_id][region_config]["Absent"] == (
            self.ubd[region_id][region_config]["TN"] + self.ubd[region_id][region_config]["FP"]
        ), "Mismatch between total activity absent and TN + FP for UBD"
        assert self.activity[region_id][region_config]["Absent"] == (
            self.cd[region_id][region_config]["TN"] + self.cd[region_id][region_config]["FP"]
        ), "Mismatch between total activity absent and TN + FP for CD"
        assert self.activity[region_id][region_config]["Absent"] == (
            self.leg[region_id][region_config]["TN"] + self.leg[region_id][region_config]["FP"]
        ), "Mismatch between total activity absent and TN + FP for CD"
        if self.activity[region_id][region_config]["Absent"] > 0:
            data["upper_body"]["tn"] = self.ubd[region_id][region_config]["TN"] / float(
                self.activity[region_id][region_config]["Absent"]
            )
            data["scene"]["tn"] = self.cd[region_id][region_config]["TN"] / float(
                self.activity[region_id][region_config]["Absent"]
            )
            data["trajectory"]["tn"] = self.leg[region_id][region_config]["TN"] / float(
                self.activity[region_id][region_config]["Absent"]
            )
        else:
            data["upper_body"]["tn"] = 0.0
            data["scene"]["tn"] = 0.0
            data["trajectory"]["tn"] = 0.0
        return data

    def remake_unperfect(self, sensor_accuracy):
        error = 0.005
        temp = {i: {"tp": 0.0, "tn": 0.0} for i in sensor_accuracy}
        for sensory_type, vals in sensor_accuracy.iteritems():
            for i, j in vals.iteritems():
                if j == 1.0:
                    temp[sensory_type][i] = j - error
                elif j == 0.0:
                    temp[sensory_type][i] = j + error
                else:
                    temp[sensory_type][i] = j
        return temp

    def store_to_file(self, path, no_perfect_sensor=False):
        data = dict()
        for region_id in self.regions:
            for wp in self.topo_map:
                temp = self.calculate_accuracy_percentage(region_id, wp)
                summation = sum(temp["upper_body"].values()) + sum(temp["scene"].values())
                summation += sum(temp["trajectory"].values())
                if summation == 0.0:
                    continue
                if region_id not in data:
                    data[region_id] = dict()
                if no_perfect_sensor:
                    temp = self.remake_unperfect(temp)
                data[region_id][wp] = temp
        # data = {
        #     region_id: {
        #         wp: self.calculate_accuracy_percentage(region_id, wp) for wp in self.topo_map
        #     } for region_id in self.regions
        # }
        with open("%s/%s.yaml" % (path, self.map), 'w') as f:
            f.write(yaml.dump({self.config: data}))


if __name__ == '__main__':
    rospy.init_node("view_accuracy")
    parser = argparse.ArgumentParser(prog=rospy.get_name())
    parser.add_argument("soma_config", help="Soma configuration")
    parser.add_argument(
        "-s", dest="store_to_file", default="0",
        help="Storing accuracy to file with the provided path [1(yes) / 0(no)]"
    )
    args = parser.parse_args()
    va = ViewAccuracy(args.soma_config)
    if int(args.store_to_file):
        path = raw_input("Store to (path): ")
        no_perfect = raw_input("Applying 'No perfect sensor' [1(yes) / 0(no)]: ")
        va.store_to_file(path, int(no_perfect))
    else:
        region = raw_input("Region %s to view: " % str(va.regions.keys()))
        va.view_accuracy(region)
