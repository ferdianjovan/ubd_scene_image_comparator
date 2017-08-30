#!/usr/bin/env python

import yaml
import rospy
import argparse
from region_observation.util import get_soma_info
from mongodb_store.message_store import MessageStoreProxy
from ubd_scene_image_comparator.msg import UbdSceneAccuracy


class ViewAccuracy(object):

    def __init__(self, config):
        self.config = config
        self.regions, self.map = get_soma_info(config)
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
        self._db_store = MessageStoreProxy(collection="ubd_scene_accuracy")
        self.load_accuracy()

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

    def view_accuracy(self, region_id):
        # portion of activity in region_id
        p_present = self.activity[region_id]["Present"] / float(
            self.activity[region_id]["Absent"] + self.activity[region_id]["Present"]
        )
        data = self.calculate_accuracy_percentage(region_id)

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

    def calculate_accuracy_percentage(self, region_id):
        data = {"scene": dict(), "upper_body": dict(), "trajectory": dict()}
        # TPR
        assert self.activity[region_id]["Present"] == (
            self.ubd[region_id]["TP"] + self.ubd[region_id]["FN"]
        ), "Mismatch between total activity present and TP + FN for UBD"
        assert self.activity[region_id]["Present"] == (
            self.cd[region_id]["TP"] + self.cd[region_id]["FN"]
        ), "Mismatch between total activity present and TP + FN for CD"
        assert self.activity[region_id]["Present"] == (
            self.leg[region_id]["TP"] + self.leg[region_id]["FN"]
        ), "Mismatch between total activity present and TP + FN for LEG"
        if self.activity[region_id]["Present"] > 0:
            data["upper_body"]["tp"] = self.ubd[region_id]["TP"] / float(self.activity[region_id]["Present"])
            data["scene"]["tp"] = self.cd[region_id]["TP"] / float(self.activity[region_id]["Present"])
            data["trajectory"]["tp"] = self.leg[region_id]["TP"] / float(self.activity[region_id]["Present"])
        else:
            data["upper_body"]["tp"] = 0.0
            data["scene"]["tp"] = 0.0
            data["trajectory"]["tp"] = 0.0
        # TNR
        assert self.activity[region_id]["Absent"] == (
            self.ubd[region_id]["TN"] + self.ubd[region_id]["FP"]
        ), "Mismatch between total activity absent and TN + FP for UBD"
        assert self.activity[region_id]["Absent"] == (
            self.cd[region_id]["TN"] + self.cd[region_id]["FP"]
        ), "Mismatch between total activity absent and TN + FP for CD"
        assert self.activity[region_id]["Absent"] == (
            self.leg[region_id]["TN"] + self.leg[region_id]["FP"]
        ), "Mismatch between total activity absent and TN + FP for CD"
        if self.activity[region_id]["Absent"] > 0:
            data["upper_body"]["tn"] = self.ubd[region_id]["TN"] / float(self.activity[region_id]["Absent"])
            data["scene"]["tn"] = self.cd[region_id]["TN"] / float(self.activity[region_id]["Absent"])
            data["trajectory"]["tn"] = self.leg[region_id]["TN"] / float(self.activity[region_id]["Absent"])
        else:
            data["upper_body"]["tn"] = 0.0
            data["scene"]["tn"] = 0.0
            data["trajectory"]["tn"] = 0.0
        return data

    def store_to_file(self, path):
        data = {
            region_id: self.calculate_accuracy_percentage(region_id) for region_id in self.regions
        }
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
        va.store_to_file(path)
    else:
        region = raw_input("Region %s to view: " % str(va.regions.keys()))
        va.view_accuracy(region)
