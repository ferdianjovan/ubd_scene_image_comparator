#!/usr/bin/env python

import rospy
import argparse
from region_observation.util import get_soma_info
from mongodb_store.message_store import MessageStoreProxy
from ubd_scene_image_comparator.msg import UbdSceneAccuracy


class ViewAccuracy(object):

    def __init__(self, config):
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
            used_rois.append(log.region_id)
        rospy.loginfo("Loading...")
        rospy.loginfo("Activity: %s" % str(self.activity))
        rospy.loginfo("UBD: %s" % str(self.ubd))
        rospy.loginfo("Scene: %s" % str(self.cd))

    def view_accuracy(self, region_id):
        # portion of activity in region_id
        p_present = self.activity[region_id]["Present"] / float(
            self.activity[region_id]["Absent"] + self.activity[region_id]["Present"]
        )
        # TPR and TNR UBD
        assert self.activity[region_id]["Present"] == (
            self.ubd[region_id]["TP"] + self.ubd[region_id]["FN"]
        ), "Mismatch between total activity present and TP + FN for UBD"
        tpr_ubd = self.ubd[region_id]["TP"] / float(self.activity[region_id]["Present"])
        assert self.activity[region_id]["Absent"] == (
            self.ubd[region_id]["TN"] + self.ubd[region_id]["FP"]
        ), "Mismatch between total activity absent and TN + FP for UBD"
        tnr_ubd = self.ubd[region_id]["TN"] / float(self.activity[region_id]["Absent"])
        # TPR and TNR CD
        assert self.activity[region_id]["Present"] == (
            self.cd[region_id]["TP"] + self.cd[region_id]["FN"]
        ), "Mismatch between total activity present and TP + FN for CD"
        tpr_cd = self.cd[region_id]["TP"] / float(self.activity[region_id]["Present"])
        assert self.activity[region_id]["Absent"] == (
            self.cd[region_id]["TN"] + self.cd[region_id]["FP"]
        ), "Mismatch between total activity absent and TN + FP for CD"
        tnr_cd = self.cd[region_id]["TN"] / float(self.activity[region_id]["Absent"])

        text = "Probability of activity happening: %.2f \n" % p_present
        text += "UBD True Positive Rate: %.2f, UBD True Negative Rate: %.2f \n" % (
            tpr_ubd, tnr_ubd
        )
        text += "CD True Positive Rate: %.2f, CD True Negative Rate: %.2f \n" % (
            tpr_cd, tnr_cd
        )
        rospy.loginfo(text)


if __name__ == '__main__':
    rospy.init_node("view_accuracy")
    parser = argparse.ArgumentParser(prog=rospy.get_name())
    parser.add_argument("soma_config", help="Soma configuration")
    args = parser.parse_args()
    va = ViewAccuracy(args.soma_config)
    region = raw_input("Region %s: " % str(va.regions.keys()))
    va.view_accuracy(region)
