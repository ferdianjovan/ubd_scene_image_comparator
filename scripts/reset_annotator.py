#!/usr/bin/env python

import rospy
from mongodb_store.message_store import MessageStoreProxy
from ubd_scene_image_comparator.msg import UbdSceneImgLog


if __name__ == '__main__':
    rospy.init_node("ubd_scene_reset_annotator")
    rospy.loginfo("Resetting all annotated logs...")
    db = MessageStoreProxy(collection="ubd_scene_log")
    logs = db.query(
        UbdSceneImgLog._type, {"annotated": True},
        limit=10
    )
    while not rospy.is_shutdown() and len(logs) > 0:
        rospy.loginfo("Resetting %d entries" % len(logs))
        for log in logs:
            log[0].annotated = False
            db.update(
                log[0],
                message_query={"header.stamp.secs": log[0].header.stamp.secs}
            )
        logs = db.query(
            UbdSceneImgLog._type, {"annotated": True},
            limit=10
        )
    rospy.loginfo("Done...")
    rospy.loginfo(
        "Please remove entries on ubd_scene_accuracy if you want to have fresh learnt accuracy"
    )
    rospy.spin()
