#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
tf_frame_switch.py ¡ª TF Ö¡Ãû×ª·¢/ÇÐ»»½Úµã

ÅäºÏ cartographer_node_watchdog.sh Ê¹ÓÃ£º
  - Ô¤ÈÈÆÚ¼ä cartographer_node ·¢²¼ map_warmup ¡ú odom_warmup
  - ¾É½Úµã±»É±ºó£¬±¾½Úµã°Ñ map_warmup ¡ú odom_warmup ×ª³É map ¡ú odom ·¢³ö
  - µ¼º½Õ»Ê¼ÖÕ¿´µ½ map ¡ú odom£¬ÎÞÐèÐÞ¸ÄÈÎºÎÏÂÓÎ½Úµã

¿ØÖÆ½Ó¿Ú£º
  - ·þÎñ ~enable  (std_srvs/SetBool):  true=¿ªÊ¼×ª·¢, false=Í£Ö¹×ª·¢
  - Ò²¿ÉÍ¨¹ý²ÎÊý ~active ¿ØÖÆ

·­Òë¹æÔò£¨¿ÉÀ©Õ¹£©£º
  map_warmup ¡ú map
  odom_warmup ¡ú odom
"""

import rospy
import tf2_ros
import tf2_msgs.msg
from std_srvs.srv import SetBool, SetBoolResponse
import threading


class TfFrameSwitch:
    """¶©ÔÄ /tf£¬°´Ðè½« warmup Ö¡Ãû·­ÒëÎªÕýÊ½Ö¡ÃûºóÖØÐÂ·¢²¼¡£"""

    def __init__(self):
        # ·­ÒëÓ³Éä£ºwarmup_frame ¡ú real_frame
        self.translation_map = {
            "map_warmup": "map",
            "odom_warmup": "odom",
        }

        self.active = False
        self.lock = threading.Lock()

        # ¶©ÔÄ /tf£¨°üÀ¨ warmup Ö¡ºÍÕýÊ½Ö¡£©
        self.tf_sub = rospy.Subscriber("/tf", tf2_msgs.msg.TFMessage, self.tf_callback)

        # ÖØÐÂ·¢²¼·­ÒëºóµÄ TF
        self.tf_pub = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=100)

        # ¿ØÖÆ·þÎñ£º~enable (true/false)
        self.enable_srv = rospy.Service("~enable", SetBool, self.handle_enable)

        # Ò²Ö§³ÖÍ¨¹ý²ÎÊý¶¯Ì¬ÇÐ»»
        self.active = rospy.get_param("~active", False)

        rospy.loginfo(f"TF Frame Switch started, active={self.active}")
        rospy.loginfo(f"Translation map: {self.translation_map}")

    def handle_enable(self, req):
        with self.lock:
            self.active = req.data
        rospy.logwarn(f"TF Frame Switch: active = {self.active}")
        return SetBoolResponse(success=True, message=f"active={self.active}")

    def tf_callback(self, msg):
        """ÊÕµ½ TF ÏûÏ¢£¬Èç¹ûÐèÒªÔò·­Òëºó×ª·¢¡£"""
        if not self.active:
            return

        # ¼ì²éÊÇ·ñÐèÒª·­Òë
        needs_translation = False
        for transform in msg.transforms:
            if (transform.header.frame_id in self.translation_map or
                    transform.child_frame_id in self.translation_map):
                needs_translation = True
                break

        if not needs_translation:
            return

        # ¹¹Ôì·­ÒëºóµÄÏûÏ¢
        new_msg = tf2_msgs.msg.TFMessage()
        for transform in msg.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id

            # ·­Òë frame Ãû
            new_parent = self.translation_map.get(parent, parent)
            new_child = self.translation_map.get(child, child)

            # Ö»×ª·¢ÓÐ±ä»¯µÄ£¨±ÜÃâ°Ñ·Ç warmup µÄÖ¡ÖØ¸´·¢³öÈ¥£©
            if new_parent == parent and new_child == child:
                continue

            t = tf2_msgs.msg.TransformStamped()
            t.header.stamp = transform.header.stamp
            t.header.frame_id = new_parent
            t.child_frame_id = new_child
            t.transform = transform.transform
            new_msg.transforms.append(t)

        if new_msg.transforms:
            self.tf_pub.publish(new_msg)


if __name__ == "__main__":
    rospy.init_node("tf_frame_switch")
    node = TfFrameSwitch()
    rospy.spin()
