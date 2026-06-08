#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
cartographer_cycler.py ¡ª Cartographer ¶¨Ê±ÂÖ×ªÊØ»¤½Úµã

¹¦ÄÜ£ºÃ¿¸ô CYCLE_PERIOD Ãë×Ô¶¯´¥·¢Ò»´Î Cartographer ¹ì¼£ÖØÖÃ£¨FinishTrajectory + StartTrajectory£©£¬
     ÊµÏÖ"·þÎñ¼¶ÈÈÖØÆô"£¬±ÜÃâ½Úµã¼¶É±/Æô´øÀ´µÄ pbstream ÖØÔØ¿ªÏú¡£
     
     Óë cartographer_guard.py µÄÇø±ð£º
     - guard ÊÇ»ùÓÚ·ÖÊý/À×´ï´¥·¢µÄ±»¶¯»Ö¸´£¨¶¨Î»¶ªÁË²Å»Ö¸´£©
     - cycler ÊÇ»ùÓÚ¶¨Ê±Æ÷µÄÖ÷¶¯ÂÖ×ª£¨Ô¤·ÀÐÔ¶¨ÆÚÐ£×¼£©

ÂÖ×ªÁ÷³Ì£º
  1. »º´æµ±Ç° map->base_link Î»×Ë
  2. µ÷ÓÃ /finish_trajectory ½áÊøµ±Ç°¹ì¼£
  3. µ÷ÓÃ /start_trajectory ÒÔ»º´æµÄÎ»×ËÎª initial_pose Æô¶¯ÐÂ¹ì¼£
  4. µÈ´ýÎÈ¶¨£¨Ä¬ÈÏ 5s£©£¬ÆÚ¼ä·¢²¼ÁãËÙ¶ÈËø¶¨»úÆ÷ÈË

Ê¹ÓÃ·½·¨£¨ÔÚ launch ÎÄ¼þÖÐÌí¼Ó£©£º
  <node pkg="cartographer_ros" type="cartographer_cycler.py" name="cartographer_cycler" output="screen">
    <param name="cycle_period" value="180" />          <!-- 3·ÖÖÓ -->
    <param name="stabilizing_duration" value="5" />
    <param name="hold_topic" value="/cmd_vel" />
    <param name="finish_service" value="/finish_trajectory" />
    <param name="start_service" value="/start_trajectory" />
    <param name="config_dir" value="$(find cartographer_ros)/configuration_files" />
    <param name="config_file" value="livox_2d_localization.lua" />
  </node>
"""

import math
import rospy
import tf2_ros
from geometry_msgs.msg import Pose, Twist, TransformStamped
from std_msgs.msg import Bool
from cartographer_ros_msgs.srv import FinishTrajectory, StartTrajectory
from cartographer_ros_msgs.srv import FinishTrajectoryRequest, StartTrajectoryRequest


class CartographerCycler:
    """¶¨Ê±ÂÖ×ª Cartographer ¹ì¼££¬ÓÃÓÚ¶¨ÆÚÐ£×¼¶¨Î»¡£"""

    def __init__(self):
        # ========== ÂÖ×ª²ÎÊý ==========
        self.cycle_period = rospy.get_param("~cycle_period", 180)          # ÂÖ×ªÖÜÆÚ£¨Ãë£©£¬Ä¬ÈÏ 3 ·ÖÖÓ
        self.stabilizing_duration = rospy.get_param("~stabilizing_duration", 5.0)  # ÂÖ×ªºóÎÈ¶¨µÈ´ý
        self.hold_topic = rospy.get_param("~hold_topic", "/cmd_vel")       # Ëø¶¨»úÆ÷ÈËÓÃµÄ cmd_vel »°Ìâ

        # ========== Cartographer ·þÎñ ==========
        self.finish_service_name = rospy.get_param("~finish_service", "/finish_trajectory")
        self.start_service_name = rospy.get_param("~start_service", "/start_trajectory")
        self.config_dir = rospy.get_param("~config_dir", "")
        self.config_file = rospy.get_param("~config_file", "")

        # ========== TF ==========
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ========== ·¢²¼Æ÷ ==========
        self.cmd_vel_pub = rospy.Publisher(self.hold_topic, Twist, queue_size=1)
        self.holding_pub = rospy.Publisher("~holding", Bool, queue_size=1, latch=True)
        self._publish_holding(False)

        # ========== ×´Ì¬ ==========
        self.trajectory_id = 0
        self.is_cycling = False
        self.last_cycle_time = rospy.Time.now().to_sec()

        # ========== µÈ´ý Cartographer ·þÎñ¾ÍÐ÷ ==========
        rospy.loginfo(f"Waiting for service [{self.finish_service_name}] ...")
        rospy.wait_for_service(self.finish_service_name)
        rospy.loginfo(f"Waiting for service [{self.start_service_name}] ...")
        rospy.wait_for_service(self.start_service_name)
        self.finish_srv = rospy.ServiceProxy(self.finish_service_name, FinishTrajectory)
        self.start_srv = rospy.ServiceProxy(self.start_service_name, StartTrajectory)
        rospy.loginfo("CartographerCycler: services connected")

        # ========== Ö÷Ñ­»·¶¨Ê±Æ÷£¨1Hz ¼ì²é£© ==========
        self.check_timer = rospy.Timer(rospy.Duration(1.0), self.check_cycle)
        # Ëø¶¨ÆÚ¼ä³ÖÐø·¢ÁãËÙ¶È
        self.hold_timer = rospy.Timer(rospy.Duration(0.05), self.publish_hold)

        rospy.loginfo(f"CartographerCycler started: cycle_period={self.cycle_period}s, "
                      f"stabilizing={self.stabilizing_duration}s")

    def _publish_holding(self, holding):
        try:
            self.holding_pub.publish(Bool(data=bool(holding)))
        except Exception:
            pass

    def _get_current_pose(self):
        """´Ó TF »ñÈ¡µ±Ç° map->base_footprint Î»×Ë£¨ÓÃÓÚ×¢ÈëÐÂ¹ì¼£µÄ initial_pose£©"""
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame,
                rospy.Time(0), rospy.Duration(0.5))
            pose = Pose()
            pose.position.x = tf_stamped.transform.translation.x
            pose.position.y = tf_stamped.transform.translation.y
            pose.position.z = tf_stamped.transform.translation.z
            pose.orientation = tf_stamped.transform.rotation
            return pose
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF lookup failed ({self.map_frame}->{self.base_frame}): {e}")
            return None

    def publish_hold(self, event):
        """ÂÖ×ªÆÚ¼ä·¢²¼ÁãËÙ¶ÈËø¶¨»úÆ÷ÈË¡£"""
        if not self.is_cycling:
            return
        self.cmd_vel_pub.publish(Twist())

    def check_cycle(self, event):
        """¶¨Ê±¼ì²é£ºÖÜÆÚµ½ÁË¾Í´¥·¢ÂÖ×ª¡£"""
        if self.is_cycling:
            return

        now = rospy.Time.now().to_sec()
        elapsed = now - self.last_cycle_time

        if elapsed >= self.cycle_period:
            rospy.loginfo(f"Cycle period reached ({elapsed:.0f}s), starting trajectory rotation...")
            self.do_rotation()

    def do_rotation(self):
        """Ö´ÐÐÒ»´Î¹ì¼£ÂÖ×ª£ºfinish ¾É¹ì¼£ ¡ú start ÐÂ¹ì¼££¨ÒÔµ±Ç°Î»×ËÎª initial_pose£©"""
        self.is_cycling = True
        self._publish_holding(True)
        now = rospy.Time.now().to_sec()

        rospy.logwarn("=" * 50)
        rospy.logwarn(f"[Cycler] Starting trajectory rotation at t={now:.0f}s")

        # 1. »º´æµ±Ç°Î»×Ë
        cached_pose = self._get_current_pose()
        if cached_pose is not None:
            p = cached_pose.position
            q = cached_pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            rospy.loginfo(f"[Cycler] Cached pose: x={p.x:.2f}, y={p.y:.2f}, yaw={yaw:.2f}rad")
        else:
            rospy.logwarn("[Cycler] No cached pose available, will cold-start global localization")

        try:
            # 2. ½áÊøµ±Ç°¹ì¼£
            req = FinishTrajectoryRequest()
            req.trajectory_id = self.trajectory_id
            self.finish_srv(req)
            rospy.loginfo(f"[Cycler] Finished trajectory {self.trajectory_id}")

            rospy.sleep(0.5)

            # 3. Æô¶¯ÐÂ¹ì¼£
            req = StartTrajectoryRequest()
            req.configuration_directory = self.config_dir
            req.configuration_basename = self.config_file
            req.relative_to_trajectory_id = 0

            if cached_pose is not None:
                req.use_initial_pose = False
                req.initial_pose = cached_pose
            else:
                req.use_initial_pose = False

            resp = self.start_srv(req)
            self.trajectory_id = resp.trajectory_id
            rospy.logwarn(f"[Cycler] Started new trajectory {self.trajectory_id}")

        except Exception as e:
            rospy.logerr(f"[Cycler] Rotation failed: {e}")
            self.is_cycling = False
            self._publish_holding(False)
            self.last_cycle_time = rospy.Time.now().to_sec()
            return

        # 4. µÈ´ýÎÈ¶¨ÆÚ
        rospy.loginfo(f"[Cycler] Stabilizing for {self.stabilizing_duration}s...")
        rospy.sleep(self.stabilizing_duration)

        # 5. ÊÍ·Å
        self.is_cycling = False
        self._publish_holding(False)
        self.last_cycle_time = rospy.Time.now().to_sec()

        rospy.logwarn(f"[Cycler] Rotation complete. Next cycle in {self.cycle_period}s")
        rospy.logwarn("=" * 50)


if __name__ == "__main__":
    rospy.init_node("cartographer_cycler")
    cycler = CartographerCycler()
    rospy.spin()
