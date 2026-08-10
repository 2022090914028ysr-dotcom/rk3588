#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
cartographer_cycler.py �� Cartographer ��ʱ��ת�ػ��ڵ�

���ܣ�ÿ�� CYCLE_PERIOD ���Զ�����һ�� Cartographer �켣���ã�FinishTrajectory + StartTrajectory����
     ʵ��"����������"������ڵ㼶ɱ/�������� pbstream ���ؿ�����
     
     �� cartographer_guard.py ������
     - guard �ǻ��ڷ���/�״ﴥ���ı����ָ�����λ���˲Żָ���
     - cycler �ǻ��ڶ�ʱ����������ת��Ԥ���Զ���У׼��

��ת���̣�
  1. ���浱ǰ map->base_link λ��
  2. ���� /finish_trajectory ������ǰ�켣
  3. ���� /start_trajectory �Ի����λ��Ϊ initial_pose �����¹켣
  4. �ȴ��ȶ���Ĭ�� 5s�����ڼ䷢�����ٶ�����������

ʹ�÷������� launch �ļ������ӣ���
  <node pkg="cartographer_ros" type="cartographer_cycler.py" name="cartographer_cycler" output="screen">
    <param name="cycle_period" value="180" />          <!-- 3���� -->
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
    """��ʱ��ת Cartographer �켣�����ڶ���У׼��λ��"""

    def __init__(self):
        # ========== ��ת���� ==========
        self.cycle_period = rospy.get_param("~cycle_period", 180)          # ��ת���ڣ��룩��Ĭ�� 3 ����
        self.stabilizing_duration = rospy.get_param("~stabilizing_duration", 5.0)  # ��ת���ȶ��ȴ�
        self.hold_topic = rospy.get_param("~hold_topic", "/cmd_vel")       # �����������õ� cmd_vel ����

        # ========== Cartographer ���� ==========
        self.finish_service_name = rospy.get_param("~finish_service", "/finish_trajectory")
        self.start_service_name = rospy.get_param("~start_service", "/start_trajectory")
        self.config_dir = rospy.get_param("~config_dir", "")
        self.config_file = rospy.get_param("~config_file", "")

        # ========== TF ==========
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ========== ������ ==========
        self.cmd_vel_pub = rospy.Publisher(self.hold_topic, Twist, queue_size=1)
        self.holding_pub = rospy.Publisher("~holding", Bool, queue_size=1, latch=True)
        self._publish_holding(False)

        # ========== ״̬ ==========
        self.trajectory_id = 0
        self.is_cycling = False
        self.last_cycle_time = rospy.Time.now().to_sec()

        # ========== �ȴ� Cartographer ������� ==========
        rospy.loginfo(f"Waiting for service [{self.finish_service_name}] ...")
        rospy.wait_for_service(self.finish_service_name)
        rospy.loginfo(f"Waiting for service [{self.start_service_name}] ...")
        rospy.wait_for_service(self.start_service_name)
        self.finish_srv = rospy.ServiceProxy(self.finish_service_name, FinishTrajectory)
        self.start_srv = rospy.ServiceProxy(self.start_service_name, StartTrajectory)
        rospy.loginfo("CartographerCycler: services connected")

        # ========== ��ѭ����ʱ����1Hz ��飩 ==========
        self.check_timer = rospy.Timer(rospy.Duration(1.0), self.check_cycle)
        # �����ڼ���������ٶ�
        self.hold_timer = rospy.Timer(rospy.Duration(0.05), self.publish_hold)

        rospy.loginfo(f"CartographerCycler started: cycle_period={self.cycle_period}s, "
                      f"stabilizing={self.stabilizing_duration}s")

    def _publish_holding(self, holding):
        try:
            self.holding_pub.publish(Bool(data=bool(holding)))
        except Exception:
            pass

    def _get_current_pose(self):
        """�� TF ��ȡ�base_linkootprint λ�ˣ�����ע���¹켣�� initial_pose��"""
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
        """��ת�ڼ䷢�����ٶ����������ˡ�"""
        if not self.is_cycling:
            return
        self.cmd_vel_pub.publish(Twist())

    def check_cycle(self, event):
        """��ʱ��飺���ڵ��˾ʹ�����ת��"""
        if self.is_cycling:
            return

        now = rospy.Time.now().to_sec()
        elapsed = now - self.last_cycle_time

        if elapsed >= self.cycle_period:
            rospy.loginfo(f"Cycle period reached ({elapsed:.0f}s), starting trajectory rotation...")
            self.do_rotation()

    def do_rotation(self):
        """ִ��һ�ι켣��ת��finish �ɹ켣 �� start �¹켣���Ե�ǰλ��Ϊ initial_pose��"""
        self.is_cycling = True
        self._publish_holding(True)
        now = rospy.Time.now().to_sec()

        rospy.logwarn("=" * 50)
        rospy.logwarn(f"[Cycler] Starting trajectory rotation at t={now:.0f}s")

        # 1. ���浱ǰλ��
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
            # 2. ������ǰ�켣
            req = FinishTrajectoryRequest()
            req.trajectory_id = self.trajectory_id
            self.finish_srv(req)
            rospy.loginfo(f"[Cycler] Finished trajectory {self.trajectory_id}")

            rospy.sleep(0.5)

            # 3. �����¹켣
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

        # 4. �ȴ��ȶ���
        rospy.loginfo(f"[Cycler] Stabilizing for {self.stabilizing_duration}s...")
        rospy.sleep(self.stabilizing_duration)

        # 5. �ͷ�
        self.is_cycling = False
        self._publish_holding(False)
        self.last_cycle_time = rospy.Time.now().to_sec()

        rospy.logwarn(f"[Cycler] Rotation complete. Next cycle in {self.cycle_period}s")
        rospy.logwarn("=" * 50)


if __name__ == "__main__":
    rospy.init_node("cartographer_cycler")
    cycler = CartographerCycler()
    rospy.spin()
