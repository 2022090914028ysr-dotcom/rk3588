#!/usr/bin/env python3

import math
import re
from typing import Dict, Optional, Tuple

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


def _yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _normalize_text(text: str) -> str:
    if text is None:
        return ""
    text = text.strip().lower()
    # Keep unicode word characters (includes Chinese), drop punctuation.
    # Note: in Python3, \w is unicode-aware by default.
    text = re.sub(r"[^\w\s]", " ", text, flags=re.UNICODE)
    text = re.sub(r"\s+", " ", text).strip()
    return text


def _extract_target_phrase(text: str) -> str:
    """Return the part after common command prefixes, else the full text."""
    raw = (text or "").strip()
    if not raw:
        return ""

    # Check raw (non-normalized) first for Chinese commands like "去厨房" (no spaces).
    raw_prefixes = [
        "去到",
        "去",
        "前往",
        "导航到",
        "带我去",
    ]
    for p in raw_prefixes:
        if raw.startswith(p):
            return raw[len(p):].strip()

    t = _normalize_text(raw)
    if not t:
        return ""

    prefixes = [
        "go to ",
        "navigate to ",
        "move to ",
        "take me to ",
        "go ",
        "navigate ",
        "move ",
    ]
    for p in prefixes:
        if t.startswith(p):
            return t[len(p):].strip()
    return t


def _canonicalize_place_name(raw: str) -> str:
    """Map common variants to canonical keys."""
    t = _normalize_text(raw)

    # collapse spaced forms
    t = t.replace("bed room", "bedroom")
    t = t.replace("hall room", "hall")

    aliases = {
        # English
        "meeting room": "meeting_room",
        "meetingroom": "meeting_room",
        "living room": "living_room",
        "livingroom": "living_room",
        "bedroom": "bedroom",
        "kitchen": "kitchen",
        "balcony": "balcony",
        "balcoy": "balcony",  # user-typed variant
        "hall": "hall",
        "hallway": "hall",

        # Chinese
        "厨房": "kitchen",
        "会议室": "meeting_room",
        "会议厅": "meeting_room",
        "客厅": "living_room",
        "起居室": "living_room",
        "阳台": "balcony",
        "卧室": "bedroom",
        "卧房": "bedroom",
        "大厅": "hall",
        "大堂": "hall",
        "走廊": "hall",
    }

    if t in aliases:
        return aliases[t]

    # If user already says canonical with underscore
    t2 = t.replace(" ", "_")
    return aliases.get(t2, t2)


def _load_places_from_param() -> Dict[str, Dict[str, float]]:
    raw = rospy.get_param("~places", {})
    if not isinstance(raw, dict):
        rospy.logwarn("[voice_goal_router] ~places is not a dict; got %s", type(raw))
        return {}

    out: Dict[str, Dict[str, float]] = {}
    for key, val in raw.items():
        if not isinstance(val, dict):
            rospy.logwarn("[voice_goal_router] place '%s' must be a dict; got %s", key, type(val))
            continue
        try:
            x = float(val.get("x", 0.0))
            y = float(val.get("y", 0.0))
            yaw = float(val.get("yaw", 0.0))
            out[str(key)] = {"x": x, "y": y, "yaw": yaw}
        except Exception as e:
            rospy.logwarn("[voice_goal_router] invalid place '%s': %s", key, e)
    return out


def _find_place(places: Dict[str, Dict[str, float]], utterance: str) -> Optional[str]:
    """Try exact match on canonical key, else substring match."""
    target = _canonicalize_place_name(_extract_target_phrase(utterance))
    if not target:
        return None

    if target in places:
        return target

    # Try matching by value in utterance (e.g., 'go to the kitchen')
    u = _normalize_text(utterance)
    for k in places.keys():
        k_norm = _normalize_text(k).replace("_", " ")
        if k_norm and k_norm in u:
            return k
    return None


def _make_pose(frame_id: str, x: float, y: float, yaw: float) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = 0.0
    qx, qy, qz, qw = _yaw_to_quat(yaw)
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return msg


class VoiceGoalRouter:
    def __init__(self):
        self.voice_topic = rospy.get_param("~voice_topic", "/voice/text")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.frame_id = rospy.get_param("~frame_id", "map")

        self.places = _load_places_from_param()
        rospy.loginfo("[voice_goal_router] loaded %d places", len(self.places))
        if self.places:
            rospy.loginfo("[voice_goal_router] places: %s", ", ".join(sorted(self.places.keys())))

        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1, latch=True)
        self._sub = rospy.Subscriber(self.voice_topic, String, self._on_text, queue_size=10)

        rospy.loginfo("[voice_goal_router] listening on %s; publishing goals to %s (frame=%s)",
                      self.voice_topic, self.goal_topic, self.frame_id)

    def _on_text(self, msg: String):
        utterance = msg.data or ""
        place_key = _find_place(self.places, utterance)
        if not place_key:
            rospy.logwarn("[voice_goal_router] no place matched for: '%s'", utterance)
            return

        p = self.places.get(place_key)
        if not p:
            rospy.logwarn("[voice_goal_router] matched place '%s' missing data", place_key)
            return

        pose = _make_pose(self.frame_id, p["x"], p["y"], p["yaw"])
        self.goal_pub.publish(pose)
        rospy.loginfo("[voice_goal_router] goal sent: %s -> x=%.3f y=%.3f yaw=%.3f",
                      place_key, p["x"], p["y"], p["yaw"])


def main():
    rospy.init_node("voice_goal_router", anonymous=False)
    VoiceGoalRouter()
    rospy.spin()


if __name__ == "__main__":
    main()
