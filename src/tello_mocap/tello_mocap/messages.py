"""The three Vicon message types the stack accepts (lab.yaml mocap.raw_type), read and built uniformly."""
from __future__ import annotations

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry

TYPES = {"PoseStamped": PoseStamped, "TransformStamped": TransformStamped, "Odometry": Odometry}


def read_pose(msg):
    """(header, (x, y, z), (qx, qy, qz, qw)) from any accepted type."""
    if isinstance(msg, PoseStamped):
        p, q = msg.pose.position, msg.pose.orientation
    elif isinstance(msg, TransformStamped):
        p, q = msg.transform.translation, msg.transform.rotation
    else:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return msg.header, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)


def build_pose(type_name: str, header, p, q, child_frame: str):
    msg = TYPES[type_name]()
    msg.header = header
    if type_name == "PoseStamped":
        pos, ori = msg.pose.position, msg.pose.orientation
    elif type_name == "TransformStamped":
        msg.child_frame_id = child_frame
        pos, ori = msg.transform.translation, msg.transform.rotation
    else:
        msg.child_frame_id = child_frame
        pos, ori = msg.pose.pose.position, msg.pose.pose.orientation
    pos.x, pos.y, pos.z = (float(v) for v in p)
    ori.x, ori.y, ori.z, ori.w = (float(v) for v in q)
    return msg
