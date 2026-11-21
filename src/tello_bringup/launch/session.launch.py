"""Session launch (spec section 8): tello_node, the Vicon adapter, the emulated Vicon (fake backend) and the
safety console.

    ros2 launch tello_bringup session.launch.py [backend:=fake] [session:=YYYY-MM-DD_a] [lab_file:=...]
                                                [console:=xterm|none] [dataset_root:=./dataset]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from tello_bringup.layout import default_session, session_dir
from tello_core.lab import load_lab


def setup(context):
    arg = lambda k: LaunchConfiguration(k).perform(context)  # noqa: E731
    backend, lab_file = arg("backend"), arg("lab_file")
    load_lab(lab_file)                                    # fail now, with all problems listed, if the lab is invalid
    sdir = session_dir(arg("dataset_root"), backend, arg("session"))
    sdir.mkdir(parents=True, exist_ok=True)
    lab = [{"lab_file": lab_file}]
    actions = [
        LogInfo(msg=f"session {arg('session')}: backend {backend}, data in {sdir}"),
        Node(package="tello_driver", executable="tello_node", name="tello_node", output="screen",
             parameters=[{"lab_file": lab_file, "backend": backend}]),
        Node(package="tello_mocap", executable="mocap_adapter", name="mocap_adapter", output="screen", parameters=lab),
    ]
    if backend == "fake":
        actions.append(Node(package="tello_mocap", executable="mocap_sim", name="mocap_sim", output="screen",
                            parameters=lab))
    if arg("console") == "xterm":
        actions.append(Node(package="tello_flight", executable="safety_console", name="safety_console",
                            prefix="xterm -geometry 160x8 -title tello_safety_console -e", output="screen"))
    return actions


def generate_launch_description():
    share = get_package_share_directory("tello_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="fake", description="fake (sdk: Plan 4, sim: Plan 5)"),
        DeclareLaunchArgument("session", default_value=default_session()),
        DeclareLaunchArgument("lab_file", default_value=os.path.join(share, "config", "lab.yaml")),
        DeclareLaunchArgument("console", default_value="xterm", description="xterm or none"),
        DeclareLaunchArgument("dataset_root", default_value=os.path.join(os.getcwd(), "dataset")),
        OpaqueFunction(function=setup),
    ])
