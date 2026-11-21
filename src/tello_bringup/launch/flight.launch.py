"""Flight launch (spec section 8): dry run, then the recorder and flight_runner. Needs a running session.

    ros2 launch tello_bringup flight.launch.py plan:=<plan.yaml> [session:=...] [notes:="..."] [backend:=fake]
                                               [lab_file:=...] [dataset_root:=./dataset] [shutdown_on_exit:=true]

If the dry run fails, nothing is started and no flight directory is created. When flight_runner exits, the
launch shuts down, which stops the recorder cleanly (SIGINT) so the bag is finalised.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction, \
    RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown as ShutdownEvent
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from tello_bringup.layout import SIMULATED, check_plan, default_session, next_flight_id, record_topics, session_dir
from tello_core.lab import load_lab
from tello_core.plan import load_plan

RECORDER_NODE = "tello_recorder"         # tello_node detects recording by this subscriber name


def setup(context):
    arg = lambda k: LaunchConfiguration(k).perform(context)  # noqa: E731
    backend, plan_file, lab_file, session = arg("backend"), arg("plan"), arg("lab_file"), arg("session")
    lab = load_lab(lab_file)
    plan = load_plan(plan_file)
    report = check_plan(plan, lab)
    if not report.ok:
        problems = "\n  - ".join(report.problems)
        return [LogInfo(msg=f"dry run failed for '{plan.name}', nothing started:\n  - {problems}"),
                Shutdown(reason="dry run failed")]
    sdir = session_dir(arg("dataset_root"), backend, session)
    flight_id = next_flight_id(sdir, session, plan.name)
    fdir = sdir / flight_id
    fdir.mkdir(parents=True)
    topics = record_topics(os.path.join(get_package_share_directory("tello_bringup"), "config",
                                        "record_topics.yaml"), lab, backend)
    cmd = ["ros2", "bag", "record", "-s", "mcap", "-o", str(fdir / "bag"), "--node-name", RECORDER_NODE]
    if backend == "sim":
        cmd.append("--use-sim-time")
    recorder = ExecuteProcess(cmd=cmd + ["--topics", *topics], output="screen", name="recorder")
    runner = Node(package="tello_flight", executable="flight_runner", name="flight_runner", output="screen",
                  parameters=[{"plan_file": os.path.abspath(plan_file), "lab_file": lab_file,
                               "flight_dir": str(fdir), "flight_id": flight_id, "session": session,
                               "backend": backend, "notes": arg("notes"), "repo_dir": os.getcwd(),
                               "use_sim_time": backend == "sim"}])
    actions = [LogInfo(msg=f"dry run OK ({report.duration:.0f} s simulated); flight {flight_id} -> {fdir}"),
               recorder, runner]
    if arg("shutdown_on_exit") == "true":
        actions.append(RegisterEventHandler(OnProcessExit(target_action=runner, on_exit=[
            EmitEvent(event=ShutdownEvent(reason="flight_runner finished"))])))
    return actions


def generate_launch_description():
    share = get_package_share_directory("tello_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("plan", description="path to the flight plan YAML"),
        DeclareLaunchArgument("session", default_value=default_session()),
        DeclareLaunchArgument("notes", default_value=""),
        DeclareLaunchArgument("backend", default_value="fake"),
        DeclareLaunchArgument("lab_file", default_value=os.path.join(share, "config", "lab.yaml")),
        DeclareLaunchArgument("dataset_root", default_value=os.path.join(os.getcwd(), "dataset")),
        DeclareLaunchArgument("shutdown_on_exit", default_value="true"),
        OpaqueFunction(function=setup),
    ])
