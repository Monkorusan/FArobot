import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ur_moveit_config.launch_common import load_yaml


def generate_launch_description() -> LaunchDescription:
    share_dir = get_package_share_directory("FArobot")
    default_config = os.path.join(share_dir, "config", "boxes.yaml")

    ur_type = "ur5e"
    description_package = "ur_description"
    description_file = "ur.urdf.xacro"
    moveit_config_package = "ur_moveit_config"
    moveit_config_file = "ur.srdf.xacro"
    prefix = '""'

    joint_limit_params = os.path.join(
        get_package_share_directory(description_package),
        "config",
        ur_type,
        "joint_limits.yaml",
    )
    kinematics_params = os.path.join(
        get_package_share_directory(description_package),
        "config",
        ur_type,
        "default_kinematics.yaml",
    )
    physical_params = os.path.join(
        get_package_share_directory(description_package),
        "config",
        ur_type,
        "physical_parameters.yaml",
    )
    visual_params = os.path.join(
        get_package_share_directory(description_package),
        "config",
        ur_type,
        "visual_parameters.yaml",
    )

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            os.path.join(
                get_package_share_directory(description_package),
                "urdf",
                description_file,
            ),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=true",
            " ",
            "safety_pos_margin:=0.15",
            " ",
            "safety_k_position:=20",
            " ",
            "name:=ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_description_semantic_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            os.path.join(
                get_package_share_directory(moveit_config_package),
                "srdf",
                moveit_config_file,
            ),
            " ",
            "name:=ur",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    kinematics_yaml = load_yaml("ur_moveit_config", "config/kinematics.yaml")
    if "robot_description_kinematics" in kinematics_yaml:
        kinematics_params = kinematics_yaml["robot_description_kinematics"]
    elif "/**" in kinematics_yaml:
        kinematics_params = kinematics_yaml["/**"]["ros__parameters"]["robot_description_kinematics"]
    else:
        kinematics_params = kinematics_yaml

    robot_description_kinematics = {
        "robot_description_kinematics": kinematics_params
    }
    robot_description_planning = {
        "robot_description_planning": load_yaml(
            "ur_moveit_config",
            "config/joint_limits.yaml",
        )
    }
    planning_pipelines = {
        "planning_pipelines": {
            "pipeline_names": ["ompl"],
            "namespace": "",
        }
    }

    ompl_planning_pipeline_config = {
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    ur_moveit_launch = os.path.join(
        get_package_share_directory("ur_moveit_config"),
        "launch",
        "ur_moveit.launch.py",
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ur_moveit_launch),
                launch_arguments={"ur_type": ur_type}.items(),
            ),
            Node(
                package="FArobot",
                executable="robot_description_publisher_node",
                name="robot_description_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            Node(
                package="FArobot",
                executable="packing_demo_node",
                name="packing_demo_node",
                output="screen",
                parameters=[
                    {"config": default_config},
                    {"frame_id": "base_link"},
                    {"publish_rate_hz": 1.0},
                    {"show_bin": False},
                ],
            ),
            Node(
                package="FArobot",
                executable="pick_place_executor_node",
                name="pick_place_executor_node",
                output="screen",
                parameters=[
                    {"config": default_config},
                    {"planning_group": "ur_manipulator"},
                    {"eef_link": "tool0"},
                    {"base_frame": "base_link"},
                    {"approach_offset": 0.08},
                    {"lift_offset": 0.12},
                    {"auto_start": True},
                    {"execution_mode": "simulate"},
                    {"enable_attach": False},
                    {"max_boxes": 1},
                    {"enable_place_override": True},
                    {"place_override_xyz": [-0.5, -0.5, 0.05]},
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                    robot_description_planning,
                    planning_pipelines,
                    ompl_planning_pipeline_config,
                    moveit_controllers,
                    trajectory_execution,
                ],
            ),
        ]
    )
