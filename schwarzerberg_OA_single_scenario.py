# ********************************************************************************
# Copyright (c) 2025 Contributors to the Eclipse Foundation
#
# See the NOTICE file(s) distributed with this work for additional
# information regarding copyright ownership.
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0
#
# SPDX-License-Identifier: EPL-2.0
# ********************************************************************************

from launch import LaunchDescription
from launch_ros.actions import Node

import sys
import os
sys.path.append(os.path.dirname(__file__)) # this line is very importatnt to find the helper functions

from position import Position, Waypoint
from simulated_vehicle import create_simulated_vehicle
from visualizer import create_visualizer

base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if base_dir not in sys.path:
    sys.path.insert(0, base_dir)

launch_file_dir = os.path.dirname(os.path.realpath(__file__))
vehicle_parameters_folder = os.path.abspath(os.path.join(launch_file_dir, "../assets/vehicle_params/"))
maps_folder = os.path.abspath(os.path.join(launch_file_dir, "../assets/tracks/"))
odd_folder = os.path.abspath(os.path.join(launch_file_dir, "../assets/odd/"))

_obstacle_avoidance_candidates = [
    os.path.abspath(
        os.path.join(
            launch_file_dir,
            "../../ros2_workspace/src/adore_ros2_nodes/decision_maker/config/obstacle_avoidance.yaml",
        )
    ),
    os.path.abspath(
        os.path.join(
            launch_file_dir,
            "../../ros2_workspace/src/adore_ros2_nodes/decision_maker/config/obstacle_avoidance.yaml",
        )
    ),
]
obstacle_avoidance_file = next(
    (path for path in _obstacle_avoidance_candidates if os.path.isfile(path)),
    _obstacle_avoidance_candidates[0],
)

planner_params = {
        "dt": 0.1,
        "horizon_steps": 40,
        "lane_error": 0.3,
        "long_error": 0.01,
        "speed_error": 1.0,
        "heading_error": 0.5,
        "steering_angle": 1.0,
        "acceleration": 0.1,
        "max_iterations": 300,
        "max_ms": 80,
        "debug": 0.0,
        "max_lateral_acceleration": 2.0,
        "idm_time_headway": 3.0,
        "ref_traj_length": 200
    }

controller_pid_params = {
    "kp_x": 0.0,
    "ki_x": 0.0,
    "velocity_weight": 0.0,
    "kp_y": 0.4,
    "ki_y": 0.0,
    "heading_weight": 0.75,
    "kp_omega": 0.0,
    "dt": 0.05,
    "steering_comfort": 10000.25,
    "acceleration_comfort": 400.0,
    "acceleration_threshold": 0.25,
    "velocity_threshold": 0.25,
    "constant_brake": -1.0,
    "lookahead_time": 0.3
}

#start_position = Position(lat_long=(52.291613, 10.516043), psi=-3.04)
#goal_position = Position(lat_long=(52.291207, 10.511044), psi=0.0)
start_position = Position(lat_long=(52.291498, 10.514635), psi=-3.04)
goal_position = Position(lat_long=(52.290905, 10.508069), psi=0.0)
#goal_position = Position(lat_long=(52.291074, 10.509670), psi=0.0)
start_pose_utm=start_position.get_utm_coordinates()
goal_position_utm=goal_position.get_utm_coordinates()
# mission_control reads the goal via the "goals" parameter (list of "x,y,stop"
# strings); goal_position_x/y are no longer read after the develop merge.
goal_strings = [f"{x},{y},{stop}"
                for x, y, stop in (wp.to_goal_tuple() for wp in [Waypoint(goal_position)])]
vehicle_id=111
v2x_id=0

start_position_parked_1 = Position(lat_long=(52.291328132295035, 10.512519125479159), psi=-3.0)

start_position_parked_2 = Position(lat_long=(52.291317965673734, 10.512402812494981), psi=-3.0)


def generate_launch_description():
    return LaunchDescription([
        *create_visualizer(
            whitelist=["ego_vehicle"],
            visualization_offset=start_position.get_utm_coordinates(),
        ),
        Node(
            package="simulated_vehicle",
            executable="simulated_vehicle",
            name="simulated_vehicle",
            namespace="ego_vehicle",
            parameters=[
                {"set_start_utm_position_x": start_pose_utm[0]},
                {"set_start_utm_position_y": start_pose_utm[1]},
                {"set_start_utm_zone_number": start_pose_utm[2]},
                {"set_start_utm_zone_letter": start_pose_utm[3]},
                {"set_start_psi": start_pose_utm[4]},
                {"vehicle_id": vehicle_id},
                {"v2x_id": v2x_id},
                {"controllable": True},
                {"vehicle_model_file": vehicle_parameters_folder + "/" + "test_vehicle.json"},
            ],
        ),
        Node(
            package="simulated_vehicle",
            executable="simulated_vehicle",
            name="simulated_vehicle",
            namespace="parked_vehicle_1",
            parameters=[
                {"set_start_utm_position_x": start_position_parked_1.get_utm_coordinates()[0]},
                {"set_start_utm_position_y": start_position_parked_1.get_utm_coordinates()[1]},
                {"set_start_utm_zone_number": start_position_parked_1.get_utm_coordinates()[2]},
                {"set_start_utm_zone_letter": start_position_parked_1.get_utm_coordinates()[3]},
                {"set_start_psi": start_position_parked_1.get_utm_coordinates()[4]},
                {"vehicle_id": 2},
                {"v2x_id": 2},
                {"controllable": True},
                {"vehicle_model_file": vehicle_parameters_folder + "/" + "test_vehicle.json"},
            ],
        ),
        # Node(
        #     package="simulated_vehicle",
        #     executable="simulated_vehicle",
        #     name="simulated_vehicle",
        #     namespace="parked_vehicle_2",
        #     parameters=[
        #         {"set_start_utm_position_x": start_position_parked_2.get_utm_coordinates()[0]},
        #         {"set_start_utm_position_y": start_position_parked_2.get_utm_coordinates()[1]},
        #         {"set_start_utm_zone_number": start_position_parked_2.get_utm_coordinates()[2]},
        #         {"set_start_utm_zone_letter": start_position_parked_2.get_utm_coordinates()[3]},
        #         {"set_start_psi": start_position_parked_2.get_utm_coordinates()[4]},
        #         {"vehicle_id": 3},
        #         {"v2x_id": 3},
        #         {"controllable": True},
        #         {"vehicle_model_file": vehicle_parameters_folder + "/" + "test_vehicle.json"},
        #     ],
        # ),
        Node(
            package="odd_monitor",
            executable="odd_monitor",
            name="odd_monitor",
            namespace="ego_vehicle",
            parameters=[
                {"openodd_file": odd_folder + "/" + "simulation_odd.yaml" },
            ],
        ),
        Node(
            package="mission_control",
            executable="mission_control",
            name="mission_control",
            namespace="ego_vehicle",
            parameters=[
                {"map file": maps_folder + "/" + "de_bs_borders_wfs.r2sr"},  # kept literal key as in original
                {"goals": goal_strings},
                {"local_map_size": 100.0},
                # {"request_assistance_polygon": None},
            ],
        ),
        Node(
            package="decision_maker",
            executable="decision_maker",
            name="decision_maker",
            namespace="ego_vehicle",
            parameters=[
                obstacle_avoidance_file,
                {"planner_settings_keys": list(planner_params.keys())},
                {"planner_settings_values": list(planner_params.values())},
                {"vehicle_model_file": vehicle_parameters_folder + "/" + "test_vehicle.json"},
                {"v2x_id": v2x_id},
            ],
        ),
        Node(
            package="trajectory_tracker",
            executable="trajectory_tracker",
            name="trajectory_tracker",
            namespace="ego_vehicle",
            parameters=[
                {"set_controller": "PID"}, # MPC, PID, iLQR, Passthrough
                {"controller_settings_keys": list(
                    controller_pid_params.keys())},
                {"controller_settings_values": list(
                    controller_pid_params.values())},
                {"vehicle_model_file": vehicle_parameters_folder + "/" + "test_vehicle.json"},
            ],
        ),
    ])
