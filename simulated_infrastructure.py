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

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

from launch import Action
from launch_ros.actions import Node

import os
import sys
import utm

base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if base_dir not in sys.path:
    sys.path.insert(0, base_dir)

launch_file_dir = os.path.dirname(os.path.realpath(__file__))
vehicle_parameters_folder = os.path.abspath(os.path.join(launch_file_dir, "../assets/vehicle_params/"))
maps_folder = os.path.abspath(os.path.join(launch_file_dir, "../assets/tracks/"))

def create_simulated_infrastructure(
    infrastructure_position_utm: Tuple[float, float, int, str, float],
    polygon_utm: list[Tuple[float, float, int, str, float]],
    map_file: str = "de_bs_borders_wfs.r2sr",
    namespace: str = "infrastructure",
) -> List[Action]:

    validity_polygon_in_utm = []
    for point in polygon_utm:
        validity_polygon_in_utm.append(point[0])
        validity_polygon_in_utm.append(point[1])

    multi_agent_pid_planner_parameters = {
        "preview_distance": 4.5,
        "k_yaw": 2.0,
        "k_distance": 1.0
    }

    return [
        Node(
            package='decision_maker_infrastructure',
            namespace=namespace,
            executable='decision_maker_infrastructure',
            name='decision_maker_infrastructure',
            parameters=[
                {"map file": maps_folder + "/" + map_file},
                {"infrastructure_position_x": infrastructure_position_utm[0]},
                {"infrastructure_position_y": infrastructure_position_utm[1]},
                {"infrastructure_yaw": infrastructure_position_utm[4]},
                {"max_participant_age": 0.5},
                {"max_route_length": 200.0},
                {"route_replan_dist": 5.0}, # If the vehicle deviates with more than 5m from its route, it recalculates
                {"local_map_size": 200.0},
                {"should_publish_local_map": True}, # only used for visualization
                {"debug": True},
                {"validity_polygon": validity_polygon_in_utm},
                {"multi_agent_PID_settings_keys": list(multi_agent_pid_planner_parameters.keys())},
                {"multi_agent_PID_settings_values": list(multi_agent_pid_planner_parameters.values())},
            ],
        )
    ]
