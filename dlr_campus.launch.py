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
sys.path.append(os.path.dirname(__file__))
from position import Position, Waypoint, WaypointBehavior
from simulated_vehicle import create_simulated_vehicle
from visualizer import create_visualizer

start_position = Position(lat_long=(52.315849, 10.562169), psi=0.0)
goal_positions = [
    Waypoint(Position(utm=(606471.04, 5797161.11, 32, "U")), WaypointBehavior.STOP),
]

def generate_launch_description():
    return LaunchDescription([
        *create_simulated_vehicle(
            namespace="ego_vehicle",
            start_position_utm=start_position.get_utm_coordinates(),
            goals=goal_positions,
            vehicle_id=111,
            v2x_id=0,
        ),
        *create_visualizer(
            whitelist=["ego_vehicle"],
            visualization_offset=start_position.get_utm_coordinates(),
        )
    ])
