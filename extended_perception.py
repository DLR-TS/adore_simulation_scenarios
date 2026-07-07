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
from simulated_infrastructure import create_simulated_infrastructure

start_position = Position(lat_long=(52.291331, 10.513119), psi=0.0)
goal_positions = [
    Waypoint(Position(lat_long=(52.291327, 10.512913)), WaypointBehavior.STOP),
]

infrastructure_position = Position(lat_long=(52.291412, 10.514557), psi=0.0)
infrastructure_polygon = [
    Position(lat_long=(52.291376, 10.514187)).get_utm_coordinates(),
    Position(lat_long=(52.291481, 10.514139)).get_utm_coordinates(),
    Position(lat_long=(52.291540, 10.514884)).get_utm_coordinates(),
    Position(lat_long=(52.291435, 10.514892)).get_utm_coordinates(),
]

def generate_launch_description():
    return LaunchDescription([
        *create_simulated_vehicle(
            namespace="ego_vehicle",
            start_position_utm=start_position.get_utm_coordinates(),
            goals=goal_positions,
            vehicle_id=111,
            v2x_id=0,
            sensor_range=0.0,
        ),
        *create_simulated_vehicle(
            namespace="object1",
            start_position_utm=Position(utm=(603288.8704, 5794535.249, 32, "U"), psi=0.0).get_utm_coordinates(),
            goals=goal_positions,
            vehicle_id=112,
            v2x_id=112,
            controllable=False
        ),
        *create_simulated_infrastructure(
            infrastructure_position_utm=infrastructure_position.get_utm_coordinates(),
            polygon_utm=infrastructure_polygon,
            namespace="infrastructure"
        ),
        *create_visualizer(
            whitelist=["ego_vehicle", "infrastructure"],
            visualization_offset=start_position.get_utm_coordinates(),
        )
    ])
