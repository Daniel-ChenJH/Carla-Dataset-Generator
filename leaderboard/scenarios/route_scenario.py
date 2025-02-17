#!/usr/bin/env python

# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides Challenge routes as standalone scenarios
"""

from __future__ import print_function

import glob
import os
import sys
import importlib
import inspect
import json
import py_trees
import traceback
import numpy as np
import random
import math

import carla
from agents.navigation.local_planner import RoadOption

from srunner.scenarioconfigs.scenario_configuration import ActorConfigurationData
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

from srunner.scenariomanager.scenarioatomics.atomic_behaviors import ScenarioTriggerer, Idle
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import WaitForBlackboardVariable
from srunner.scenariomanager.scenarioatomics.atomic_criteria import (CollisionTest,
                                                                     InRouteTest,
                                                                     RouteCompletionTest,
                                                                     OutsideRouteLanesTest,
                                                                     RunningRedLightTest,
                                                                     RunningStopTest,
                                                                     ActorBlockedTest,
                                                                     MinimumSpeedRouteTest)

from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenarios.background_activity import BackgroundBehavior
from srunner.scenariomanager.weather_sim import RouteWeatherBehavior
from srunner.scenariomanager.lights_sim import RouteLightsBehavior
from srunner.scenariomanager.timer import RouteTimeoutBehavior

from leaderboard.utils.route_parser import RouteParser, DIST_THRESHOLD
from leaderboard.utils.route_manipulation import interpolate_trajectory

import leaderboard.utils.parked_vehicles as parked_vehicles

# lane detection
from collections import deque

LIFE_TIME = 10000

lanes = [[], 
         [], 
         [], 
         []]


class RouteScenario(BasicScenario):

    """
    Implementation of a RouteScenario, i.e. a scenario that consists of driving along a pre-defined route,
    along which several smaller scenarios are triggered
    """

    category = "RouteScenario"
    INIT_THRESHOLD = 500 # Runtime initialization trigger distance to ego (m)
    PARKED_VEHICLES_INIT_THRESHOLD = INIT_THRESHOLD - 50 # Runtime initialization trigger distance to parked vehicles (m)

    def __init__(self, world, config, debug_mode=0, criteria_enable=True, json_path=''):
        """
        Setup all relevant parameters and create scenarios along route
        """
        self.client = CarlaDataProvider.get_client()
        self.config = config
        self.route = self._get_route(config)
        self.route_length = len(self.route)
        self.world = world
        self.map = CarlaDataProvider.get_map()
        self.timeout = 10000

        self.all_scenario_classes = None
        self.ego_data = None

        self.scenario_triggerer = None
        self.behavior_node = None # behavior node created by _create_behavior()
        self.criteria_node = None # criteria node created by _create_test_criteria()

        self.list_scenarios = []
        self.occupied_parking_locations = []
        self.available_parking_locations = []
        self.lanemarkings = LaneMarkings()
        self.non_four_wheel_vehicles = [
            "vehicle.bh.crossbike",
            "vehicle.gazelle.omafiets",
            "vehicle.vespa.zx125",
            "vehicle.yamaha.yzf",
            "vehicle.kawasaki.ninja",
            "vehicle.diamondback.century",
            "vehicle.harley-davidson.low_rider",
            "vehicle.carlamotors",
        ]

        self.parking_available_vehicle = [\
            bp for bp in CarlaDataProvider.get_world().get_blueprint_library().filter("vehicle.*")\
            if True not in [bp.id.startswith(vehicle) for vehicle in self.non_four_wheel_vehicles]]


        scenario_configurations = self._filter_scenarios(config.scenario_configs)
        self.scenario_configurations = scenario_configurations
        self.missing_scenario_configurations = scenario_configurations.copy()
        self.json_path = json_path

        ego_vehicle = self._spawn_ego_vehicle()
        if ego_vehicle is None:
            raise ValueError("Shutting down, couldn't spawn the ego vehicle")

        if debug_mode>0:
            self._draw_waypoints(self.route, vertical_shift=0.1, size=0.1, downsample=10)

        self._parked_ids = []
        self._get_parking_slots()

        super(RouteScenario, self).__init__(
            config.name, [ego_vehicle], config, world, debug_mode > 3, False, criteria_enable
        )

        # Do it after the 'super', as we need the behavior and criteria tree to be initialized
        self.build_scenarios(ego_vehicle, debug=debug_mode > 0)

        # Set runtime init mode. Do this after the first set of scenarios has been initialized!
        CarlaDataProvider.set_runtime_init_mode(True)

        # self.camera_rgb_transform = [actor.get_transform() for actor in world.get_actors() if str(actor.id).startswith('Camera_Center')][0]


    def _get_route(self, config):
        """
        Gets the route from the configuration, interpolating it to the desired density,
        saving it to the CarlaDataProvider and sending it to the agent

        Parameters:
        - world: CARLA world
        - config: Scenario configuration (RouteConfiguration)
        - debug_mode: boolean to decide whether or not the route poitns are printed
        """

        # Prepare route's trajectory (interpolate and add the GPS route)
        self.gps_route, self.route, self.carla_waypoint = interpolate_trajectory(config.keypoints)
        return self.route

    def _filter_scenarios(self, scenario_configs):
        """
        Given a list of scenarios, filters out does that don't make sense to be triggered,
        as they are either too far from the route or don't fit with the route shape

        Parameters:
        - scenario_configs: list of ScenarioConfiguration
        """
        new_scenarios_config = []
        for scenario_number, scenario_config in enumerate(scenario_configs):
            trigger_point = scenario_config.trigger_points[0]
            if not RouteParser.is_scenario_at_route(trigger_point, self.route):
                print("WARNING: Ignoring scenario '{}' as it is too far from the route".format(scenario_config.name))
                continue

            scenario_config.route_var_name = "ScenarioRouteNumber{}".format(scenario_number)
            new_scenarios_config.append(scenario_config)

        return new_scenarios_config

    def _spawn_ego_vehicle(self):
        """Spawn the ego vehicle at the first waypoint of the route"""
        elevate_transform = self.route[0][0]
        elevate_transform.location.z += 0.5

        ego_vehicle = CarlaDataProvider.request_new_actor('vehicle.lincoln.mkz_2020',
                                                          elevate_transform,
                                                          rolename='hero')
        if not ego_vehicle:
            return

        spectator = self.world.get_spectator()
        spectator.set_transform(carla.Transform(elevate_transform.location + carla.Location(z=50),
                                                    carla.Rotation(pitch=-90)))

        self.world.tick()

        return ego_vehicle

    def _get_parking_slots(self, max_distance=100, route_step=10):
        """Spawn parked vehicles."""

        def is_close(slot_location):
            for i in range(0, len(self.route), route_step):
                route_transform = self.route[i][0]
                if route_transform.location.distance(slot_location) < max_distance:
                    return True
            return False

        min_x, min_y = float('inf'), float('inf')
        max_x, max_y = float('-inf'), float('-inf')
        for route_transform, _ in self.route:
            min_x = min(min_x, route_transform.location.x - max_distance)
            min_y = min(min_y, route_transform.location.y - max_distance)
            max_x = max(max_x, route_transform.location.x + max_distance)
            max_y = max(max_y, route_transform.location.y + max_distance)

        # Occupied parking locations
        occupied_parking_locations = []
        for scenario in self.list_scenarios:
            occupied_parking_locations.extend(scenario.get_parking_slots())

        available_parking_locations = []
        map_name = self.map.name.split('/')[-1]
        available_parking_locations = getattr(parked_vehicles, map_name, [])

        # Exclude parking slots that are too far from the route
        for slot in available_parking_locations:
            slot_transform = carla.Transform(
                location=carla.Location(slot["location"][0], slot["location"][1], slot["location"][2]),
                rotation=carla.Rotation(slot["rotation"][0], slot["rotation"][1], slot["rotation"][2])
            )

            in_area = (min_x < slot_transform.location.x < max_x) and (min_y < slot_transform.location.y < max_y)
            close_to_route = is_close(slot_transform.location)
            if not in_area or not close_to_route:
                available_parking_locations.remove(slot)
                continue

        self.available_parking_locations = available_parking_locations

    def spawn_parked_vehicles(self, ego_vehicle, max_scenario_distance=10):
        """Spawn parked vehicles."""
        def is_close(slot_location, ego_location):
            return slot_location.distance(ego_location) < self.PARKED_VEHICLES_INIT_THRESHOLD
        def is_free(slot_location):
            for occupied_slot in self.occupied_parking_locations:
                if slot_location.distance(occupied_slot) < max_scenario_distance:
                    return False
            return True

        new_parked_vehicles = []

        ego_location = CarlaDataProvider.get_location(ego_vehicle)
        if ego_location is None:
            return

        for slot in self.available_parking_locations:
            slot_transform = carla.Transform(
                location=carla.Location(slot["location"][0], slot["location"][1], slot["location"][2]),
                rotation=carla.Rotation(slot["rotation"][0], slot["rotation"][1], slot["rotation"][2])
            )

            # Add all vehicles that are close to the ego and in a free space
            # 添加静态类型车辆  类型是static.prop.mesh
            # if is_close(slot_transform.location, ego_location) and is_free(slot_transform.location):
            #     mesh_bp = CarlaDataProvider.get_world().get_blueprint_library().filter("static.prop.mesh")[0]
            #     mesh_bp.set_attribute("mesh_path", slot["mesh"])
            #     mesh_bp.set_attribute("scale", "0.9")
            #     new_parked_vehicles.append(carla.command.SpawnActor(mesh_bp, slot_transform))
            #     self.available_parking_locations.remove(slot)

            # 添加动态车辆 类型是vehicle 
            if is_close(slot_transform.location, ego_location) and is_free(slot_transform.location):
                vehicle_bp = random.choice(self.parking_available_vehicle)
                new_parked_vehicles.append(carla.command.SpawnActor(vehicle_bp, slot_transform))
                self.available_parking_locations.remove(slot)


        # Add the actors to _parked_ids
        for response in CarlaDataProvider.get_client().apply_batch_sync(new_parked_vehicles):
            if not response.error:
                self._parked_ids.append(response.actor_id)

    # pylint: disable=no-self-use
    def _draw_waypoints(self, waypoints, vertical_shift, size, downsample=1):
        """
        Draw a list of waypoints at a certain height given in vertical_shift.
        """
        for i, w in enumerate(waypoints):
            if i % downsample != 0:
                continue

            wp = w[0].location + carla.Location(z=vertical_shift)

            if w[1] == RoadOption.LEFT:  # Yellow 左转路点：黄色
                color = carla.Color(128, 128, 0)
            elif w[1] == RoadOption.RIGHT:  # Cyan  右转路点：青色
                color = carla.Color(0, 128, 128)
            elif w[1] == RoadOption.CHANGELANELEFT:  # Orange 切换到左边车道：橙色
                color = carla.Color(128, 32, 0)
            elif w[1] == RoadOption.CHANGELANERIGHT:  # Dark Cyan  切换到右边车道：深青色
                color = carla.Color(0, 32, 128)
            elif w[1] == RoadOption.STRAIGHT:  # Gray  直行：灰色
                color = carla.Color(64, 64, 64)
            else:  # LANEFOLLOW
                color = carla.Color(0, 128, 0)  # Green  沿着当前车道前进、无特殊操作：绿色

            self.world.debug.draw_point(wp, size=size, color=color, life_time=self.timeout)

        # 路径起点：蓝色
        self.world.debug.draw_point(waypoints[0][0].location + carla.Location(z=vertical_shift), size=2*size,
                                    color=carla.Color(0, 0, 128), life_time=self.timeout)
        # 路径终点：灰色
        self.world.debug.draw_point(waypoints[-1][0].location + carla.Location(z=vertical_shift), size=2*size,
                                    color=carla.Color(128, 128, 128), life_time=self.timeout)
        
        for new_waypoint in self.carla_waypoint:
            lane_markings = self.lanemarkings.calculate3DLanepoints(new_waypoint)

        # Draw all 3D lanemarkings in carla simulator
        for i, color in enumerate(self.lanemarkings.colormap_carla):
            for j in range(0, self.route_length - 1):
                # self.lanemarkings.draw_lanes(self.world, lane_markings[i][j], lane_markings[i][j + 1], self.lanemarkings.colormap_carla[color])
                # if (lane_markings[i][j] and lane_markings[i][j + 1]):
                    # self.world.debug.draw_line(lane_markings[i][j], lane_markings[i][j + 1], thickness=0.05, 
                    #     color=self.lanemarkings.colormap_carla[color], life_time=self.timeout, persistent_lines=False)
                if (lane_markings[i][j]):
                    point = lane_markings[i][j]['coords']
                    carla_point = carla.Location(x=point[0], y=point[1], z=point[2])
                    self.world.debug.draw_point(carla_point, size=0.05, life_time=self.timeout, persistent_lines=False, color=self.lanemarkings.colormap_carla[color])
        
        f = open(os.path.join(self.json_path, 'lanemarkings.json'), 'w', encoding='utf-8')
        lane_dict = {'left_lane':lane_markings[0], 'right_lane':lane_markings[1], 'left_left_lane':lane_markings[2], 'right_right_lane':lane_markings[3]}
        json.dump(lane_dict, f, indent=4)
        f.write('\n')
        f.flush()
        f.close()


    def get_all_scenario_classes(self):
        """
        Searches through the 'scenarios' folder for all the Python classes
        """
        # Path of all scenario at "srunner/scenarios" folder
        scenarios_list = glob.glob("{}/srunner/scenarios/*.py".format(os.getenv('SCENARIO_RUNNER_ROOT', "./").replace('\\','/')))

        all_scenario_classes = {}

        for scenario_file in scenarios_list:

            # Get their module
            module_name = os.path.basename(scenario_file).split('.')[0]
            sys.path.insert(0, os.path.dirname(scenario_file))
            scenario_module = importlib.import_module(module_name)

            # And their members of type class
            for member in inspect.getmembers(scenario_module, inspect.isclass):
                # TODO: Filter out any class that isn't a child of BasicScenario
                all_scenario_classes[member[0]] = member[1]

        return all_scenario_classes

    def build_scenarios(self, ego_vehicle, debug=False):
        """
        Initializes the class of all the scenarios that will be present in the route.
        If a class fails to be initialized, a warning is printed but the route execution isn't stopped
        """
        new_scenarios = []

        if self.all_scenario_classes is None:
            self.all_scenario_classes = self.get_all_scenario_classes()
        if self.ego_data is None:
            self.ego_data = ActorConfigurationData(ego_vehicle.type_id, ego_vehicle.get_transform(), 'hero')

        # Part 1. Check all scenarios that haven't been initialized, starting them if close enough to the ego vehicle
        for scenario_config in self.missing_scenario_configurations:
            scenario_config.ego_vehicles = [self.ego_data]
            scenario_config.route = self.route

            try:
                scenario_class = self.all_scenario_classes[scenario_config.type]
                trigger_location = scenario_config.trigger_points[0].location

                ego_location = CarlaDataProvider.get_location(ego_vehicle)
                if ego_location is None:
                    continue

                # Only init scenarios that are close to ego
                if trigger_location.distance(ego_location) < self.INIT_THRESHOLD:
                    scenario_instance = scenario_class(self.world, [ego_vehicle], scenario_config, timeout=self.timeout)

                    # Add new scenarios to list
                    self.list_scenarios.append(scenario_instance)
                    new_scenarios.append(scenario_instance)
                    self.missing_scenario_configurations.remove(scenario_config)

                    self.occupied_parking_locations.extend(scenario_instance.get_parking_slots())

                    if debug:
                        scenario_loc = scenario_config.trigger_points[0].location
                        debug_loc = self.map.get_waypoint(scenario_loc).transform.location + carla.Location(z=0.2)
                        # 标记场景触发点及名称 触发点红色、名称蓝色
                        self.world.debug.draw_point(
                            debug_loc, size=0.2, color=carla.Color(128, 0, 0), life_time=self.timeout
                        )
                        self.world.debug.draw_string(
                            debug_loc, str(scenario_config.name), draw_shadow=False,
                            color=carla.Color(0, 0, 128), life_time=self.timeout, persistent_lines=True
                        )

            except Exception as e:
                print(f"\033[93mSkipping scenario '{scenario_config.name}' due to setup error: {e}")
                if debug:
                    print(f"\n{traceback.format_exc()}")
                print("\033[0m", end="")
                self.missing_scenario_configurations.remove(scenario_config)
                continue

        # Part 2. Add their behavior onto the route's behavior tree
        for scenario in new_scenarios:

            # Add behavior
            if scenario.behavior_tree is not None:
                self.behavior_node.add_child(scenario.behavior_tree)
                self.scenario_triggerer.add_blackboard(
                    [scenario.config.route_var_name, scenario.config.trigger_points[0].location]
                )

            # Add the criteria criteria
            scenario_criteria = scenario.get_criteria()
            if len(scenario_criteria) == 0:
                continue

            self.criteria_node.add_child(
                self._create_criterion_tree(scenario, scenario_criteria)
            )

    # pylint: enable=no-self-use
    def _initialize_actors(self, config):
        """
        Set other_actors to the superset of all scenario actors
        """
        # Add all the actors of the specific scenarios to self.other_actors
        for scenario in self.list_scenarios:
            self.other_actors.extend(scenario.other_actors)

    def _create_behavior(self):
        """
        Creates a parallel behavior that runs all of the scenarios part of the route.
        These subbehaviors have had a trigger condition added so that they wait until
        the agent is close to their trigger point before activating.

        It also adds the BackgroundActivity scenario, which will be active throughout the whole route.
        This behavior never ends and the end condition is given by the RouteCompletionTest criterion.
        """
        scenario_trigger_distance = DIST_THRESHOLD  # Max trigger distance between route and scenario

        behavior = py_trees.composites.Parallel(name="Route Behavior",
                                                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)

        self.behavior_node = behavior
        scenario_behaviors = []
        blackboard_list = []

        # Add the behavior that manages the scenario trigger conditions
        scenario_triggerer = ScenarioTriggerer(
            self.ego_vehicles[0], self.route, blackboard_list, scenario_trigger_distance)
        behavior.add_child(scenario_triggerer)  # Tick the ScenarioTriggerer before the scenarios

        # register var
        self.scenario_triggerer = scenario_triggerer

        # Add the Background Activity
        behavior.add_child(BackgroundBehavior(self.ego_vehicles[0], self.route, name="BackgroundActivity"))

        behavior.add_children(scenario_behaviors)
        return behavior

    def _create_test_criteria(self):
        """
        Create the criteria tree. It starts with some route criteria (which are always active),
        and adds the scenario specific ones, which will only be active during their scenario
        """
        criteria = py_trees.composites.Parallel(name="Criteria",
                                                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        self.criteria_node = criteria

        # End condition
        criteria.add_child(RouteCompletionTest(self.ego_vehicles[0], route=self.route))

        # 'Normal' criteria
        criteria.add_child(OutsideRouteLanesTest(self.ego_vehicles[0], route=self.route))
        criteria.add_child(CollisionTest(self.ego_vehicles[0], name="CollisionTest"))
        criteria.add_child(RunningRedLightTest(self.ego_vehicles[0]))
        criteria.add_child(RunningStopTest(self.ego_vehicles[0]))
        criteria.add_child(MinimumSpeedRouteTest(self.ego_vehicles[0], self.route, checkpoints=4, name="MinSpeedTest"))

        # These stop the route early to save computational time
        criteria.add_child(InRouteTest(
            self.ego_vehicles[0], route=self.route, offroad_max=30, terminate_on_failure=True))
        criteria.add_child(ActorBlockedTest(
            self.ego_vehicles[0], min_speed=0.1, max_time=180.0, terminate_on_failure=True, name="AgentBlockedTest")
        )

        return criteria

    def _create_weather_behavior(self):
        """
        Create the weather behavior
        """
        if len(self.config.weather) == 1:
            return  # Just set the weather at the beginning and done
        return RouteWeatherBehavior(self.ego_vehicles[0], self.route, self.config.weather)

    def _create_lights_behavior(self):
        """
        Create the street lights behavior
        """
        return RouteLightsBehavior(self.ego_vehicles[0], 100)

    def _create_timeout_behavior(self):
        """
        Create the timeout behavior
        """
        return RouteTimeoutBehavior(self.ego_vehicles[0], self.route)

    def _initialize_environment(self, world):
        """
        Set the weather
        """
        # Set the appropriate weather conditions
        world.set_weather(self.config.weather[0][1])

    def _create_criterion_tree(self, scenario, criteria):
        """
        We can make use of the blackboard variables used by the behaviors themselves,
        as we already have an atomic that handles their (de)activation.
        The criteria will wait until that variable is active (the scenario has started),
        and will automatically stop when it deactivates (as the scenario has finished)
        """
        scenario_name = scenario.name
        var_name = scenario.config.route_var_name
        check_name = "WaitForBlackboardVariable: {}".format(var_name)

        criteria_tree = py_trees.composites.Sequence(name=scenario_name)
        criteria_tree.add_child(WaitForBlackboardVariable(var_name, True, False, name=check_name))

        scenario_criteria = py_trees.composites.Parallel(name=scenario_name,
                                                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        for criterion in criteria:
            scenario_criteria.add_child(criterion)
        scenario_criteria.add_child(WaitForBlackboardVariable(var_name, False, None, name=check_name))

        criteria_tree.add_child(scenario_criteria)
        criteria_tree.add_child(Idle())  # Avoid the indiviual criteria stopping the simulation
        return criteria_tree

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self._parked_ids])
        self.remove_all_actors()



class LaneMarkings():
    """
    Helper class to detect and draw lanemarkings in carla.
    """
    def __init__(self):
        
        self.colormap_carla = {'green': carla.Color(0, 255, 0),
                               'red': carla.Color(255, 0, 0),
                               'yellow': carla.Color(255, 255, 0),
                               'blue': carla.Color(0, 0, 255)}
        
        # carla.LaneMarkingType枚举类数值到字符串的映射
        self.lane_num_to_str = {
            0:'Other',
            1:'Broken',
            2:'Solid',
            3:'SolidSolid',
            4:'SolidBroken',
            5:'BrokenSolid',
            6:'BrokenBroken',
            7:'BottsDots',
            8:'Grass',
            9:'Curb',
            10:'None',
        }
    
    def vector_to_list(self, vector3d):
        return [vector3d.x, vector3d.y, vector3d.z]
            
    
    def calculate3DLanepoints(self, lanepoint):
        """
        Calculates the 3-dimensional position of the lane from the lanepoint from the informations given. 
        The information analyzed is the lanemarking type, the lane type and for the calculation the 
        lanepoint (the middle of the actual lane), the lane width and the driving direction of the lanepoint. 
        In addition to the Lanemarking position, the function does also calculate the position of respectively 
        adjacent lanes. The actual implementation includes contra flow lanes.
        
        Args:
            lanepoint: carla.Waypoint. The lanepoint, of wich the lanemarkings should be calculated.

        Returns:
            lanes: list of 4 lanelists. 3D-positions of every lanemarking of the given lanepoints actual lane, and corresponding neighbour lanes if they exist.
        """
        # 获取当前车道前向向量
        orientationVec = lanepoint.transform.get_forward_vector()
        
        length = math.sqrt(orientationVec.y*orientationVec.y+orientationVec.x*orientationVec.x)
        abVec = carla.Location(orientationVec.y,-orientationVec.x,0) / length * 0.5* lanepoint.lane_width
        # 得到左右车道标记位置
        right_lanemarking = lanepoint.transform.location - abVec 
        left_lanemarking = lanepoint.transform.location + abVec
        # 车道方向判断：.get_right_lane().transform.rotation == .get_right_lane().transform.rotation
        

        # 左右车道只要存在carla标记则加入
        lanes[0].append({'type':self.lane_num_to_str[int(lanepoint.left_lane_marking.type)], 'coords':self.vector_to_list(left_lanemarking)}) if(lanepoint.left_lane_marking.type != carla.LaneMarkingType.NONE) else lanes[0].append(None)
        lanes[1].append({'type':self.lane_num_to_str[int(lanepoint.right_lane_marking.type)], 'coords':self.vector_to_list(right_lanemarking)}) if(lanepoint.right_lane_marking.type != carla.LaneMarkingType.NONE) else lanes[1].append(None)  
    
        # Calculate remaining outer lanes (left and right).
        # 再次向外计算一个车道标记位置
        if(lanepoint.get_left_lane() and lanepoint.get_left_lane().left_lane_marking.type != carla.LaneMarkingType.NONE):
            outer_left_lanemarking  = lanepoint.transform.location + 3 * abVec
            lanes[2].append({'type':self.lane_num_to_str[int(lanepoint.get_left_lane().left_lane_marking.type)], 'coords':self.vector_to_list(outer_left_lanemarking)})
        else:
            lanes[2].append(None)

        if(lanepoint.get_right_lane() and lanepoint.get_right_lane().right_lane_marking.type != carla.LaneMarkingType.NONE):
            outer_right_lanemarking = lanepoint.transform.location - 3 * abVec
            lanes[3].append({'type':self.lane_num_to_str[int(lanepoint.get_right_lane().right_lane_marking.type)], 'coords':self.vector_to_list(outer_right_lanemarking)})
        else:
            lanes[3].append(None)

        # lanes： 左一车道标记，右一车道标记，左二车道标记，右二车道标记
        return lanes

    def draw_points(self, world, point):
        world.debug.draw_point(point + carla.Location(z=0.05), size=0.5, life_time=LIFE_TIME)    
    
    
    def draw_lanes(self, world, point0, point1, color):
        if(point0 and point1):
            world.debug.draw_line(point0 + carla.Location(z=0.05), point1 + carla.Location(z=0.05), thickness=0.5, 
                color=color, life_time=LIFE_TIME)
            self.draw_points(world, point0)
