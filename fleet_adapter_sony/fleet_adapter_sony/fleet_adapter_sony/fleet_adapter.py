import os
import sys
import argparse
import yaml
import nudged
import time
import threading
import math
import numpy as np

import rclpy
import rclpy.node
from rclpy.parameter import Parameter

import rmf_adapter as adpt
import rmf_adapter.vehicletraits as traits
import rmf_adapter.battery as battery
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph
import rmf_adapter.plan as plan

from rmf_task_msgs.msg import TaskProfile, TaskType

from functools import partial

from .RobotCommandHandle import RobotCommandHandle
from .RobotClientAPI import RobotAPI


# -------------------------------------
# Fleet Configuration Class
# -------------------------------------
class FleetConfig:
    def __init__(
        self,
        node,
        config_root: dict,
        fleet_config: dict,
        nav_graph_path,
    ):
        """Configure fleet parameters

        Args:
            node (_type_): _description_
            fleet_config (dict): _description_
            nav_graph_path (_type_): _description_
        """
        self.node = node
        self.config = fleet_config
        self.nav_graph_path = nav_graph_path
        # Profile and traits
        self.profile = traits.Profile(
            geometry.make_final_convex_circle(fleet_config['profile']['footprint']),
            geometry.make_final_convex_circle(fleet_config['profile']['vicinity']))
        self.vehicle_traits = traits.VehicleTraits(
            linear=traits.Limits(*fleet_config['limits']['linear']),
            angular=traits.Limits(*fleet_config['limits']['angular']),
            profile=self.profile)
        self.vehicle_traits.differential.reversible = fleet_config['reversible']
        # Battery system
        self.battery_sys = battery.BatterySystem.make(
            fleet_config['battery_system']['voltage'],
            fleet_config['battery_system']['capacity'],
            fleet_config['battery_system']['charging_current'])
        # Mechanical system
        self.mech_sys = battery.MechanicalSystem.make(
            fleet_config['mechanical_system']['mass'],
            fleet_config['mechanical_system']['moment_of_inertia'],
            fleet_config['mechanical_system']['friction_coefficient'])
        # Power system
        self.ambient_power_sys = battery.PowerSystem.make(
            fleet_config['ambient_system']['power'])
        self.tool_power_sys = battery.PowerSystem.make(
            fleet_config['tool_system']['power'])
        # Power sinks
        self.motion_sink = battery.SimpleMotionPowerSink(self.battery_sys, self.mech_sys)
        self.ambient_sink = battery.SimpleDevicePowerSink(self.battery_sys, self.ambient_power_sys)
        self.tool_sink = battery.SimpleDevicePowerSink(self.battery_sys, self.tool_power_sys)
        # Fleet settings
        self.fleet_name = fleet_config['name']
        # Robot API
        self.api = RobotAPI(
            fleet_config['fleet_manager']['prefix'],
            fleet_config['fleet_manager']['user'],
            fleet_config['fleet_manager']['password'],
            extra_config=self.config_root
        )

    def get_nav_graph(self):
        nav_graph = graph.parse_graph(self.nav_graph_path, self.vehicle_traits)
        return nav_graph

    def init_adapter(self, use_sim_time):
        adapter = adpt.Adapter.make(f"{self.fleet_name}_fleet_adapter")
        if use_sim_time:
            adapter.node.use_sim_time()
        assert adapter, ("Unable to initialize fleet adapter. Please ensure "
                         "RMF Schedule Node is running")
        adapter.start()
        time.sleep(1.0)
        return adapter

    def add_fleet(self, adapter, nav_graph, server_uri):
        fleet_handle = adapter.add_fleet(
            self.fleet_name, self.vehicle_traits, nav_graph, server_uri)
        if not self.config['publish_fleet_state']:
            fleet_handle.fleet_state_publish_period(None)
        # Set task planner params
        set_task_planner_flag = fleet_handle.set_task_planner_params(
            self.battery_sys,
            self.motion_sink,
            self.ambient_sink,
            self.tool_sink,
            self.config['recharge_threshold'],
            self.config['recharge_soc'],
            self.config['account_for_battery_drain'],
            self.config['task_capabilities']['finishing_request'])
        assert set_task_planner_flag, ("Unable to set task planner params")
        return fleet_handle

    def configure_task_requests(self, fleet_handle):
        # Configure task requests
        def _consider(description: dict):
            confirm = adpt.fleet_update_handle.Confirmation()
            confirm.accept()
            return confirm
        task_capabilities = self.config['task_capabilities']
        for category, consider in task_capabilities.items():
            if category == 'loop':
                category = 'patrol'
            if consider:
                if category == 'patrol':
                    self.node.get_logger().info(
                        f"Fleet {self.fleet_name} is configured to perform patrol tasks")
                    fleet_handle.consider_patrol_requests(_consider)
                elif category == 'delivery':
                    self.node.get_logger().info(
                        f"Fleet {self.fleet_name} is configured to perform delivery tasks")
                    fleet_handle.consider_delivery_requests(_consider, _consider)
                elif category == 'clean':
                    self.node.get_logger().info(
                        f"Fleet {self.fleet_name} is configured to perform clean tasks")
                    fleet_handle.consider_cleaning_requests(_consider)
                elif category != 'finishing_request':
                    self.node.get_logger().info(
                        f"Fleet {self.fleet_name} is configured to perform {category} tasks")
                    fleet_handle.consider_usertasks_requests(category, _consider)

# -------------------------------------
# Helper functions
# -------------------------------------
def initialize_fleet(config_yaml, nav_graph_path, node, use_sim_time, server_uri):
    # fleet configuration
    fleet_config = FleetConfig(
        node=node,
        config_root=config_yaml,
        fleet_config=config_yaml['rmf_fleet'],
        nav_graph_path=nav_graph_path)
    adapter = fleet_config.init_adapter(use_sim_time=use_sim_time)
    nav_graph = fleet_config.get_nav_graph()
    fleet_handle = fleet_config.add_fleet(
        adapter=adapter,
        nav_graph=nav_graph,
        server_uri=server_uri)
    fleet_config.configure_task_requests(fleet_handle=fleet_handle)

    # Transforms
    rmf_coordinates = config_yaml['reference_coordinates']['rmf']
    robot_coordinates = config_yaml['reference_coordinates']['robot']
    transforms = {
        'rmf_to_robot': nudged.estimate(rmf_coordinates, robot_coordinates),
        'robot_to_rmf': nudged.estimate(robot_coordinates, rmf_coordinates)}
    transforms['orientation_offset'] = \
        transforms['rmf_to_robot'].get_rotation()
    mse = nudged.estimate_error(transforms['rmf_to_robot'],
                                rmf_coordinates,
                                robot_coordinates)
    print(f"Coordinate transformation error: {mse}")
    print("RMF to Robot transform:")
    print(f"    rotation:{transforms['rmf_to_robot'].get_rotation()}")
    print(f"    scale:{transforms['rmf_to_robot'].get_scale()}")
    print(f"    trans:{transforms['rmf_to_robot'].get_translation()}")
    print("Robot to RMF transform:")
    print(f"    rotation:{transforms['robot_to_rmf'].get_rotation()}")
    print(f"    scale:{transforms['robot_to_rmf'].get_scale()}")
    print(f"    trans:{transforms['robot_to_rmf'].get_translation()}")

    def _updater_inserter(cmd_handle, update_handle):
        """Insert a RobotUpdateHandle."""
        cmd_handle.update_handle = update_handle

    # Initialize robot API for this fleet
    api = fleet_config.api

    # Initialize robots for this fleet

    missing_robots = config_yaml['robots']

    def _add_fleet_robots():
        robots = {}
        while len(missing_robots) > 0:
            time.sleep(0.2)
            for robot_name in list(missing_robots.keys()):
                node.get_logger().debug(f"Connecting to robot: {robot_name}")
                position = api.position(robot_name)
                if position is None:
                    continue
                if len(position) > 2:
                    node.get_logger().info(f"Initializing robot: {robot_name}")
                    robots_config = config_yaml['robots'][robot_name]
                    rmf_config = robots_config['rmf_config']
                    robot_config = robots_config['robot_config']
                    initial_waypoint = rmf_config['start']['waypoint']
                    initial_orientation = rmf_config['start']['orientation']
                    x, y = transforms['robot_to_rmf'].transform(
                        [position[0], position[1]])
                    theta = math.radians(position[2]) - transforms['orientation_offset']
                    if theta > np.pi:
                        theta = theta - (2 * np.pi)
                    if theta < -np.pi:
                        theta = (2 * np.pi) + theta
                    position_rmf = [x, y, theta]

                    starts = []
                    time_now = adapter.now()

                    if (initial_waypoint is not None) and\
                            (initial_orientation is not None):
                        node.get_logger().info(
                            f"Using provided initial waypoint "
                            f"{initial_waypoint} "
                            f"and orientation {initial_orientation:.2f} to "
                            f"initialize starts for robot {robot_name}")
                        # Get the waypoint index for initial_waypoint
                        initial_waypoint_index = nav_graph.find_waypoint(
                            initial_waypoint).index
                        starts = [plan.Start(time_now,
                                             initial_waypoint_index,
                                             initial_orientation)]
                    else:
                        node.get_logger().info(
                            f"Running compute_plan_starts for robot: {robot_name}")
                        starts = plan.compute_plan_starts(
                            nav_graph,
                            rmf_config['start']['map_name'],
                            position_rmf,
                            time_now)

                    if starts is None or len(starts) == 0:
                        node.get_logger().error(
                            f"Unable to determine StartSet for {robot_name}")
                        continue

                    robot = RobotCommandHandle(
                        name=robot_name,
                        fleet_name=fleet_config.fleet_name,
                        config=robot_config,
                        node=node,
                        graph=nav_graph,
                        vehicle_traits=fleet_config.vehicle_traits,
                        transforms=transforms,
                        map_name=rmf_config['start']['map_name'],
                        start=starts[0],
                        position=position_rmf,
                        charger_waypoint=rmf_config['charger']['waypoint'],
                        update_frequency=rmf_config.get(
                            "robot_state_update_frequency", 1),
                        adapter=adapter,
                        api=api)

                    if robot.initialized:
                        robots[robot_name] = robot
                        # Add robot to fleet
                        fleet_handle.add_robot(robot,
                                               robot_name,
                                               fleet_config.profile,
                                               [starts[0]],
                                               partial(_updater_inserter,
                                                       robot))
                        node.get_logger().info(
                            f"Successfully added new robot: {robot_name}")

                    else:
                        node.get_logger().error(
                            f"Failed to initialize robot: {robot_name}")

                    del missing_robots[robot_name]

                else:
                    pass
                    node.get_logger().debug(
                        f"{robot_name} not found, trying again...")
        return

    add_robots = threading.Thread(target=_add_fleet_robots, args=())
    add_robots.start()
    return adapter


# -------------------------------------
# Main
# -------------------------------------
def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    adpt.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="fleet_adapter",
        description="Configure and spin up the fleet adapter")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Path to the config.yaml file")
    parser.add_argument("-n", "--nav_graph", type=str, required=True,
                        help="Path to the nav_graph for this fleet adapter")
    parser.add_argument("-s", "--server_uri", type=str, required=False, default="",
                        help="URI of the api server to transmit state and task information")
    parser.add_argument("--use_sim_time", action="store_true",
                        help="Use sim time, default: false")
    args = parser.parse_args(args_without_ros[1:])
    print(f"Starting fleet adapter...")

    config_path = args.config_file
    nav_graph_path = args.nav_graph

    # Load config and nav graph yamls
    with open(config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    # ROS 2 node for the command handle
    fleet_name = config_yaml['rmf_fleet']['name']
    node = rclpy.node.Node(f"{fleet_name}_command_handle")

    # Enable sim time for testing offline
    if args.use_sim_time:
        param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        node.set_parameters([param])

    if args.server_uri == "":
        server_uri = None
    else:
        server_uri = args.server_uri

    adapter = initialize_fleet(
        config_yaml=config_yaml,
        nav_graph_path=nav_graph_path,
        node=node,
        use_sim_time=args.use_sim_time,
        server_uri=server_uri)

    # Create executor for the command handle node
    rclpy_executor = rclpy.executors.SingleThreadedExecutor()
    rclpy_executor.add_node(node)

    # Start the fleet adapter
    rclpy_executor.spin()

    # Shutdown
    node.destroy_node()
    rclpy_executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
