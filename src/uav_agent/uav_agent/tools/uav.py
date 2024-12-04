#  Copyright (c) 2024. Jet Propulsion Laboratory. All rights reserved.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#  https://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

from math import cos, sin, sqrt, acos, atan2, pi
from typing import List
import numpy as np

import time

import rclpy
from rclpy.node import Node
from rclpy import qos
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from langchain.agents import tool
from std_srvs.srv import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from pyquaternion import Quaternion

def clamp(n, smallest, largest): return max(smallest, min(n, largest))

cmd_vel_pubs = {}

def add_cmd_vel_pub(name: str, publisher: Node.publishers):
    global cmd_vel_pubs
    cmd_vel_pubs[name] = publisher

def remove_cmd_vel_pub(name: str):
    global cmd_vel_pubs
    cmd_vel_pubs.pop(name, None)

def get_wpt_orient_from_current(name,x,y,z) -> Quaternion():
    target_position = np.array([x,y,z])
    current_pose = get_robot_pose.invoke({"names": [name]})
    current_position = np.array([current_pose[name].position.x, current_pose[name].position.y, current_pose[name].position.z])
    position_error_in_world = target_position-current_position
    target_orient = Quaternion(axis=(0.0, 0.0, 1.0), radians=atan2(position_error_in_world[1],position_error_in_world[0]))
    return target_orient

def get_wpt_orient_from_prev(name,x,y,z,prev_wpt) -> Quaternion(): 
    target_position = np.array([x,y,z])
    prev_position = np.array(prev_wpt[1],prev_wpt[2],prev_wpt[3])
    position_error_in_world = target_position-prev_position
    target_orient = Quaternion(axis=(0.0, 0.0, 1.0), radians=atan2(position_error_in_world[1],position_error_in_world[0]))
    return target_orient


waypoint_list_all_robots = {}

def add_wpt(name, x, y, z):
    global waypoint_list_all_robots
    waypoint_list_all_robots[name].append(list((name,x,y,z,_)))
    if (len(waypoint_list_all_robots[name])==1):
        waypoint_list_all_robots[name][len(waypoint_list_all_robots[name])-1][4] = get_wpt_orient_from_current(name,x,y,z)
    else:
        prev_wpt = waypoint_list_all_robots[name][len(waypoint_list_all_robots[name])-2]
        waypoint_list_all_robots[name][len(waypoint_list_all_robots[name])-1][4] = get_wpt_orient_from_prev(name,x,y,z,prev_wpt)

def clear_wpt(name):
    global waypoint_list_all_robots
    waypoint_list_all_robots[name].clear()

# # Add the default robot publisher on startup
# qos_profile = QoSProfile(
#             reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
#             history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
#             depth=1
#         )
# add_cmd_vel_pub("alfa", Node.create_publisher(msg_type=Twist, topic="/alfa/cmd_vel", qos_profile=qos_profile))

pose = Pose()
twist = Twist()
def pose_callback(msg):
    global pose, twist
    pose = msg.pose.pose
    twist = msg.twist.twist


def within_bounds(x: float, y: float) -> tuple:
    """
    Check if the given x, y coordinates are within the bounds of the environment.

    :param x: The x-coordinate.
    :param y: The y-coordinate.
    """
    if -50 <= x <= 50 and -50 <= y <= 50:
        return True, "Coordinates are within bounds."
    else:
        return False, f"({x}, {y}) will be out of bounds. Range is [-50, 50] for each."


def will_be_within_bounds(
    name: str, velocity: float, lateral: float, angle: float, duration: float = 1.0
) -> tuple:
    """Check if the robot will be within bounds after publishing a twist command."""
    # Get the current pose of the robot
    pose = get_robot_pose.invoke({"names": [name]})
    current_x = pose[name].position.x
    current_y = pose[name].position.y
    current_theta = 2*acos(pose[name].orientation.w)

    # Calculate the new position and orientation
    if abs(angle) < 1e-6:  # Straight line motion
        new_x = (
            current_x
            + (velocity * cos(current_theta) - lateral * sin(current_theta)) * duration
        )
        new_y = (
            current_y
            + (velocity * sin(current_theta) + lateral * cos(current_theta)) * duration
        )
    else:  # Circular motion
        radius = sqrt(velocity**2 + lateral**2) / abs(angle)
        center_x = current_x - radius * sin(current_theta)
        center_y = current_y + radius * cos(current_theta)
        angle_traveled = angle * duration
        new_x = center_x + radius * sin(current_theta + angle_traveled)
        new_y = center_y - radius * cos(current_theta + angle_traveled)

        # Check if any point on the circle is out of bounds
        for t in range(int(duration) + 1):
            angle_t = current_theta + angle * t
            x_t = center_x + radius * sin(angle_t)
            y_t = center_y - radius * cos(angle_t)
            in_bounds, _ = within_bounds(x_t, y_t)
            if not in_bounds:
                return (
                    False,
                    f"The circular path will go out of bounds at ({x_t:.2f}, {y_t:.2f}).",
                )

    # Check if the final x, y coordinates are within bounds
    in_bounds, message = within_bounds(new_x, new_y)
    if not in_bounds:
        return (
            False,
            f"This command will move the robot out of bounds to ({new_x:.2f}, {new_y:.2f}).",
        )

    return True, f"The robot will remain within bounds at ({new_x:.2f}, {new_y:.2f})."

# def get_robot_pose(names_str: str):

@tool
def get_robot_pose(names: List[str]) -> dict:
    """
    Get the pose of one or more robots.

    :param names: List of string of names of the robots to get the pose of. It is not a string. Use [ and ] to create the list of string separated by commas.
    """

    # Remove any forward slashes from the names
    # names = names_str.split()
    names = [name.replace("/", "") for name in names]
    poses = {}

    global pose
    # Get the pose of each robot
    for name in names:
        try:
            poses[name] = pose
        except Exception as e:
            return {
                "Error": f"Failed to get pose for {name}: /{name}/openvins/poseimu not available."
            }
    return poses

@tool
def get_robot_twist(names: List[str]) -> dict:
    """
    Get the twist of one or more robots.

    :param names: List of string of names of the robots to get the twist of. It is not a string. Use [ and ] to create the list of string separated by commas.
    """

    # Remove any forward slashes from the names
    # names = names_str.split()
    names = [name.replace("/", "") for name in names]
    twists = {}

    global twist
    # Get the pose of each robot
    for name in names:
        try:
            twists[name] = twist
        except Exception as e:
            return {
                "Error": f"Failed to get pose for {name}: /{name}/openvins/poseimu not available."
            }
    return twists

@tool
def add_robot_waypoint(name: str, wpt_x: float, wpt_y: float, wpt_z: float = 0.0, clear_previous: bool = False) -> tuple:
    """
    Adds one waypoint for a robot to a list.

    :param name: name of the robot (do not include the forward slash)
    :param wpt_x: x-coordinate of waypoint
    :param wpt_y: x-coordinate of waypoint
    :param wpt_z: x-coordinate of waypoint or altitude
    :param clear_previous: set to True to delete any previous waypoints that were already present in the waypoint list
    """
    if (clear_previous):
        clear_wpt(name)
    add_wpt(name, wpt_x, wpt_y, wpt_z)
    return True, f"Added a waypoint ({wpt_x}, {wpt_y}, {wpt_z}) for {name}."

async def publish_cmd(name, wpt, tolerance) -> tuple:
    target_orient = wpt[3]
    target_position = np.array([wpt[0],wpt[1],wpt[2]])
    in_bounds, _ = within_bounds(target_position[0], target_position[2])
    if not in_bounds:
        return True, f"The target coordinates is out of bounds at ({target_x:.2f}, {target_y:.2f})."

    try:
        global cmd_vel_pubs
        pub = cmd_vel_pubs[name]
    except Exception as e:
        return True, f"Did not find publisher for {name}/cmd_vel: {e}"
    
    vel = Twist()

    current_pose = get_robot_pose.invoke({"names": [name]})
    current_position = np.array([current_pose[name].position.x, current_pose[name].position.y, current_pose[name].position.z])
    distance_to_target = np.linalg.norm(target_position-current_position)
    current_twist = get_robot_twist.invoke({"names": [name]})
    current_velocity = np.array([current_twist[name].linear.x, current_twist[name].linear.y, current_twist[name].linear.z])
    position_error_in_world = target_position-current_position
    current_orient = Quaternion(current_pose[name].orientation.w,current_pose[name].orientation.x,current_pose[name].orientation.y,current_pose[name].orientation.z)
    position_error_in_body = (current_orient.inverse).rotate(position_error_in_world)

    max_error_mag = 10.0
    if (distance_to_target > max_error_mag):
        position_error_unit_vector = position_error_in_body / distance_to_target
        position_error_in_body = max_error_mag*position_error_unit_vector

    orient_error_quat = target_orient * (current_orient.conjugate)
    orient_error = orient_error_quat[3]
    orient_error = clamp(orient_error,-0.5,0.5)

    if (distance_to_target<tolerance+0.1):
        return True, f"{name} has moved to ({current_pose[name].position.x}, {current_pose[name].position.y}, {current_pose[name].position.z})."

    vel.linear.x = 0.5*position_error_in_body[0] - 0.5*current_velocity[0]
    vel.linear.y = 0.5*position_error_in_body[1] - 0.5*current_velocity[1]
    vel.linear.z = 1.0*position_error_in_body[2] - 0.5*current_velocity[2]
    vel.angular.x, vel.angular.y, vel.angular.z = 0.0, 0.0, -2.0*orient_error + 0.4*current_twist[name].angular.z

    try:
        pub.publish(vel)
        # time.sleep(0.05)
    except Exception as e:
        return True, f"Failed to publish {vel} to /{name}/cmd_vel: {e}"

async def mission_loop():
    global waypoint_list_all_robots
    for waypoint_list in waypoint_list_all_robots:
        if (len(waypoint_list)>0):
            active_waypoint = waypoint_list[0]
            name = active_waypoint[0]
            wpt = active_waypoint[1:4]
            reached,_ = publish_cmd(name,wpt,0.5)
            if (reached): 
                waypoint_list.pop([0])

# @tool
# def publish_twist_to_cmd_vel(
#     name: str,
#     longitudinal: float,
#     lateral: float,
#     vertical: float,
#     yaw_rate: float,
#     steps: int = 1,
# ):
#     """
#     Publish a Twist message to the /{name}/cmd_vel topic to move the robot.
#     Use a combination of linear and angular velocities to move the robot in the desired direction.

#     :param name: name of the robot (do not include the forward slash)
#     :param longitudinal: longitudinal velocity, where positive is forward and negative is backward
#     :param lateral: lateral velocity, where positive is left and negative is right
#     :param vertical: vertical velocity, where positive is up and negative is down
#     :param yaw_rate: angular velocity, where positive is counterclockwise and negative is clockwise
#     :param steps: Number of times to publish the twist message
#     """
#     # Remove any forward slashes from the name
#     name = name.replace("/", "")

#     # Check if the movement will keep the turtle within bounds
#     in_bounds, message = will_be_within_bounds(
#         name, longitudinal, lateral, yaw_rate, duration=steps*0.01
#     )
#     if not in_bounds:
#         return message

#     vel = Twist()
#     vel.linear.x, vel.linear.y, vel.linear.z = longitudinal, lateral, vertical
#     vel.angular.x, vel.angular.y, vel.angular.z = 0.0, 0.0, yaw_rate

#     try:
#         global cmd_vel_pubs
#         pub = cmd_vel_pubs[name]

#         for _ in range(steps):
#             pub.publish(vel)
#             time.sleep(0.1)
#     except Exception as e:
#         return f"Failed to publish {vel} to /{name}/cmd_vel: {e}"
#     finally:
#         current_pose = get_robot_pose.invoke({"names": [name]})
#         return (
#             f"New Pose ({name}): x={current_pose[name].position.x}, y={current_pose[name].position.y}, "
#             f"theta={2*acos(current_pose[name].orientation.w)} rads, "
#             # f"linear_velocity={current_pose[name].linear_velocity}, "
#             # f"angular_velocity={current_pose[name].angular_velocity}."
#         )

# @tool
# def move_robot_to(name: str, target_x: float, target_y: float, target_z: float = 0.0, tolerance: float = 0.5) -> tuple:
#     """
#     Move the robot to target coordinates.

#     :param name: name of the robot (do not include the forward slash)
#     :param target_x: target x-coordinate
#     :param target_y: target y-coordinate
#     :param target_z: target z-coordinate or altitude
#     :param tolerance: tolerance level to determine if robot has reached the target (default 0.5)
#     """
#     target_position = np.array([target_x,target_y,target_z])
#     in_bounds, _ = within_bounds(target_x, target_y)
#     if not in_bounds:
#         return False, f"The target coordinates is out of bounds at ({target_x:.2f}, {target_y:.2f})."

#     try:
#         global cmd_vel_pubs
#         pub = cmd_vel_pubs[name]
#     except Exception as e:
#         return False, f"Did not find publisher for {name}/cmd_vel: {e}"
        
#     # current_pose = get_robot_pose.invoke({"names": [name]})
#     # current_position = np.array([current_pose[name].position.x, current_pose[name].position.y, current_pose[name].position.z])
#     # return False, f"New Pose ({name}): x={current_position[0]}, y={current_position[1]}, z={current_position[2]},"
    
#     vel = Twist()
#     target_orient = Quaternion()
#     target_heading_not_set = True
#     while True:
#         current_pose = get_robot_pose.invoke({"names": [name]})
#         current_position = np.array([current_pose[name].position.x, current_pose[name].position.y, current_pose[name].position.z])
#         distance_to_target = np.linalg.norm(target_position-current_position)
#         current_twist = get_robot_twist.invoke({"names": [name]})
#         current_velocity = np.array([current_twist[name].linear.x, current_twist[name].linear.y, current_twist[name].linear.z])
#         position_error_in_world = target_position-current_position
#         current_orient = Quaternion(current_pose[name].orientation.w,current_pose[name].orientation.x,current_pose[name].orientation.y,current_pose[name].orientation.z)
#         position_error_in_body = (current_orient.inverse).rotate(position_error_in_world)

#         max_error_mag = 10.0
#         if (distance_to_target > max_error_mag):
#             position_error_unit_vector = position_error_in_body / distance_to_target
#             position_error_in_body = max_error_mag*position_error_unit_vector

#         if (target_heading_not_set): 
#             # target_heading = atan2(position_error_in_world[1],position_error_in_world[0])
#             target_orient = Quaternion(axis=(0.0, 0.0, 1.0), radians=atan2(position_error_in_world[1],position_error_in_world[0]))
#             target_heading_not_set = False
#         # target_orient = Quaternion(axis=(0.0, 0.0, 1.0), radians=atan2(position_error_in_world[1],position_error_in_world[0]))

#         orient_error_quat = target_orient * (current_orient.conjugate)
#         orient_error = orient_error_quat[3]
#         orient_error = clamp(orient_error,-0.5,0.5)

#         if (distance_to_target<tolerance+0.1):
#             break

#         vel.linear.x = 0.5*position_error_in_body[0] - 0.5*current_velocity[0]
#         vel.linear.y = 0.5*position_error_in_body[1] - 0.5*current_velocity[1]
#         vel.linear.z = 1.0*position_error_in_body[2] - 0.5*current_velocity[2]
#         vel.angular.x, vel.angular.y, vel.angular.z = 0.0, 0.0, -2.0*orient_error + 0.4*current_twist[name].angular.z

#         try:
#             pub.publish(vel)
#             time.sleep(0.05)
#         except Exception as e:
#             return False, f"Failed to publish {vel} to /{name}/cmd_vel: {e}"
        
#     return True, f"{name} has moved to ({current_pose[name].position.x}, {current_pose[name].position.y}, {current_pose[name].position.z})."

# @tool
# def stop_robot(name: str):
#     """
#     Stop the robot by publishing a Twist message with zero linear and angular velocities.

#     :param name: name of the robot
#     """
#     return publish_twist_to_cmd_vel.invoke(
#         {
#             "name": name,
#             "longitudinal": 0.0,
#             "lateral": 0.0,
#             "vertical": 0.0,
#             "yaw_rate": 0.0,
#             "steps": 1,
#         }
#     )

@tool
def has_moved_to_expected_coordinates(
    name: str, expected_x: float, expected_y: float, tolerance: float = 0.5
) -> str:
    """
    Check if the robot has moved to the expected position.

    :param name: name of the robot
    :param expected_x: expected x-coordinate
    :param expected_y: expected y-coordinate
    :param tolerance: tolerance level for the comparison
    """
    current_pose = get_robot_pose.invoke({"names": [name]})
    current_x = current_pose[name].position.x
    current_y = current_pose[name].position.y

    distance = ((current_x - expected_x) ** 2 + (current_y - expected_y) ** 2) ** 0.5
    if distance <= tolerance+0.1:
        return (
            f"{name} has moved to the expected position ({expected_x}, {expected_y})."
        )
    else:
        return f"{name} has NOT moved to the expected position ({expected_x}, {expected_y})."
