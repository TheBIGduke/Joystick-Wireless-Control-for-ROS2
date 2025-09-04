#!/usr/bin/env python
import time
import threading
import math
import subprocess

from delivery_bridge.webapp.settings import settings

# Models
from delivery_bridge.webapp.apps.waypoints.cruds.waypoint_cruds import waypoint_crud
from delivery_bridge.webapp.apps.waypoints.models import Waypoint

# Serializers
from delivery_bridge.modules.navigation_manager_models import NavigationState
from delivery_bridge.webapp.apps.navigation.serializers.navigation_serializers import (
    NavigationStatusSerializer,
)

# Local apps
from delivery_bridge.clients.navigation_client import NavigationClient
from delivery_bridge.utils import euler_from_quaternion
from delivery_bridge.webapp.socket_io import emitEvent
from delivery_bridge.base_node import base_node


class NavigationManager:
    def __init__(self):
        self.client: NavigationClient = base_node.navigation_client
        self.client.feedback_callback = self.navigation_feedback_callback

        # Clear costmap service from terminal
        self.clear_map_cmd = ' '.join(['ros2','service','call','/local_costmap/clear_entirely_local_costmap','nav2_msgs/srv/ClearEntireCostmap','request:\\','{}'])

        self.option = "stop"
        self.mode_ready = True

        self.on_navigation = False
        self.paused_navigation = False
        self.state = NavigationState.READY
        self.message = ""
        self.on_waypoint = False

        self.supervisor_thread = None
        self.near_to_waypoint = False
        # self.timeout_reached = False
        self.cant_reach_wp = False

    def cancel_navigation(self):
        if self.on_navigation:
            self.client.cancel_all_goals()
            self.on_navigation = False
            self.paused_navigation = False
            return True
        else:
            return False

    def start_navigation( self, waypoint_id: int): #, option: WpOption ):
        if self.on_navigation:
            return False, "Navigation is already on"

        if waypoint_id is not None and waypoint_id > 0:
            self.delivery_waypoint: Waypoint = waypoint_crud.get(waypoint_id)
            if self.delivery_waypoint is None:
                return False, "Waypoint not found"
        else:
            return False, "Waypoint is required"

        # self.next_stop_waypoint_attempts = 0
        self.near_to_waypoint = False

        myThread = threading.Thread(target=self.loop_navigation, args=[self.delivery_waypoint])
        myThread.setDaemon(True)
        myThread.start()

        self.supervisor_thread = threading.Thread(target=self.supervisor)
        self.supervisor_thread.setDaemon(True)
        self.supervisor_thread.start()

        base_node.logger.info(
            f"*Navigation*Navigation started for wp: {self.delivery_waypoint.name}"
        )

        return True, "Navigation started"

    def pause_navigation(self):
        if self.on_navigation:
            self.client.cancel_goal()
            # self.client.cancel_all_goals()
            self.paused_navigation = True
            return True
        else:
            return False

    def resume_navigation(self):
        if self.on_navigation:
            self.paused_navigation = False
            # self.next_stop_waypoint_attempts = 0
            return True
        else:
            return False

    def cancel_goal(self):
        if self.on_navigation:
            base_node.logger.info("cancel goal")
            self.client.cancel_goal()
            return True
        else:
            return False
    
    def supervisor(self):
        base_node.logger.info("Start navigator supervisor")

        while self.on_navigation:
            time.sleep(0.05)
            # continue if navigation is not active
            if self.state != NavigationState.ACTIVE:
                continue

            # get current position
            current_x = base_node.pose_subscriber.pose_data.position_x
            current_y = base_node.pose_subscriber.pose_data.position_y

            # get waypoint position
            waypoint_x = self.delivery_waypoint.position_x
            waypoint_y = self.delivery_waypoint.position_y
            waypoint_name = self.delivery_waypoint.name

            # get distance between current and waypoint
            euclidian_distance = math.sqrt(
                (current_x - waypoint_x) ** 2 + (current_y - waypoint_y) ** 2
            )
            if euclidian_distance < settings.NAVIGATION_MANAGER.DISTANCE_ERROR:
                # base_node.logger.warning(
                #     f"*Navigation*Near to waypoint {waypoint_name}"
                # )
                # self.client.cancel_goal()
                self.near_to_waypoint = True

            frontal_distance = base_node.frontal_free_subscriber.distance
            # base_node.logger.info(f"*** dist: {frontal_distance}")

            linear_vel_nav = base_node.cmd_vel_nav_subscriber.linear_x
            angular_vel_nav = base_node.cmd_vel_nav_subscriber.angular_z
            # base_node.logger.info("v: {:2.4f}, w: {:2.4f}".format(linear_vel_nav, angular_vel_nav))
            
            if (
                frontal_distance < settings.FRONTALFREE.DISTANCE_SAFE #and frontal_distance > 0.1 
                and not self.paused_navigation
                and abs(angular_vel_nav) < 0.07 and linear_vel_nav > 0.05 # moving in straight line
            ):
                self.pause_navigation()
                base_node.logger.info(f"---- frontal_free_lane, robot stopped")
                self.state = NavigationState.PAUSED
                self.message = "Something on the lane"
                self.send_status_event()

                # ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap request:\ {}

        base_node.logger.info("End navigator supervisor")
    

    def on_fail_reach_wp(self):
        pass

    def navigation_feedback_callback(self, feedback_msg):
        # calculate every second
        if (time.time() - self.client.last_t0) > 1.0:
            # for current pose
            current_pose = feedback_msg.feedback.current_pose.pose
            current_position_x = current_pose.position.x
            current_position_y = current_pose.position.y
            current_orientation_z = euler_from_quaternion(
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z,
                current_pose.orientation.w,
            )[2]

            euclidean_distance = math.sqrt(
                (current_position_x - self.client.goal_position_x) ** 2
                + (current_position_y - self.client.goal_position_y) ** 2
            )
            doz = abs(current_orientation_z - self.client.goal_orientation)

            eta = (
                feedback_msg.feedback.estimated_time_remaining.sec
                + feedback_msg.feedback.estimated_time_remaining.nanosec / 1e9
            )
            dr = feedback_msg.feedback.distance_remaining
            tt = (
                feedback_msg.feedback.navigation_time.sec
                + feedback_msg.feedback.navigation_time.nanosec / 1e9
            )
            r = feedback_msg.feedback.number_of_recoveries
            delta_dr = (self.client.last_dr - dr) / (self.client.last_t0 - time.time())
            delta_doz = abs(self.client.last_doz - doz)
            self.client.last_dr = dr
            self.client.last_doz = doz
            self.client.last_t0 = time.time()

            if delta_dr > -0.01 and delta_doz < 0.1:
                self.client.count_lost += 1
            else:
                self.client.count_lost = 0

            # ff_dist = base_node.frontal_free_subscriber.distance
            # base_node.logger.info(f"-------------- ff dist: {ff_dist} -----------------------")

            # base_node.logger.info(
            #     f"NavCFb ETA:{eta:2.1f} s, "
            #     f"Dr:{dr:2.2f} m, "
            #     f"Deu:{euclidean_distance:2.2f} m, "
            #     f"doz: {doz:2.1f} rad, "
            #     f"Tt:{tt:2.1f} s, "
            #     f"R:{r},\t "
            #     f"V:{delta_dr:2.2f} m/s, "
            #     f"d_doz:{delta_doz:2.1f} rad/s, "
            #     f"cl:{self.client.count_lost}"
            # )

    def loop_navigation(self, delivery_waypoint):
        # set initial state
        self.on_navigation = True
        self.on_waypoint = False
        self.client.wait_until_ready()

        # start navigation loop
        while self.on_navigation:
            # send goal if navigation is not paused
            if not self.paused_navigation:
                base_node.logger.info("----------------------------------------")
                base_node.logger.info("for waypoint: {}".format(str(delivery_waypoint.name)))

                base_node.logger.info(f"{self.clear_map_cmd}")
                
                p = subprocess.Popen(self.clear_map_cmd, stderr=subprocess.PIPE, stdout=subprocess.PIPE, shell=True)
                stdoutdata, stderrdata = p.communicate()  #this is blocking
                base_node.logger.info(f"{stdoutdata.decode()}---------")

                # send the goal and wait
                self.state = self.send_waypoint_and_wait(delivery_waypoint)

                self.message = self.state.description
                base_node.logger.info(
                    f""" End wait goal
                                state:   {self.state.name}
                                message: {self.message}
                            """
                )

            # check if navigation is paused
            if self.paused_navigation:
                base_node.logger.info("Navigation paused")
                self.state = NavigationState.PAUSED
                self.message = "Navigation paused"
                self.send_status_event()
                base_node.logger.info("*Navigation*Navigation paused")
                # wait for resume navigation
                while self.paused_navigation and self.on_navigation:
                    time.sleep(settings.NAVIGATION_MANAGER.PAUSE_TIME)
                    base_node.logger.info(f"*Paused timeout exceed!")

                    frontal_distance = base_node.frontal_free_subscriber.distance
                    # # base_node.logger.info(f"d: {frontal_distance}")

                    if ( frontal_distance >= settings.FRONTALFREE.DISTANCE_SAFE ):
                        self.resume_navigation()
                        base_node.logger.info(f"----- frontal_free_lane, robot released")
                        
                        self.state = NavigationState.ACTIVE
                        self.message = "Lane released"
                        self.send_status_event()

                        p = subprocess.Popen(self.clear_map_cmd, stderr=subprocess.PIPE, stdout=subprocess.PIPE, shell=True)
                        stdoutdata, stderrdata = p.communicate()  #this is blocking
                    else:
                        base_node.logger.info(f"----- frontal_free_lane, robot stopped")
                
                base_node.logger.info("Navigation resumed (or cancelled)")
                self.state = NavigationState.ACTIVE
                self.message = "Navigation resumed (or cancelled)"
                self.send_status_event()
                base_node.logger.info(
                    "*Navigation*Navigation resumed (or cancelled)"
                )
                continue
            # check if navigation was cancelled
            elif not self.on_navigation:
                base_node.logger.info("Navigation was cancelled")
                base_node.logger.info("*Navigation*Navigation was cancelled")
                self.state = NavigationState.CANCELED
                self.message = "Navigation Canceled"
                break

            elif self.state != NavigationState.SUCCEEDED:
                base_node.logger.warning(f"Navigation failed: {self.state}")
                base_node.logger.warning(
                    f"*Navigation*Navigation failed: {self.state}"
                )
                pass
            # check if navigation reached waypoint
            elif (
                self.state == NavigationState.SUCCEEDED #or self.near_to_waypoint
            ):
                base_node.logger.info("Navigation reached waypoint")
                self.on_waypoint = True
                # publish event on websocket to notify
                self.send_status_event()
                self.on_waypoint = False
                self.message = "Navigation finished (WP reached)"
                # break loop for receive pause/resume navigation
                break
            
            # if navigation is canceled or failed, stop waypoint loop
            if not self.on_navigation:
                break

            

        # stop supervisor thread
        try:
            self.supervisor_thread.stop()
            base_node.logger.info("stop navigation supervisor")
        except Exception:
            base_node.logger.warning("navigation supervisor was not running")
        
        self.on_navigation = False

        # Stop robot after a path finished
        base_node.cmd_vel_publisher.publish(0.0, 0.0)

        self.send_status_event() # publish event on websocket to notify

    
    def send_waypoint_and_wait(self, waypoint: Waypoint):
        self.timeout_reached = False
        self.cant_reach_wp = False
        self.client.count_lost = 0
        self.client.send_goal( waypoint.position_x, waypoint.position_y, waypoint.orientation )

        self.state = NavigationState.ACTIVE
        self.message = "On route"

        # publish event on websocket to notify
        self.send_status_event()

        # wait for goal response
        return self.client.wait_until_task_complete()

    def get_navigation_status(self) -> NavigationStatusSerializer:
        # waypoint = None
        # if self.next_stop_waypoint:
        #     waypoint = waypoint_crud.get(self.next_stop_waypoint.waypoint)
        return NavigationStatusSerializer(
            on_navigation=self.on_navigation,
            paused_navigation=self.paused_navigation,
            # path=PathSimplestSerializer(self.path) if self.path else None,
            # option=self.mode,
            # laps=self.laps,
            # lap=self.lap,
            on_waypoint=self.on_waypoint,
            # next_waypoint=WaypointSimplestSerializer(waypoint) if waypoint else None,
            # attempt=self.next_stop_waypoint_attempts,
            # start_time=self.next_stop_waypoint_start_time,
            state=self.state,
            message=self.message,
        )

    def send_status_event(self):
        emitEvent(
            "on_status_change",
            {"data": {"navigation": self.get_navigation_status().to_dict()}},
        )


navigation_manager = NavigationManager()
