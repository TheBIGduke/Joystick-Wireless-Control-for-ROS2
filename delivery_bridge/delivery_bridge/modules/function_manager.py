#!/usr/bin/env python
import copy
import subprocess
from subprocess import Popen
import time
import psutil
import threading
from os import environ #, path

# Third apps

# Local apps
from delivery_bridge.webapp.socket_io import emitEvent
from delivery_bridge.webapp.settings import FunctionMode
# from delivery_bridge.modules.event_handlers import (
#     on_mode_notification,
# )
from delivery_bridge.webapp.settings import settings #, APP_DATA_DIR
from delivery_bridge.topics.pose_subscriber import RobotPoseData
from delivery_bridge.base_node import base_node
from delivery_bridge.modules.navigation_manager import navigation_manager


class FunctionManager(threading.Thread):
    SET_SAME_POSE = "same"
    SET_LAST_MODE_POSE = "last_mode"
    NO_SET_POSE = ""

    def __init__(self):
        threading.Thread.__init__(self)

        self.desired_function: FunctionMode = FunctionMode.STOP
        self.function: FunctionMode = FunctionMode.STOP
        self.function_ready = True
        self.processes = []

        self.last_mode_pose: RobotPoseData = None
        self.currently_changing_funct = False

        self.setDaemon(True)

    def set_mode(
        self,
        mode: FunctionMode,
    ):
        # if desired mode already is current mode, do nothing
        if self.function == mode and self.function_ready:
            # publish event with mode status
            self.send_status_event()
            return True, "Mode already is {}".format(mode)

        base_node.logger.info((f"Change mode from '{self.function}' to '{mode}'"))

        if not self.function_ready and self.currently_changing_funct:
            return (
                False,
                "FunctionManager is not ready to change functionality, or is already changing",
            )

        self.desired_function = mode
        self.function_ready = False

        return True, "changing functionality to {}".format(mode)

    def kill_process(self, popen_instance: Popen):
        try:
            process = psutil.Process(popen_instance.pid)
            for proc in process.children(recursive=True):
                proc.kill()
            process.kill()
        except Exception as e:
            base_node.logger.error(f"Error killing process: {e}")

    def supervise_process(self, popen_instance: Popen):
        base_node.logger.info(
            "Start supervise process ------------------------------------------------"
        )
        while popen_instance.returncode is None:
            # handle output by direct access to stdout and stderr
            for line in popen_instance.stdout:
                emitEvent(
                    "on_process_output",
                    {"data": {"output": line.decode("utf-8").strip()}},
                )
            # set returncode if the process has exited
            popen_instance.poll()
        base_node.logger.info(
            "End supervise process ------------------------------------------------"
        )

    def processes_add_command(self, env: dict, command: str):
        env = {**environ, **env}
        process = Popen(
            command.split(), env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT
        )
        supervisor_thread = threading.Thread(
            target=self.supervise_process, args=(process,)
        )
        supervisor_thread.daemon = True
        supervisor_thread.start()
        self.processes.append(
            {"process": process, "supervisor_thread": supervisor_thread}
        )

    def processes_stop(self):
        if len(self.processes) > 0:
            base_node.logger.info("Stopping processes ...")
            for process_item in self.processes:
                self.kill_process(process_item["process"])
            for process_item in self.processes:
                process_item["process"].wait()
            base_node.logger.info("                   ... Processes stopped")
            self.processes = []
        else:
            base_node.logger.info("No processes to stop")

    def run(self):
        current_pose: RobotPoseData = None

        self.desired_function = FunctionMode.STATIC
        self.function_ready = False
        base_node.logger.info("FunctionManager thread to manage functionalities started")
        while True:
            # if no change in functionality, do nothing
            if self.function == self.desired_function:
                time.sleep(1)
                continue

            self.currently_changing_funct = True

            # cancel navigation and stop robot
            navigation_manager.cancel_navigation() #---------------------------
            base_node.cmd_vel_publisher.publish(0.0, 0.0)

            # get last pose
            if base_node.pose_subscriber.pose_available:
                self.last_mode_pose = copy.deepcopy(base_node.pose_subscriber.pose_data)
                base_node.logger.info(
                    f"""Last mode pose:
                        position_x: {self.last_mode_pose.position_x:0.2f},
                        position_y: {self.last_mode_pose.position_y:0.2f},
                        orientation: {self.last_mode_pose.orientation:0.2f}"""
                )
            # unsubscribe to pose topic
            # base_node.pose_subscriber.try_unsubscribe()

            # stop current functionality by processes if exist
            # self.processes_stop() ???????????????

            # Stop robot after stop processes
            base_node.cmd_vel_publisher.publish(0.0, 0.0)

            # clear map
            # base_node.map_subscriber.clear_map()

            success_function_startup = False
            # if mode is self.STOP_MODE, mode start success
            if self.desired_function == FunctionMode.STOP:
                success_function_startup = True

            # Start Attemps to change mode
            attempt_number = 0
            maximun_attempts = 3
            while (
                not success_function_startup
                and attempt_number < maximun_attempts
            ):
                attempt_number += 1
                base_node.logger.info(
                    f"---- Attempt to change functionality No. {attempt_number} "
                    f"from '{self.function}' to '{self.desired_function}' ----"
                )
                # stop processes
                self.processes_stop()

                # start desired functionality. start processes
                process_set = settings.FUNCTION_MANAGER.get_process(self.desired_function)
                for command in process_set.COMMANDS:
                    base_node.logger.info("Starting process: {}".format(command))
                    self.processes_add_command(process_set.ENVIRON_VARS, command)
                ##############################

                # base_node.map_subscriber.try_subscribe()
                # base_node.pose_publisher.set_pose(0.0, 0.0, 0.0)

                # wait to pose topic to confirm start mode ------------------------------
                # wait to map topic to confirm start mode -------------------------------


                # if reach this point, mode start success
                success_function_startup = True
                # End attemps to change mode

            # if cant change mode after many attempts ------------------------

            # if mode is stop, do nothing
            if self.desired_function == FunctionMode.STOP:
                base_node.logger.info("On 'self.STOP_MODE' mode")
                self.function = self.desired_function
                self.function_ready = True
                # publish event on websocket to notify
                self.send_status_event()
                self.currently_changing_funct = False
                continue

            base_node.logger.info("wait 2 seconds")
            time.sleep(2.0)

            # restore last pose
            if (
                settings.POSE_TO_SET == self.SET_LAST_MODE_POSE
                and self.last_mode_pose is not None
            ):
                base_node.logger.info(
                    f"""SET LAST MODE POSE with initial covariance:
                        position_x: {self.last_mode_pose.position_x:0.2f},
                        position_y: {self.last_mode_pose.position_y:0.2f},
                        orientation: {self.last_mode_pose.orientation:0.2f}"""
                )
                base_node.pose_publisher.set_pose(
                    self.last_mode_pose.position_x,
                    self.last_mode_pose.position_y,
                    self.last_mode_pose.orientation,
                )
            elif (
                settings.POSE_TO_SET == self.SET_SAME_POSE and current_pose is not None
            ):
                base_node.logger.info(
                    f"""SET SAME POSE with initial covariance:
                            position_x: {current_pose.position_x:0.2f},
                            position_y: {current_pose.position_y:0.2f},
                            orientation_z: {current_pose.orientation:0.2f}"""
                )
                base_node.pose_publisher.set_pose(
                    current_pose.position_x,
                    current_pose.position_y,
                    current_pose.orientation,
                )
            elif settings.POSE_TO_SET == self.NO_SET_POSE:
                base_node.logger.warning("NO SET POSE")
            else:
                base_node.logger.warning(
                    "No last pose, SET POSE TO ZERO with initial covariance"
                )
                base_node.pose_publisher.set_pose(0.0, 0.0, 0.0)

            self.function = self.desired_function
            self.function_ready = True
            # publish event on websocket to notify
            base_node.logger.info(f"*Modes*Mode changed to {self.function.value}")
            self.send_status_event()
            self.currently_changing_funct = False

    def send_status_event(self):
        emitEvent(
            "on_status_change",
            {
                "data": {
                    "functionality_mode": {
                        "function": self.function.value,
                        "ready": self.function_ready,
                    }
                }
            },
        )
        # base_node.logger.info("-- emitEvent, on_status_change -------")
        # base_node.mode_status_publisher.publish(self.function.value)


function_manager = FunctionManager()
