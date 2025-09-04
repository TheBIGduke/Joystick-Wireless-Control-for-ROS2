#!/usr/bin/env python
from typing import Union

# Third apps
from fastapi import APIRouter, status
from fastapi.requests import Request

# Local apps
from delivery_bridge.webapp.apps.base.serializers import SimpleResponse, ErrorResponse
from delivery_bridge.webapp.apps.ros2_app.forms.set_pose_forms import (
    SetPoseRequestForm,
    SetPoseToWaypointRequestForm,
)
from delivery_bridge.webapp.apps.waypoints.cruds.waypoint_cruds import (
    waypoint_crud,
)
from delivery_bridge.webapp.apps.base.errors import ERRORS
from delivery_bridge.base_node import base_node
# from delivery_bridge.webapp.settings import OperationMode
# from delivery_bridge.modules.mode_manager import mode_manager

router = APIRouter()


@router.post(
    "/set_pose/free",
    response_model=Union[SimpleResponse, ErrorResponse],
    status_code=status.HTTP_200_OK,
    summary="Publish an initial pose",
)
def set_pose(request: Request, form: SetPoseRequestForm):
    """
    Set robot pose

    Set the robot pose in the map. The position is given in meters and the
    orientation in radians.
    """
    # if not mode_manager.mode_ready or mode_manager.mode not in (
    #     OperationMode.STATIC,
    #     OperationMode.WAITRESS,
    #     OperationMode.NAVIGATION,
    # ):
    #     return ErrorResponse(
    #         status="FAIL",
    #         message="No se puede setear la posición en el estado actual",
    #         error=ERRORS.NO_AVAILABLE_IN_MODE.value,
    #     )

    status, response = base_node.pose_publisher.set_pose(
        form.position_x, form.position_y, form.orientation
    )

    if status:
        return SimpleResponse(status="OK", message=response)
    else:
        return ErrorResponse(status="FAIL", message=response, error="") #--------


@router.post(
    "/set_pose/{waypoint_id_name}",
    response_model=Union[SimpleResponse, ErrorResponse],
    status_code=status.HTTP_200_OK,
    summary="Set WP by either id or name",
)
def set_pose_to_waypoint(request: Request, form: SetPoseToWaypointRequestForm):
    """
    Set robot pose with a waypoint either id OR name

    Set the robot pose in the map with a waypoint.
    """
    # check if waypoint exists (first by id) -----
    if form.waypoint_id is not None:
        waypoint = waypoint_crud.get(form.waypoint_id)  
    else:
        waypoint = waypoint_crud.get_by_field("name",form.waypoint_name)    
    
    if waypoint is None:
        return ErrorResponse(
            status="FAIL",
            message="Waypoint does NOT exist",
            error=ERRORS.WAYPOINT_DOES_NOT_EXIST.value,
        )

    status, response = base_node.pose_publisher.set_pose_to_waypoint(waypoint)

    if status:
        return SimpleResponse(status="OK", message=response)
    else:
        return ErrorResponse(status="FAIL", message=response, error="Algo pasó al tratar de publicar initial pose")
