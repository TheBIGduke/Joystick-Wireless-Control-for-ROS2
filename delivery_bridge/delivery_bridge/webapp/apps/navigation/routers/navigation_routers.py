#!/usr/bin/env python
from typing import Union

# Third apps
from fastapi import APIRouter, status

# Local apps
from delivery_bridge.webapp.apps.navigation.forms.navigation_forms import (
    NavRequestFormGoal, NavRequestFormOp
)
from delivery_bridge.webapp.apps.base.serializers import SimpleResponse, ErrorResponse
from delivery_bridge.webapp.apps.navigation.serializers.navigation_serializers import (
    NavigationResponseSerializer,
)
from delivery_bridge.webapp.apps.base.errors import ERRORS
from delivery_bridge.modules.navigation_manager import navigation_manager
from delivery_bridge.modules.navigation_manager_models import WpOption
from delivery_bridge.modules.function_manager import function_manager
from delivery_bridge.webapp.settings import FunctionMode

router = APIRouter()


@router.get(
    "/navigation",
    response_model=NavigationResponseSerializer,
    status_code=status.HTTP_200_OK,
    summary="Get Navigation Status",
)
def get_navigation():
    """
    Navigation status

    Get the current navigation status.
    """
    return NavigationResponseSerializer(
        status="OK",
        message="Current navigation status",
        data=navigation_manager.get_navigation_status(),
    )

@router.post(
    "/set_goal/{waypoint_id}",
    response_model=Union[SimpleResponse, ErrorResponse],
    status_code=status.HTTP_200_OK,
)
def set_nav2_goal(form_data: NavRequestFormGoal):
    """
    Set a Goal using Nav2 stack
    """
    if function_manager.function != FunctionMode.DELIVERY:
        return ErrorResponse(
            status="FAIL",
            message="Nothing to do, Robot is not in delivery functionality",
            error=ERRORS.NO_AVAILABLE_IN_MODE,
        )

    if not function_manager.function_ready:
        return ErrorResponse(
            status="FAIL",
            message="System error",
            error=ERRORS.MODE_NOT_READY,
        )
    
    status, response = navigation_manager.start_navigation(form_data.waypoint_id)

    if status:
        return SimpleResponse(
            status="OK",
            message="Navigation start set, going to Wp ...",
        )
    else:
        return ErrorResponse(
            status="FAIL",
            message="Navigation start fail",
            error=response,
        )


@router.post(
    "/set_wp/{wp_option}",
    response_model=Union[SimpleResponse, ErrorResponse],
    status_code=status.HTTP_200_OK,
    summary="Set the option for the current WP goal",
)
def set_wp_option(form_data: NavRequestFormOp):
    """
    Set the option for the current Wp. Stop, pause, resume or cancel the current WP.
    """

    # For stop or cancel
    if form_data.option == WpOption.STOP or form_data.option == WpOption.CANCEL:
        if navigation_manager.on_navigation:
            navigation_manager.cancel_navigation()
            return SimpleResponse(
                status="OK",
                message="Navigation manager stopped",
            )
        else:
            return SimpleResponse(
                status="OK",
                message="Navigation manager already stopped",
            )

    # For pause
    elif form_data.option == WpOption.PAUSE:
        if not navigation_manager.on_navigation:
            return SimpleResponse(
                status="OK",
                message="Navigation manager already stopped",
            )
        elif navigation_manager.paused_navigation:
            return SimpleResponse(
                status="OK",
                message="Navigation manager already paused",
            )
        else:
            navigation_manager.pause_navigation()
            return SimpleResponse(
                status="OK",
                message="Navigation manager paused",
            )

    # For resume
    elif form_data.option == WpOption.RESUME:
        if not navigation_manager.on_navigation:
            return SimpleResponse(
                status="OK",
                message="Navigation manager already stopped",
            )
        elif not navigation_manager.paused_navigation:
            return SimpleResponse(
                status="OK",
                message="Navigation manager already resumed",
            )
        else:
            navigation_manager.resume_navigation()
            return SimpleResponse(
                status="OK",
                message="Navigation manager resumed",
            )

    # elif navigation_manager.on_navigation:
    #     return ErrorResponse(
    #         status="FAIL",
    #         message="Currently on path",
    #         error=ERRORS.CURRENTLY_ON_PATH,
    #     )


