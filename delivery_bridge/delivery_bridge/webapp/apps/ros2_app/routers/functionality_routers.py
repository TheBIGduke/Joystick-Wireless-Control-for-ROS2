#!/usr/bin/env python
from typing import Union

# Third apps
from fastapi import APIRouter, status

# Local apps
from delivery_bridge.webapp.apps.base.serializers import SimpleResponse, ErrorResponse
from delivery_bridge.webapp.apps.ros2_app.forms.functionality_mode_forms import (
    FunctionModeRequestForm,
)
from delivery_bridge.webapp.apps.ros2_app.serializers.functionality_mode_serializers import (
    FunctionModeSerializer,
    FunctionModeResponseSerializer,
)

from delivery_bridge.modules.function_manager import function_manager

router = APIRouter()


@router.get(
    "/functionality_mode",
    response_model=FunctionModeResponseSerializer,
    status_code=status.HTTP_200_OK,
    summary="Get Current Functionality Mode",
)
def get_functionality_mode():
    """
    Get current functionality mode.
    """
    return FunctionModeResponseSerializer(
        status="OK",
        message="Modo de funcionalidad actual",
        data=FunctionModeSerializer(
            mode=function_manager.function,
            ready=function_manager.function_ready,
        ),
    )


@router.post(
    "/functionality_mode",
    response_model=Union[SimpleResponse, ErrorResponse],
    status_code=status.HTTP_200_OK,
    summary="Set Functionality Mode"
)
def set_functionality_mode(form_data: FunctionModeRequestForm):
    """
    Set functionality mode

    The functionality can be: static, delivery, cruiser.
    """

    status, response = function_manager.set_mode(
        form_data.mode,
    )

    if status:
        return SimpleResponse(status="OK", message=response)
    else:
        return ErrorResponse(
            status="ERROR",
            message="Error al solicitar el modo de funcionalidad",
            error=response,
        )
