# python imports
from typing import Union

# 3rd party imports
from fastapi import APIRouter, status

# local imports
from delivery_bridge.webapp.apps.base.serializers import ErrorResponse
from delivery_bridge.webapp.apps.ros2_app.serializers.battery_serializers import (
    BatterySerializer,
    BatteryResponseSerializer,
)
from delivery_bridge.webapp.apps.base.errors import ERRORS
from delivery_bridge.base_node import base_node

router = APIRouter()


@router.get(
    "/battery",
    response_model=Union[BatteryResponseSerializer, ErrorResponse],
    status_code=status.HTTP_200_OK,
    summary="Get battery status",
)
def get_battery():
    """
    Detalle de la bateria

    Este endpoint retorna el detalle de la bateria en formato JSON
    """
    if base_node.battery_subscriber.battery_available:
        return BatteryResponseSerializer(
            status="OK",
            message="Battery received",
            data=BatterySerializer(
                power_supply_status="-",
                voltage=base_node.battery_subscriber.battery.voltage,
                percentage=base_node.battery_subscriber.battery.percentage,
            ),
        )
    else:
        return ErrorResponse(
            status="ERROR",
            message="No battery data available",
            error=ERRORS.NO_BATTERY_DATA.value,
        )
