# python imports
from datetime import datetime

# 3rd party imports
from pydantic import Field

# Local apps
# from delivery_bridge.webapp.apps.waypoints.serializers.waypoint_serializers import (
#     WaypointSimplestSerializer,
# )
# from delivery_bridge.webapp.apps.paths.serializers.path_serializers import (
#     PathSimplestSerializer,
# )
from delivery_bridge.webapp.apps.base.serializers import DataResponse, BaseSerializer
from delivery_bridge.modules.navigation_manager_models import NavigationState, WpOption


class NavigationStatusSerializer(BaseSerializer):
    on_navigation: bool = Field(
        description=(
            "'True' si se encuentra llendo a un WP, de lo contrario"
            " es 'False'"
        )
    )
    paused_navigation: bool = Field(
        description="'True' si la navegación está pausada, de lo contrario es 'False'"
    )
    #path: PathSimplestSerializer | None = Field(description="trayectoria actual")
    # option: WpOption = Field(
    #     description="Opción del WP objetivo",
    # )
    # laps: int | None = Field(
    #     default=None,
    #     description="Número de repeticiones de la trayectoria, '-1' si es en bucle",
    # )
    # lap: int = Field(description="Indica la vuelta actual")
    # on_waypoint: bool = Field(
    #     description=(
    #         "'True' si actualmente se encuentra ubicado en un punto de la "
    #         "trayectoria, de lo contrario es 'False'"
    #     )
    # )
    # next_waypoint: WaypointSimplestSerializer | None = Field(
    #     description="Siguiente punto de la trayectoria"
    # )
    # attempt: int = Field(description="Número de intentos en el punto actual")
    # start_time: datetime = Field(description="Hora de inicio del punto actual")
    state: NavigationState = Field(
        description="Estado de la meta actual",
    )
    message: str = Field(
        description="Mensaje de estado de la meta actual",
    )


class NavigationResponseSerializer(DataResponse):
    data: NavigationStatusSerializer
