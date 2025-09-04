from pydantic import BaseModel, Field

from delivery_bridge.webapp.apps.navigation.serializers.navigation_serializers import (
    WpOption,
)

class NavRequestFormGoal(BaseModel):
    waypoint_id: int | None = Field(
        # default=None,
        description="ID de un punto de interés para navegar a él (delivery)",
    )

class NavRequestFormOp(BaseModel):
    option: WpOption | None = Field(
        description="Opción del Wp objetivo",
    )

"""
class NavigationRequestForm(BaseModel):
    option: WpOption = Field(
        description="Modo de la trayectoria",
    )
    # path_id: int | None = Field(
    #     default=None,
    #     description="ID de la trayectoria",
    # )
    waypoint_id: int | None = Field(
        default=None,
        description="ID de un punto de interes para navegar a el (delivery)",
    )
    # laps: int | None = Field(
    #     default=None,
    #     description="Valor: repeticiones de la trayectoria, -1 si bucle infinito",
    # )
"""