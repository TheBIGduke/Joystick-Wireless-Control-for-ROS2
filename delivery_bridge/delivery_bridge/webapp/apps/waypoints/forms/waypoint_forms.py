from pydantic import BaseModel, Field


class WaypointCreateForm(BaseModel):
    name: str = Field(
        description="Nombre del waypoint",
        max_length=50,
        unique=True,
        examples=["waypoint 1", "waypoint 2"],
    )
    is_mandatory: bool | None = Field(
        default=True,
        description="Waypoint obligatorio",
        examples=[True, False],
    )
    description: str | None = Field(
        default="",
        description="Descripción del waypoint",
        examples=["waypoint1", "waypoint2"],
    )

class WaypointCreateFormManual(BaseModel):
    name: str | None = Field(
        description="Nombre del waypoint",
        max_length=50,
        unique=True,
        examples=["waypoint 1", "waypoint 2"],
    )
    is_mandatory: bool | None = Field(
        default=True,
        description="Waypoint obligatorio",
        examples=[True, False],
    )
    description: str | None = Field(
        default=None,
        description="Descripción del waypoint",
        examples=["waypoint1", "waypoint2"],
    )
    position_x: float = Field(
        default=0.0,
        description="Posición en el eje X (en metros)",
        examples=[0.1, -3.78],
    )
    position_y: float = Field(
        default=0.0,
        description="Posición en el eje Y (en metros)",
        examples=[0.1, -3.78],
    )
    orientation: float = Field(
        gt=-3.1416,
        lt=3.1416,
        default=0.0,
        description="Orientación con respecto al eje X (en radianes)",
        examples=[0.1, -3.78],
    )

class WaypointUpdateForm(BaseModel):
    name: str | None = Field(
        default=None,
        description="Nombre del waypoint",
        max_length=50,
        unique=True,
        examples=["waypoint 1", "waypoint 2"],
    )
    is_mandatory: bool | None = Field(
        default=True,
        description="Waypoint obligatorio",
        examples=[True, False],
    )
    description: str | None = Field(
        default=None,
        description="Descripción del waypoint",
        examples=["waypoint1", "waypoint2"],
    )
    position_x: float | None = Field(
        default=None,
        description="Posición en el eje X (en metros)",
        examples=[0.1, -3.78],
    )
    position_y: float | None = Field(
        default=None,
        description="Posición en el eje Y (en metros)",
        examples=[0.1, -3.78],
    )
    orientation: float | None = Field(
        gt=-3.1416,
        lt=3.1416,
        default=None,
        description="Orientación con respecto al eje X (en radianes)",
        examples=[0.1, -3.78],
    )

