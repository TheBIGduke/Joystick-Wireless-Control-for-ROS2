from pydantic import BaseModel, Field


class SetPoseRequestForm(BaseModel):
    position_x: float = Field(
        description="Posición en x, en metros",
        examples=[0.0, 1.0, 2.0],
    )
    position_y: float = Field(
        description="Posición en y, en metros",
        examples=[0.0, 1.0, 2.0],
    )
    orientation: float = Field(
        description="Orientación, en radianes",
        examples=[0.0, 1.0, 2.0],
    )


class SetPoseToWaypointRequestForm(BaseModel):
    waypoint_id: int | None = Field(default=None,
        description="ID del waypoint",
        examples=[1, 2, 3],
    )
    waypoint_name: str | None = Field(default=None,
        max_length=50,
        description="Nombre del waypoint",
        examples=["mesa 1","punto1"],
    )
