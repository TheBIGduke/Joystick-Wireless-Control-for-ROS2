from pydantic import BaseModel, Field
from delivery_bridge.webapp.settings import FunctionMode


class FunctionModeRequestForm(BaseModel):
    mode: FunctionMode = Field(
        description="Modo de funcionalidad",
        examples=["static", "delivery", "cruiser"],
    )
    # map_id: int | None = Field(
    #     default=None,
    #     description="ID del mapa",
    #     examples=[1, 2, 3],
    # )

