# python imports
from typing import Optional

# 3rd party imports
from sqlmodel import Field

from delivery_bridge.webapp.apps.base.models import BaseModel


class Waypoint(BaseModel, table=True):
    """Model definition for Waypoint"""

    name: str = Field(max_length=50)
    is_mandatory: bool = Field(default=True)
    description: Optional[str] = Field(default="")
    # map_id: int = Field(foreign_key="map.id")
    position_x: Optional[float] = Field(default=0.0)
    position_y: Optional[float] = Field(default=0.0)
    orientation: Optional[float] = Field(default=0.0)
