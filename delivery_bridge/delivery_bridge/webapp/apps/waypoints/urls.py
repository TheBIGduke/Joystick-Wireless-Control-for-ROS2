# python imports

# 3rd party imports
from fastapi import APIRouter

# local imports
from delivery_bridge.webapp.apps.waypoints.routers import (
    waypoint_routers,
)


router = APIRouter()

router.include_router(waypoint_routers.router, prefix="/waypoint", tags=["Waypoints"])
