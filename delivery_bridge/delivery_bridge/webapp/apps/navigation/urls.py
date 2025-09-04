# python imports


# 3rd party imports
from fastapi import APIRouter

from delivery_bridge.webapp.apps.navigation.routers import (
    navigation_routers,
)

router = APIRouter()

router.include_router(navigation_routers.router)
