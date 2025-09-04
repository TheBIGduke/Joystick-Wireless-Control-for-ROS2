# 3rd party imports
from fastapi import APIRouter

# local imports
# Removed home_router and user_routers - no web interface needed
from delivery_bridge.webapp.apps.waypoints.urls import router as waypoints_router
from delivery_bridge.webapp.apps.ros2_app.urls import router as ros2_router
from delivery_bridge.webapp.apps.navigation.urls import router as navigation_router

router = APIRouter()

# Only include API routers needed for ESP32 communication
router.include_router(waypoints_router, prefix="/waypoints")
router.include_router(ros2_router, prefix="/ros", tags=["ROS2"])
router.include_router(navigation_router, prefix="/navigation", tags=["Navigation"])
