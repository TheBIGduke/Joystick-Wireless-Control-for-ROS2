# python imports

# 3rd party imports
from fastapi import FastAPI

from fastapi.middleware.cors import CORSMiddleware
# Removed RedirectResponse import - not needed without web interface
# Removed Request import - not needed without authentication
from fastapi.openapi.docs import get_swagger_ui_html, get_redoc_html
from fastapi.openapi.utils import get_openapi
# Removed Session, select imports - no user authentication needed

import socketio

# Import the models to create the tables, don't remove this import even if it's not used
from .apps.waypoints.models import Waypoint  # noqa: F401
# Removed User model import - no user authentication needed for ESP32

from .database import engine
# Removed NotAuthenticatedException import - no authentication needed
from .urls import router
from .socket_io import sio
from .ws_no_prefix import NoPrefixNamespace


app = FastAPI()

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

########################################################################################
# API docs - removed authentication requirement for ESP32 access

@app.get("/docs", tags=["docs"], include_in_schema=False)
async def get_docs():
    return get_swagger_ui_html(
        openapi_url="/openapi.json",
        title="Delivery Bridge API",
        swagger_favicon_url="/favicon.ico",
    )


@app.get("/redoc", tags=["docs"], include_in_schema=False)
async def get_redoc():
    return get_redoc_html(
        openapi_url="/openapi.json",
        title="Delivery Bridge API",
        redoc_favicon_url="/favicon.ico",
    )


@app.get("/openapi.json", tags=["docs"], include_in_schema=False)
async def openapi():
    return get_openapi(
        title="Delivery Bridge API",
        description=(
            "API para el Delivery Bridge, usado para la administración de la"
            " navegación autónoma, interactuando con ROS 2 via ESP32."
        ),
        version="1.0.0",
        routes=app.routes,
    )


sio.register_namespace(NoPrefixNamespace("/"))
sio_asgi_app = socketio.ASGIApp(socketio_server=sio, other_asgi_app=app)


# Removed login manager and static file mounting - not needed for ESP32 API access

# include routers
app.include_router(router)

app.add_route("/socket.io/", route=sio_asgi_app, methods=["GET", "POST"])
app.add_websocket_route("/socket.io/", sio_asgi_app)

########################################################################################


# Removed authentication exception handler - not needed for ESP32 API access


########################################################################################
