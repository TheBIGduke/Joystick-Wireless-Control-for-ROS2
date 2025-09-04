# python imports
from typing import Union

# 3rd party imports
from fastapi import APIRouter, status

# local imports
from delivery_bridge.webapp.apps.base.serializers import SimpleResponse
from delivery_bridge.webapp.apps.waypoints.cruds.waypoint_cruds import (
    waypoint_crud,
    validate_name_exists,
)
from delivery_bridge.webapp.apps.waypoints.forms.waypoint_forms import (
    WaypointCreateForm,
    WaypointUpdateForm,
    WaypointCreateFormManual
)
from delivery_bridge.webapp.apps.waypoints.models import Waypoint
from delivery_bridge.webapp.apps.waypoints.serializers.waypoint_serializers import (
    WaypointSerializer,
    WaypointListResponseSerializer,
    WaypointDetailResponseSerializer,
)
from delivery_bridge.webapp.apps.base.serializers import ErrorResponse
from delivery_bridge.webapp.apps.base.errors import ERRORS
from delivery_bridge.base_node import base_node
# from delivery_bridge.webapp.settings import OperationMode
# from delivery_bridge.modules.mode_manager import mode_manager

router = APIRouter()


@router.get(
    "", response_model=WaypointListResponseSerializer, status_code=status.HTTP_200_OK,
    summary="Waypoints List"
)
def list_waypoints():
    """
    Lista de waypoints

    Este endpoint retorna una lista de waypoints registrados, en formato JSON
    """
    waypoints = waypoint_crud.list()
    data = [WaypointSerializer(waypoint) for waypoint in waypoints]
    return WaypointListResponseSerializer(
        status="OK", message="Waypoint list", data=data
    )


@router.get(
    "/{waypoint_id}",
    response_model=Union[WaypointDetailResponseSerializer,ErrorResponse],
    status_code=status.HTTP_200_OK,
    summary="Show WP by ID",
)
def get_waypoint(waypoint_id: int):
    """
    Detalle del waypoint

    Este endpoint retorna el detalle de un waypoint, en formato JSON
    """
    waypoint = waypoint_crud.get(waypoint_id)
    print(waypoint)
    if waypoint is None:
        return ErrorResponse(
            status="FAIL",
            message="No existe wp con ese id",
            error=ERRORS.OBJECT_NOT_FOUND,
        )
    data = WaypointSerializer(waypoint)
    return WaypointDetailResponseSerializer(
        status="OK", message="Waypoint detail", data=data
    )


@router.post(
    "",
    response_model=Union[WaypointDetailResponseSerializer, ErrorResponse],
    status_code=status.HTTP_201_CREATED,
    summary="Create WP with automatically assignation of coordinates"
)
def create_waypoint(waypoint_form: WaypointCreateForm):
    """
    Crear waypoint con asignación automática de coordenadas

    Este endpoint permite crear una waypoint en formato JSON
    """
    # if validate_name_exists(waypoint_form.name, mode_manager.map_id):
    if validate_name_exists(waypoint_form.name):
        return ErrorResponse(
            status="FAIL", message="Waypoint already exists", error=ERRORS.NAME_UNIQUE
        )
    if not base_node.pose_subscriber.pose_available:
        return ErrorResponse(
            status="FAIL",
            message="No hay datos de posición disponibles",
            error=ERRORS.NO_POSE_AVAILABLE,
        )

    waypoint = Waypoint(
        name=waypoint_form.name,
        is_mandatory=waypoint_form.is_mandatory,
        description=waypoint_form.description,
        position_x=base_node.pose_subscriber.pose_data.position_x,
        position_y=base_node.pose_subscriber.pose_data.position_y,
        orientation=base_node.pose_subscriber.pose_data.orientation,
    )

    waypoint = waypoint_crud.create(waypoint)
    data = WaypointSerializer(waypoint)
    return WaypointDetailResponseSerializer(
        status="OK", message="Waypoint created", data=data
    )


@router.post(
    "/manual",
    response_model=Union[WaypointDetailResponseSerializer, ErrorResponse],
    status_code=status.HTTP_201_CREATED,
    summary="Create WP with manual assignation of coordinates"
)
def create_waypoint(waypoint_form: WaypointCreateFormManual):
    """
    Crear waypoint con asignación manual de coordenadas

    Este endpoint permite crear una waypoint en formato JSON
    """
    # if validate_name_exists(waypoint_form.name, mode_manager.map_id):
    if validate_name_exists(waypoint_form.name):
        return ErrorResponse(
            status="FAIL", message="Waypoint already exists", error=ERRORS.NAME_UNIQUE
        )

    waypoint = Waypoint(
        name=waypoint_form.name,
        is_mandatory=waypoint_form.is_mandatory,
        description=waypoint_form.description,
        position_x=waypoint_form.position_x,
        position_y=waypoint_form.position_y,
        orientation=waypoint_form.orientation,
    )

    waypoint = waypoint_crud.create(waypoint)
    data = WaypointSerializer(waypoint)
    return WaypointDetailResponseSerializer(
        status="OK", message="Waypoint created", data=data
    )


@router.put(
    "/{waypoint_id}",
    response_model=WaypointDetailResponseSerializer,
    status_code=status.HTTP_200_OK,
    summary="Update either some or all Waypoint fields"
)
def update_waypoint(waypoint_id: int, waypoint_form: WaypointUpdateForm):
    """
    Actualizar waypoint

    Este endpoint permite actualizar un waypoint
    """
    waypoint: Waypoint = waypoint_crud.get_or_404(waypoint_id)
    if waypoint_form.name is not None:
        waypoint.name = waypoint_form.name
    if waypoint_form.is_mandatory is not None:
        waypoint.is_mandatory = waypoint_form.is_mandatory
    if waypoint_form.description is not None:
        waypoint.description = waypoint_form.description
    if waypoint_form.position_x is not None:
        waypoint.position_x = waypoint_form.position_x
    if waypoint_form.position_y is not None:
        waypoint.position_y = waypoint_form.position_y
    if waypoint_form.orientation is not None:
        waypoint.orientation = waypoint_form.orientation
    waypoint_crud.update(waypoint)
    waypoint = waypoint_crud.get_or_404(waypoint_id)
    data = WaypointSerializer(waypoint)
    return WaypointDetailResponseSerializer(
        status="OK", message="Waypoint updated", data=data
    )


@router.delete(
    "/{waypoint_id}", response_model=SimpleResponse, status_code=status.HTTP_200_OK,
    summary="Waypoint Delete"
)
def delete_waypoint(waypoint_id: int):
    """
    Eliminar waypoint por id

    Este endpoint permite eliminar una waypoint en formato JSON
    """
    waypoint_crud.delete(waypoint_id)
    return SimpleResponse(status="OK", message="Waypoint deleted")