# python imports

# 3rd party imports
from sqlmodel_crud_manager.crud import CRUDManager
from sqlmodel import Session, select

# local imports
from delivery_bridge.webapp.database import engine
from delivery_bridge.webapp.apps.waypoints.models import Waypoint

waypoint_crud = CRUDManager(Waypoint, engine)


# def validate_name_exists(name: str, map_id: int) -> bool:
def validate_name_exists(name: str) -> bool:
    with Session(engine) as session:
        statement = select(Waypoint).where(
            Waypoint.name == name #, Waypoint.map_id == map_id
        )
        waypoint = session.exec(statement).first()
        if waypoint:
            return True
        return False
