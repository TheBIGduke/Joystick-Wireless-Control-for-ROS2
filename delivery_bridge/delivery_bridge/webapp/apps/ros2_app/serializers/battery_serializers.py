# python imports

# 3rd party imports
from pydantic import BaseModel

# local imports
from delivery_bridge.webapp.apps.base.serializers import DataResponse


class BatterySerializer(BaseModel):
    power_supply_status: str
    voltage: float
    percentage: int


class BatteryResponseSerializer(DataResponse):
    data: BatterySerializer
