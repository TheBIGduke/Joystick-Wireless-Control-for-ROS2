# python imports

# 3rd party imports
from pydantic import BaseModel

# local imports
from delivery_bridge.webapp.apps.base.serializers import DataResponse
from delivery_bridge.webapp.settings import FunctionMode


class FunctionModeSerializer(BaseModel):
    mode: FunctionMode
    ready: bool


class FunctionModeResponseSerializer(DataResponse):
    data: FunctionModeSerializer
