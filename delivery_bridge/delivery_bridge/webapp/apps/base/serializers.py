# python imports
# from os import path, makedirs
# import yaml
import logging
from datetime import datetime

# 3rd party imports
from pydantic import BaseModel, Field

# local imports
# from delivery_bridge.webapp.settings import APP_DATA_DIR
from delivery_bridge.utils import EnumWithDescription

logger = logging.getLogger(__name__)


class BaseSerializer(BaseModel):
    def dump_to_logger(self, indent=0):
        indent_str = "  " * indent
        for k, v in self.__dict__.items():
            if isinstance(v, BaseSerializer):
                logger.info(f"{indent_str}{k}:")
                v.dump_to_logger(indent + 1)
            elif isinstance(v, EnumWithDescription):
                logger.info(f"{indent_str}{k}: \t{v.value}")
            else:
                logger.info(f"{indent_str}{k}: \t{v}")

    def set_from_dict(self, data: dict):
        # remove None values
        data = {k: v for k, v in data.items() if v is not None}
        for k, v in data.items():
            # remove None values from nested dictionaries
            if isinstance(v, dict):
                data[k] = {k: v for k, v in v.items() if v is not None}
            # remove None values from nested lists
            elif isinstance(v, list):
                data[k] = [i for i in v if i is not None]

        # print("-"*50)
        # print(self.settings_name, "DATA TO SET", data)

        for k, v in self.__dict__.items():
            # skip if the attribute is not in the data
            if k not in data:
                continue
            if isinstance(v, BaseSerializer):
                v.set_from_dict(data[k])
            elif isinstance(v, EnumWithDescription):
                try:
                    setattr(self, k, v.__class__(data[k]))
                except ValueError as e:
                    logger.warning(f"Error setting {k}: {e}")
            elif k in data:
                setattr(self, k, data[k])

    def get_dict(self, object) -> dict:
        data_out = {}
        for k, v in object.__dict__.items():
            if isinstance(v, BaseModel):
                data_out[k] = self.get_dict(v)
            elif isinstance(v, EnumWithDescription):
                data_out[k] = v.value
            elif isinstance(v, datetime):
                data_out[k] = v.strftime("%Y-%m-%d %H:%M:%S")
            else:
                data_out[k] = v
        return data_out

    def to_dict(self) -> dict:
        return self.get_dict(self)


class SimpleResponse(BaseModel):
    status: str = Field(examples=["OK", "FAIL"])
    message: str = Field(examples=["Operation successful", "Operation failed"])


class DataResponse(BaseModel):
    status: str = Field(examples=["OK", "FAIL"])
    message: str = Field(examples=["Operation successful", "Operation failed"])
    data: dict = Field(examples=[{"key": "value"}])


class ErrorResponse(BaseModel):
    status: str = Field(examples=["FAIL"])
    message: str = Field(examples=["Operation failed"])
    error: str = Field(examples=["Error message"])

"""
class DataWithPaginationLegacy(BaseModel):
    count: int = Field(examples=[15])
    next: str | None = Field(
        default=None, examples=["http://0.0.0.0:9009/robot/notification/?page=3"]
    )
    previous: str | None = Field(
        default=None, examples=["http://0.0.0.0:9009/robot/notification/?page=1"]
    )
    results: list = Field(examples=[])
"""

class DataWithPagination(BaseModel):
    page: int = Field(examples=[1])
    page_count: int = Field(examples=[10])
    pages: list[str] = Field(examples=[["1", "2", "...", "10"]])
    last_page: int = Field(examples=[10])
    prev_page: int | None = Field(examples=[None])
    next_page: int | None = Field(examples=[2])
    records_count: int = Field(examples=[100])
    records: list = Field(examples=[[]])


# class EndpointSerializer(BaseSerializer):
#     url: str = ""
#     method: str = "POST"
#     payload: dict = {}
#     headers: dict = {}

"""
class SettingSerializer(BaseSerializer):
    settings_name: str = "settings"
    last_updated: str = ""

    def load(self):
        # load settings from ~/.datia_demographics/settings/{settings_name}.yaml
        # create the directory if it does not exist
        if not path.exists(path.join(APP_DATA_DIR, "settings")):
            makedirs(path.join(APP_DATA_DIR, "settings"))
        try:
            with open(
                path.join(APP_DATA_DIR, f"settings/{self.settings_name}.yaml"), "r"
            ) as f:
                data = yaml.safe_load(f)
                self.set_from_dict(data)
                logger.info(f"{self.settings_name} settings loaded")
        except FileNotFoundError:
            logger.error(
                f"{self.settings_name} settings file not found, using default values"
            )
            self.save()

    def save(self):
        self.last_updated = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        # save settings to ~/.datia_demographics/settings/{settings_name}.yaml
        with open(
            path.join(APP_DATA_DIR, f"settings/{self.settings_name}.yaml"), "w"
        ) as f:
            data = self.get_dict(self)
            yaml.dump(data, f, sort_keys=False)
        logger.info(f"{self.settings_name} settings saved")
"""
        
"""
class RosParamsSerializer(BaseSerializer):
    ros_package: str = "ros_package"
    node_name: str = "node"
    last_updated: str = ""

    def load(self):
        # load params from ~/.{ros_package}/{node_name}_params.yaml
        params_path = path.join(path.expanduser("~"), "." + self.ros_package)
        # create the directory if it does not exist
        if not path.exists(params_path):
            makedirs(params_path)
        try:
            with open(
                path.join(params_path, self.node_name + "_params.yaml"), "r"
            ) as f:
                data = yaml.safe_load(f)
                data = data.get("/**", {}).get("ros__parameters", {})
                self.set_from_dict(data)
                logger.info(f"{self.node_name} params loaded")
        except FileNotFoundError:
            logger.error(
                f"{self.node_name} params file not found, using default values"
            )
            self.save()

    def save(self):
        self.last_updated = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        # save params to ~/.{ros_package}/{node_name}_params.yaml
        params_path = path.join(path.expanduser("~"), "." + self.ros_package)
        with open(path.join(params_path, self.node_name + "_params.yaml"), "w") as f:
            data = self.get_dict(self)
            yaml.dump({"/**": {"ros__parameters": data}}, f, sort_keys=False)
        logger.info(f"{self.node_name} params saved")
"""