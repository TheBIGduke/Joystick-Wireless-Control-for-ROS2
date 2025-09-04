from enum import Enum


class EnumWithDescription(Enum):
    def __new__(cls, *args, **kwds):
        obj = object.__new__(cls)
        obj._value_ = args[0]
        return obj

    # ignore the first param since it's already set by __new__
    def __init__(self, _: int, description: str = None):
        self._description_ = description
        self.__doc__ = description

    # this makes sure that the description is read-only
    @property
    def description(self):
        return self._description_

    @classmethod
    def getItems(cls):
        return [item.description for item in cls]

    # set by description
    @classmethod
    def setByDescription(cls, description):
        for item in cls:
            if item.description == description:
                return item
        return None
