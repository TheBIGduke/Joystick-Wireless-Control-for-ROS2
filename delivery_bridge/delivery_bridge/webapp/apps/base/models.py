# python imports
import logging
from typing import Optional
from datetime import datetime

# 3rd party imports
from sqlmodel import Field, SQLModel, Session

# local imports
from delivery_bridge.webapp.database import engine

logger = logging.getLogger(__name__)


class BaseModel(SQLModel):
    id: Optional[int] = Field(default=None, primary_key=True)
    created_at: datetime = Field(default=datetime.now())
    updated_at: datetime = Field(default=datetime.now())

    def __str__(self):
        return f"{self.__class__.__name__} {self.id}"

    def save(self) -> "BaseModel":
        with Session(engine) as session:
            session.add(self)
            session.commit()
            session.refresh(self)
        return self

    def get(self, id: int) -> "BaseModel":
        with Session(engine) as session:
            return session.get(self.__class__, id)

    def delete(self) -> None:
        with Session(engine) as session:
            session.delete(self)
            session.commit()
        return None
