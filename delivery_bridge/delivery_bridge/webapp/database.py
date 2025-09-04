# python imports

# 3rd party imports
from sqlmodel import SQLModel, create_engine

# local imports
from .dependencies import DATABASE_URL


engine = create_engine(DATABASE_URL)


def createDatabase() -> bool:
    try:
        SQLModel.metadata.create_all(engine)
        return True
    except Exception as e:
        print(f"ERROR: Error creating database: {e}")
        return False


def checkIfDatabaseExist():
    pass
