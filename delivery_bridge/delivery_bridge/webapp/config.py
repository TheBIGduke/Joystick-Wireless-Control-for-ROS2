import os
# import random

from .settings import APP_DATA_DIR


# def generate_secret_key():
#     seq = "abcdefghijklmnopqrstuvwxyz0123456789!@#$%^&*(-_=+)"
#     return "".join(random.choice(seq) for _ in range(64))


# create app data dir if not exists
if not os.path.exists(APP_DATA_DIR):
    os.makedirs(APP_DATA_DIR)


# def try_delete_media_file(file_path: str) -> bool:
#     try:
#         os.remove(os.path.join(APP_DATA_DIR, "media/", file_path))
#         return True
#     except Exception as e:
#         print(f"Error deleting file {file_path}: {e}")
#         return False
