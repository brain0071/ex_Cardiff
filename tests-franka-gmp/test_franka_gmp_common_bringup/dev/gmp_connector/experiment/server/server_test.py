from typing import (
    Any,
    List,
    Optional,
)

import numpy as np
import requests
import os


HOME_DIR = os.environ["HOME"]


def request_graspnet_result(
    path=None, server_address="http://127.0.0.1/run", env_name=None
):
    if not path:
        print("Path is not defined.")
        return None
    print(f"REQUESTED PATH: {path}")

    if not env_name:
        env_name = "undefined"

    try:
        files = {"file": ("data.npy", open(path, "rb"))}
        response = requests.get(server_address, files=files, timeout=30)
    except Exception as e:
        print(
            f"Request failed, please make sure Contact Graspnet Server is running!\n{e}"
        )
        return None
    if response.status_code != 200:
        print("Grasping Pose Detection process failed!")
        return None

    if server_address:
        path = os.path.join(
            HOME_DIR,
            "dev",
            "gmp_connector",
            "storage",
            env_name,
            "temp",
            "poses.npz",
        )
        with open(path, "wb") as target_file:
            target_file.write(response.content)
        print(f"Results Saved: {path}")
        return path
    else:
        print(f"Response Text: {response.text}")
        return response.text


request_graspnet_result(
    path="raw_capture.npy", server_address="http://127.0.0.1:6161/run"
)
