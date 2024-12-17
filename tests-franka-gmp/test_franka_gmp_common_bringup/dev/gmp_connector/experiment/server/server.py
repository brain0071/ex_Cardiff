import json
import os
import sys
import time
import numpy as np
import subprocess

from flask import Flask, request, send_file


BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HOME_DIR = os.environ["HOME"]
SCRIPT_DIRECTORY = os.path.join(
    HOME_DIR, "dev", "gmp_connector", "experiment", "server"
)
sys.path.append(os.path.join(BASE_DIR))

current_dir = os.path.dirname(__file__)
app = Flask(__name__)


@app.route("/run")
def read_file():
    path = request.args.get("path")
    file = request.files["file"]
    env_name = request.args.get("env", "undefined")
    subprocess.run(
        ["bash", os.path.join(SCRIPT_DIRECTORY, "directory_script.sh"), "-e", env_name]
    )

    if path:
        if os.path.exists(path) and os.path.isfile(path):
            print("Loading ", path)
        else:
            return f"File not found at path: {path}"
    elif file:
        path = file.filename
        path = os.path.join(
            HOME_DIR,
            "dev",
            "gmp_connector",
            "storage",
            env_name,
            "temp",
            "raw_capture.npy",
        )
        file.save(path)
        print("Received file saved as: ", path)
    else:
        return "Please provide a valid 'path' parameter."
    subprocess.run(
        ["bash", os.path.join(SCRIPT_DIRECTORY, "run_tools.sh"), "-e", env_name]
    )
    time.sleep(0.1)

    return send_file(path, as_attachment=True)


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=6161)
