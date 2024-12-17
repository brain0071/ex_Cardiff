import os
import numpy as np
import matplotlib.pyplot as plt


home_dir = os.path.expanduser("~")

# Base directory where the scene.npy files are located
base_dir = os.path.join(
    home_dir,
    "dev",
    "storage",
)


# Function to load and plot RGB data from npy file
def load_and_plot_rgb(base_dir, index):
    folder_name = str(index)
    file_path = os.path.join(base_dir, folder_name, "scene.npy")

    if os.path.exists(file_path):
        data = np.load(file_path, allow_pickle=True).item()
        rgb_data = data.get("rgb")
        depth_data = data.get("depth")

        if rgb_data is not None:
            plt.imshow(rgb_data)
            plt.title(f"RGB Image from folder {folder_name}")
            plt.show()
        else:
            print(f"No 'RGB' data found in {file_path}")
    else:
        print(f"File {file_path} does not exist.")


# Iterate through folders and process each one
for i in range(1, 4):  # Adjust the range as needed
    load_and_plot_rgb(base_dir, i)
