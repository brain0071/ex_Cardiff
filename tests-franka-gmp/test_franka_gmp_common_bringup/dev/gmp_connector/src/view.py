import numpy as np

from visualize import visualize_grasps


def get_data():
    with np.load(
        "/home/juanhernandezvega/dev/gmp_connector/storage/safa/temp/poses.npz",
        allow_pickle=True,
    ) as data:
        return dict(data)


data = get_data()

visualize_grasps(
    full_pc=data["pc_full"],
    pred_grasps_cam=data["pred_grasps_cam"].item(),
    scores=data["scores"].item(),
    plot_opencv_cam=True,
    pc_colors=data["pc_colors"],
)
