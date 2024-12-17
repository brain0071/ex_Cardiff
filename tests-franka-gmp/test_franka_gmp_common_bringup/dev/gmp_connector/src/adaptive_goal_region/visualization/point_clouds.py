import cv2
import numpy as np
import rospy
import tf2_ros
import tf2_py as tf2
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PointStamped

def extract_point_clouds(depth, K, segmap=None, rgb=None, z_range=[0.2, 1.8], segmap_id=0,
                         skip_border_objects=False, margin_px=5):
    """
    Converts depth map + intrinsics to point cloud.
    If segmap is given, also returns segmented point clouds. If rgb is given, also returns pc_colors.

    Arguments:
        depth {np.ndarray} -- HxW depth map in m
        K {np.ndarray} -- 3x3 camera Matrix

    Keyword Arguments:
        segmap {np.ndarray} -- HxW integer array that describes segeents (default: {None})
        rgb {np.ndarray} -- HxW rgb image (default: {None})
        z_range {list} -- Clip point cloud at minimum/maximum z distance (default: {[0.2,1.8]})
        segmap_id {int} -- Only return point cloud segment for the defined id (default: {0})
        skip_border_objects {bool} -- Skip segments that are at the border of the depth map to avoid artificial edges (default: {False})
        margin_px {int} -- Pixel margin of skip_border_objects (default: {5})

    Returns:
        [np.ndarray, dict[int:np.ndarray], np.ndarray] -- Full point cloud, point cloud segments, point cloud colors
    """

    if K is None:
        raise ValueError('K is required either as argument --K or from the input numpy file')

    # Convert to pc
    pc_full, pc_colors = depth2pc(depth, K, rgb)

    # Threshold distance
    if pc_colors is not None:
        pc_colors = pc_colors[(pc_full[:, 2] < z_range[1]) & (pc_full[:, 2] > z_range[0])]
    pc_full = pc_full[(pc_full[:, 2] < z_range[1]) & (pc_full[:, 2] > z_range[0])]

    # Extract instance point clouds from segmap and depth map
    pc_segments = {}
    if segmap is not None:
        pc_segments = {}
        obj_instances = [segmap_id] if segmap_id else np.unique(segmap[segmap > 0])
        for i in obj_instances:
            if skip_border_objects and not i == segmap_id:
                obj_i_y, obj_i_x = np.where(segmap == i)
                if np.any(obj_i_x < margin_px) or np.any(obj_i_x > segmap.shape[1] - margin_px) or np.any(
                        obj_i_y < margin_px) or np.any(obj_i_y > segmap.shape[0] - margin_px):
                    print('object {} not entirely in image bounds, skipping'.format(i))
                    continue
            inst_mask = segmap == i
            pc_segment, _ = depth2pc(depth * inst_mask, K)
            pc_segments[i] = pc_segment[(pc_segment[:, 2] < z_range[1]) & (pc_segment[:, 2] > z_range[
                0])]  # regularize_pc_point_count(pc_segment, grasp_estimator._contact_grasp_cfg['DATA']['num_point'])

    return pc_full, pc_segments, pc_colors


def depth2pc(depth, K, rgb=None):
    """
    Convert depth and intrinsics to point cloud and optionally point cloud color
    :param depth: hxw depth map in m
    :param K: 3x3 Camera Matrix with intrinsics
    :returns: (Nx3 point cloud, point cloud color)
    """

    mask = np.where(depth > 0)
    x, y = mask[1], mask[0]

    normalized_x = (x.astype(np.float32) - K[0, 2])
    normalized_y = (y.astype(np.float32) - K[1, 2])

    world_x = normalized_x * depth[y, x] / K[0, 0]
    world_y = normalized_y * depth[y, x] / K[1, 1]
    world_z = depth[y, x]

    if rgb is not None:
        rgb = rgb[y, x, :]

    pc = np.vstack((world_x, world_y, world_z)).T
    return (pc, rgb)


def load_available_input_data(p, K=None):
    """
    Load available data from input file path.

    Numpy files .npz/.npy should have keys
    'depth' + 'K' + (optionally) 'segmap' + (optionally) 'rgb'
    or for point clouds:
    'xyz' + (optionally) 'xyz_color'

    png files with only depth data (in mm) can be also loaded.
    If the image path is from the GraspNet dataset, corresponding rgb, segmap and intrinic are also loaded.

    :param p: .png/.npz/.npy file path that contain depth/pointcloud and optionally intrinsics/segmentation/rgb
    :param K: 3x3 Camera Matrix with intrinsics
    :returns: All available data among segmap, rgb, depth, cam_K, pc_full, pc_colors
    """

    segmap, rgb, depth, pc_full, pc_colors = None, None, None, None, None

    if K is not None:
        if isinstance(K, str):
            cam_K = eval(K)
        cam_K = np.array(K).reshape(3, 3)

    if '.np' in p:
        data = np.load(p, allow_pickle=True)
        if '.npz' in p:
            keys = data.files
        else:
            keys = []
            if len(data.shape) == 0:
                data = data.item()
                keys = data.keys()
            elif data.shape[-1] == 3:
                pc_full = data
            else:
                depth = data

        if 'depth' in keys:
            depth = data['depth']
            if K is None and 'K' in keys:
                cam_K = data['K'].reshape(3, 3)
            if 'segmap' in keys:
                segmap = data['segmap']
            if 'seg' in keys:
                segmap = data['seg']
            if 'rgb' in keys:
                rgb = data['rgb']
                rgb = np.array(cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB))
        elif 'xyz' in keys:
            pc_full = np.array(data['xyz']).reshape(-1, 3)
            if 'xyz_color' in keys:
                pc_colors = data['xyz_color']
    elif '.png' in p:
        if os.path.exists(p.replace('depth', 'label')):
            # graspnet data
            depth, rgb, segmap, K = load_graspnet_data(p)
        elif os.path.exists(p.replace('depths', 'images').replace('npy', 'png')):
            rgb = np.array(Image.open(p.replace('depths', 'images').replace('npy', 'png')))
        else:
            depth = np.array(Image.open(p))
    else:
        raise ValueError('{} is neither png nor npz/npy file'.format(p))

    return segmap, rgb, depth, cam_K, pc_full, pc_colors


def get_pc_data(path=None):
    if path is None:
        path = "storage/captures/002/data.npy"

    K = None
    skip_border_objects = False
    z_range = [0.4, 1.4]

    segmap, rgb, depth, cam_K, pc_full, pc_colors = load_available_input_data(path, K=K)

    if pc_full is None:
        pc_full, pc_segments, pc_colors = extract_point_clouds(
            depth,
            cam_K,
            segmap=segmap,
            rgb=rgb,
            skip_border_objects=skip_border_objects,
            z_range=z_range,
        )
    print(f"First Z value was = {pc_full[0][2]}")
    pc_full = np.array(transform_point_cloud_frame(pc_full))
    print(f"Transformed Z value is = {pc_full[0][2]:.3f}")
    return pc_full, pc_colors


def transform_point_cloud_frame(cloud_points):

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1)  # Wait for the listener to get some data

    transformed_points = []
    for point in cloud_points:
        point_stamped = PointStamped()
        point_stamped.point.x = point[0]
        point_stamped.point.y = point[1]
        point_stamped.point.z = point[2]
        point_stamped.header.frame_id = "camera_depth_optical_frame"
        point_stamped.header.stamp = rospy.Time(0)

        # Transform the point
        transformed_point = tf_buffer.transform(point_stamped, "world", rospy.Duration(1.0))
        transformed_points.append(
            (transformed_point.point.x, transformed_point.point.y, transformed_point.point.z))
    return transformed_points