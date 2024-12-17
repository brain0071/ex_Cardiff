import argparse
import os
import time

from adaptive_goal_region.robot_controller import RobotController
from adaptive_goal_region.src.agr_helper import save_agr_data_as_txt


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="AGR - Gazebo Image Capture and Save Results"
    )

    # Add the arguments
    parser.add_argument(
        "-i", "--input_file", type=str, required=True, help="The input file"
    )
    parser.add_argument(
        "-o", "--output_dir", type=str, required=True, help="The output dir"
    )

    args = parser.parse_args()

    robot_controller = RobotController(real_robot=False)

    matrices_data = robot_controller.read_grasping_poses_file(args.input_file)
    poses, labels = robot_controller.convert_matrices_to_array_new(matrices_data)
    poses = robot_controller.array_frame_transformation(poses)
    poses = robot_controller.array_rotation(poses, -45)

    if any(
        pose[2] < 0.5 for pose in poses
    ):  # Assuming pose[2] is the z value in each pose
        print("Z value is less than 0.5. Stopping execution.")
        # Here you can use any method to stop execution, such as raising an exception or using sys.exit()
        raise SystemExit("Z value is less than 0.5. Stopping execution.")
    else:
        print("All z values are >= 0.5. Continuing execution.")
    save_agr_data_as_txt(poses, os.path.join(args.output_dir, "agr_output_single.txt"))

    time.sleep(3)
    print("TRANSFORMATION AND CLUSTERING COMPLETED!")
    print("GRASP POSES SAVED AT: ")
    print(args.output_dir)
