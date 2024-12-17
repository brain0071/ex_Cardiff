import argparse
import time

from adaptive_goal_region.robot_controller import RobotController


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="AGR - Gazebo Image Capture and Save Results"
    )

    # Add the arguments
    parser.add_argument(
        "-e",
        "--environment",
        default="undefined",
        type=str,
        required=True,
        help="The environment name",
    )

    args = parser.parse_args()

    # Now you can use the environment variable
    env_name = args.environment
    print(f"The environment name is: {env_name}")

    robot_controller = RobotController(real_robot=False)

    robot_controller.hand_open()

    robot_controller.go_to_capture_location()

    robot_controller.capture_image_and_save_info(env_name)
    time.sleep(3)
    print("CAPTURE COMPLETED!")
