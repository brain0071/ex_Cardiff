import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
import tf.transformations as transformations


def delete_model_from_gazebo(model_name):
    """
    Deletes a model from Gazebo.

    :param model_name: Name of the model to be deleted.
    """
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp = delete_model(model_name)
        if resp.success:
            print(f"Model '{model_name}' deleted successfully.")
        else:
            print(f"Failed to delete model '{model_name}': {resp.status_message}")
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")


def spawn_line_in_gazebo(name, pose, line_length=1.0, line_radius=0.02):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    # Define the line model (SDF format)
    # This is a simplified example; you would need to create a proper SDF model
    line_sdf = f"""
    <sdf version='1.6'>
        <model name='{name}'>
            <static>true</static>
            <link name='link'>
                <visual name='visual'>
                    <geometry>
                        <cylinder>
                          <radius>{line_radius}</radius>
                          <length>{line_length}</length>
                        </cylinder>
                    </geometry>
                </visual>
            </link>
        </model>
    </sdf>
    """

    try:
        spawn_model(name, line_sdf, "", pose, "world")
        print(f"Spawned line '{name}' in Gazebo.")
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

