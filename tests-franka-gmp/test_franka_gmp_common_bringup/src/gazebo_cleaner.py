#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import DeleteModel

# List of base model names
base_model_names = [
    "cylinder",  # Example base name 1
    "box",  # Example base name 2
]


def delete_model(model_name):
    rospy.wait_for_service("/gazebo/delete_model")
    try:
        delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        resp = delete_model(model_name)
        if resp.success:
            rospy.loginfo("Deleted model '{}' successfully.".format(model_name))
        else:
            rospy.logwarn(
                "Failed to delete model '{}': {}".format(
                    model_name, resp.status_message
                )
            )
    except rospy.ServiceException as e:
        rospy.logerr("Delete model service call failed: {}".format(e))


if __name__ == "__main__":
    rospy.init_node("delete_models_node", anonymous=True)

    # Iterate over each base model name
    for base_name in base_model_names:
        # Try to delete models with suffixes from 0 to a reasonable upper limit (e.g., 100)
        for i in range(2):  # Adjust the range if you expect more than 100 instances
            if i == 0:
                model_name = base_name
            else:
                model_name = "{}_{}".format(base_name, i)
            delete_model(model_name)

    rospy.signal_shutdown("Script completed execution.")
