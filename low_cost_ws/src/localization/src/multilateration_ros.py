#! /usr/bin/env python3

import time

import numpy as np
import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from scipy.optimize import minimize
from std_msgs.msg import Float64MultiArray

method = "Nelder-Mead"

minimize_options = {}
minimize_options["Nelder-Mead"] = {
    "maxiter": int(1e4),
    "maxfev": int(1e4),
    "xatol": 1e-6,
    "fatol": 1e-6,
}
minimize_options["TNC"] = {"maxfun": int(1e3)}
minimize_options["BFGS"] = {"maxiter": int(1e4)}


class Args:
    def __init__(self, dictionary):
        for key, value in dictionary.items():
            setattr(self, key, value)


class AnchorLoader:
    @staticmethod
    def load_yaml(yaml_file):
        with open(yaml_file, "r") as file:
            try:
                data = yaml.safe_load(file)
                return data
            except yaml.YAMLError as e:
                print(f"Error loading YAML file: {e}")
                return None

    @staticmethod
    def convert_to_numpy(data):
        anchor_list = []
        for key, value in data.items():
            anchor_list.append([value["x"], value["y"], value["z"]])
        return np.array(anchor_list)

    @staticmethod
    def load_and_convert(yaml_file):
        data = AnchorLoader.load_yaml(yaml_file)
        if data:
            return AnchorLoader.convert_to_numpy(data)
        else:
            return None


class Multilateration:
    def __init__(self, args):
        self.distances_msg = Float64MultiArray()
        self.distances_sub = rospy.Subscriber("uwb1/distances", Float64MultiArray, self.distances_callback)
        self.pose_pub = rospy.Publisher("uwb1/pose", PoseStamped, queue_size=100)
        self.distances = np.zeros((10, 6))
        self.anchor_pos = AnchorLoader.load_and_convert(args.config_path)
        self.robot_pos = np.zeros((1, 3))
        self.optim_pos = np.zeros((1, 3))
        print("Anchor poses:\n", self.anchor_pos)

    def distances_callback(self, msg):
        msg.data = np.array(msg.data)
        self.distances_msg = msg
        self.distances = np.array([msg.data])
        self.optimize()
        self.pub_pose()

    def residual(self, robot_pos):
        estimated_distances = np.linalg.norm(self.anchor_pos - robot_pos, axis=1)
        error = estimated_distances - self.distances
        squared_error = np.square(error)
        return np.sum(squared_error)

    def optimize(self):
        # time this function
        start_time = time.time()
        result = minimize(self.residual, self.optim_pos, method=method, options=minimize_options[method])
        end_time = time.time()
        time_elapsed = end_time - start_time

        if 0 and time_elapsed * 1000.0 > 10.0 or not result.success:
            print("Time elapsed: {:.2f} ms".format(time_elapsed * 1000.0))
            print(f"nfev: {result.nfev}")
            print(f"nit: {result.nit}")
            print(f"succes: {result.success}")
        self.optim_pos = np.array([result.x])
        if result.success:
            self.robot_pos = np.array([result.x])
        else:
            print("Optimization failed to converge.")
        print(
            "Robot Coordinates (x, y, z): ({:.2f}, {:.2f}, {:.2f})".format(
                self.robot_pos[0][0] / 1000.0,
                self.robot_pos[0][1] / 1000.0,
                self.robot_pos[0][2] / 1000.0,
            )
        )

    def pub_pose(self):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "wamv/uwb_origin"
        pose.pose.position.x = self.robot_pos[0][0] / 1000.0
        pose.pose.position.y = self.robot_pos[0][1] / 1000.0
        pose.pose.position.z = self.robot_pos[0][2] / 1000.0
        self.pose_pub.publish(pose)

    def update_robot_position(self, new_position):
        # Calculate the difference between the new and old positions
        diff = new_position - self.robot_pos
        norm_diff = np.linalg.norm(diff)

        if norm_diff > 0:
            # Scale the difference to stay within the max_position_update_distance
            scaling_factor = min(norm_diff, self.max_position_update_distance * 1000) / norm_diff
            print("Scaling factor: {:.2f}".format(scaling_factor))
            self.robot_pos += scaling_factor * diff * self.position_update_rate


if __name__ == "__main__":
    rospy.init_node("multilateration", anonymous=False)
    config_path = rospy.get_param("~config_path")
    args = {"config_path": config_path}
    args = Args(args)
    multilateration = Multilateration(args)
    rospy.spin()
