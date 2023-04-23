import time
from traceback import print_tb

from yaml import BlockEndToken
import rclpy
import sensor_msgs.msg
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion
from .fastslam64.slam import Slam
from dv_common.point_cloud2 import create_cloud_xyz32, create_cloud_xyz32_intensity, read_points
from dv_common.constants import QOS_PROFILE
from dv_common.util import convert_to_quaternions
from typing import Dict
from std_msgs.msg import Bool
from .fastslam64.config_ros import config

from tvojemama.logger import Logger
from pathlib import Path

from std_msgs.msg import Header
from rclpy.publisher import Publisher
import numpy as np

import pickle

LOG = True


class SLAM(Node):
    def __init__(self):
        super().__init__("SLAM")
        self.slam = None
        # TODO: port the SLAM code to ROS
        self.go = False
        self._publishers: Dict[str, Publisher] = {}
        self.init_subscribers()
        self.init_publishers()
        self.log = []

        # LOGGER SETUP

        # 1. setup logger
        data_folder_path = Path.home() / Path("test_data")
        self.logger = Logger(log_name="slam", log_folder_name="AS", main_folder_path=data_folder_path, queue_size=2)

        self.logger.log(time.time(), ("SLAM_CONFIGURATION", {"SEED": config.SEED,
                                                             "N": config.N,
                                                             "DT": config.DT,
                                                             "THREADS": config.THREADS,
                                                             "GPU_HEAP_SIZE_BYTES": config.GPU_HEAP_SIZE_BYTES,
                                                             "THRESHOLD": config.THRESHOLD,
                                                             "sensor": {
                                                                 "RANGE": config.sensor.RANGE,
                                                                 "FOV": config.sensor.FOV,
                                                                 "VARIANCE": config.sensor.VARIANCE,
                                                                 "MAX_MEASUREMENTS": config.sensor.MAX_MEASUREMENTS,
                                                                 "MISS_PROB": config.sensor.MISS_PROB
                                                             },
                                                             "ODOMETRY_VARIANCE": config.ODOMETRY_VARIANCE,
                                                             "MAX_LANDMARKS": config.MAX_LANDMARKS}))

        # 2. log config message
        # self.logger.log(time.time(), ("SLAM_CONFIGURATION", config))
        # END OF LOGGER SETUP

    def init_subscribers(self):
        self.create_subscription(PointCloud2, "/xavier/cones/all", self.receive_cones, QOS_PROFILE)
        self.create_subscription(PoseWithCovarianceStamped, "/xavier/can/ins/ekf_position", self.receive_position, QOS_PROFILE)
        self.create_subscription(Bool, "/xavier/can/go", self.receive_go, qos_profile=QOS_PROFILE)
        self.create_subscription(Bool, "/xavier/car/finish", self.receive_finish, qos_profile=QOS_PROFILE)
        # self.create_subscription(Bool,"/xavier/can/emergency",self.receive_emergency,qos_profile=QOS_PROFILE)

    def init_publishers(self):
        self._publishers["landmarks"] = self.create_publisher(PointCloud2, "landmarks", QOS_PROFILE)
        self._publishers["pose_estimate"] = self.create_publisher(PoseStamped, "position", QOS_PROFILE)
        # self._publishers["local_landmarks"] = self.create_publisher(PointCloud2,"local_landmarks",qos_profile=QOS_PROFILE)

    def receive_cones(self, msg: PointCloud2):
        # print("cones")
        if self.slam is None:
            return
        if msg.height == 0 or msg.width == 0:
            return
        meas = self.pc_to_array(msg)
        meas
        if len(meas) > 0:
            estimate_pose, landmarks, colors = self.slam.set_measurements(self.pc_to_array(msg))
            pose = PoseStamped()
            pose.header.frame_id = "CoG"
            pose.pose.position.x = estimate_pose[0]
            pose.pose.position.y = estimate_pose[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation = convert_to_quaternions(0, 0, estimate_pose[2])
            # self.log.append([[time.time(),estimate_pose[0],estimate_pose[1],estimate_pose[2]],self.landmarks_to_array(landmarks,colors)])
            landmarks_arr = self.landmarks_to_array(landmarks, colors)
            slam_frame = {"x": estimate_pose[0],
                          "y": estimate_pose[1],
                          "heading": estimate_pose[2],
                          "yellow_cones": np.array(landmarks_arr[0]),
                          "blue_cones": np.array(landmarks_arr[1]),
                          "orange_cones": np.array(landmarks_arr[2]),
                          "big_cones": np.array(landmarks_arr[3])}
            self.logger.log(time.time(), ("SLAM_FRAME", slam_frame))
            # self.global_to_local(landmarks,colors,estimate_pose[0],estimate_pose[1],estimate_pose[2])
            self._publishers["pose_estimate"].publish(pose)
            self._publishers["landmarks"].publish(self.landmarks_to_pc(landmarks, colors))

    def receive_position(self, msg: PoseWithCovarianceStamped):
        # print("pos")
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        odometry = ((position.x, position.y), (orientation.x, orientation.y, orientation.z, orientation.w))
        if self.slam is None and self.go:
            self.slam = Slam(start_position=odometry, plot=False)
        if self.slam is not None:
           self.slam.set_odometry(odometry)

    def receive_go(self, msg: Bool):
        self.go = msg.data
        print("go")

    def receive_finish(self, msg: Bool):
        return
        print("saving")
        np.save("/home/zotac/cesko_slam.npy", self.log)

    def receive_emergency(self, msg: Bool):
        return
        print("saving")
        np.save("/home/zotac/cesko_slam.npy", self.log)

    def theta_to_quat(self, theta):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = np.sin(theta / 2)
        q.w = np.cos(theta / 2)
        return q

    def landmarks_to_pc(self, landmarks, colors):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "CoG"
        landmarks_3d = []
        for i in range(0, len(landmarks)):
            landmarks_3d.append([landmarks[i][0], landmarks[i][1], 0.0, colors[i]])
        return create_cloud_xyz32_intensity(header, landmarks_3d)

    def pc_to_array(self, pointcloud):
        cones = []
        for point in read_points(pointcloud):
            if np.sqrt(point[0]**2 + point[1]**2) < 14:
                cones.append([point[0], point[1], 0.0, point[3]])
        if len(cones) > 0:
            cones = np.array(cones)
            colors = cones[:, 3]
            colors = np.reshape(colors, (colors.shape[0], 1))
            cones = self.local_to_global(cones[:, :2], self.slam.odometry)
            cones = np.hstack((cones, colors))
            # print(cones)
            return cones
        return []

    def local_to_global(self, cones, odometry):
        """Convert cones from the car frame to the global frame"""

        cones = cones.copy()

        R = np.float32([
            [np.cos(odometry[2]), -np.sin(odometry[2])],
            [np.sin(odometry[2]), np.cos(odometry[2])]
        ])

        cones = np.matmul(R, cones.T).T
        cones[:, 0] += odometry[0] - 0.755
        cones[:, 1] += odometry[1]

        return cones

    def global_to_local(self, landmarks, colors, x, y, heading):
        inverse_heading = -heading  # +np.pi/2
        R = np.array([[np.cos(inverse_heading), -np.sin(inverse_heading)],
                      [np.sin(inverse_heading), np.cos(inverse_heading)]])
        translated = landmarks - (x, y)

        rotated = R.dot(translated.T).T
        rotated = np.hstack((rotated, colors))
        if len(rotated) > 0:
            point_cloud = self.landmarks_to_pc(rotated[:, :2], rotated[:, 2])
            self._publishers["local_landmarks"].publish(point_cloud)

    def landmarks_to_array(self, landmarks, colors):
        blue = []
        yellow = []
        orange = []
        big_orange = []
        for i in range(len(landmarks)):
            if colors[i] == 1:
                yellow.append([landmarks[i][0], landmarks[i][1]])
            elif colors[i] == 0:
                blue.append([landmarks[i][0], landmarks[i][1]])
            elif colors[i] == 3:
                big_orange.append([landmarks[i][0], landmarks[i][1]])
            else:
                orange.append([landmarks[i][0], landmarks[i][1]])
        return np.array([yellow, blue, orange, big_orange])


def main():
    rclpy.init()
    node = SLAM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()


if __name__ == "__main__":
    main()
