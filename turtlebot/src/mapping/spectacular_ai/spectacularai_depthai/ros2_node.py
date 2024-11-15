"""
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

Spectacular AI ROS2 node that manages the OAK-D device through DepthAI Python API
"""

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
import tf2_ros as tf2

import spectacularAI
import depthai
import numpy as np

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import CameraInfo, PointCloud2, PointField, Image
from std_msgs.msg import Empty


PUBLISHER_QUEUE_SIZE = 10


def toRosTime(timeInSeconds):
    t = Time()
    t.sec = int(timeInSeconds)
    t.nanosec = int((timeInSeconds % 1) * 1e9)
    return t


# def toPoseMessage(cameraPose, ts, frame_id="world"):
#     msg = PoseStamped()
#     msg.header.stamp = ts
#     msg.header.frame_id = frame_id
#     msg.pose.position.x = cameraPose.position.x
#     msg.pose.position.y = cameraPose.position.y
#     msg.pose.position.z = cameraPose.position.z
#     msg.pose.orientation.x = cameraPose.orientation.x
#     msg.pose.orientation.y = cameraPose.orientation.y
#     msg.pose.orientation.z = cameraPose.orientation.z
#     msg.pose.orientation.w = cameraPose.orientation.w
#     return msg


# def toTfMessage(cameraPose, ts, parent_frame_id="world", child_frame_id="left_camera"):
#     msg = TFMessage()
#     msg.transforms = []
#     transform = TransformStamped()
#     transform.header.stamp = ts
#     transform.header.frame_id = parent_frame_id
#     transform.child_frame_id = child_frame_id
#     transform.transform.translation.x = cameraPose.position.x
#     transform.transform.translation.y = cameraPose.position.y
#     transform.transform.translation.z = cameraPose.position.z
#     transform.transform.rotation.x = cameraPose.orientation.x
#     transform.transform.rotation.y = cameraPose.orientation.y
#     transform.transform.rotation.z = cameraPose.orientation.z
#     transform.transform.rotation.w = cameraPose.orientation.w
#     msg.transforms.append(transform)
#     return msg


# def toCameraInfoMessage(camera, frame, ts):
#     intrinsic = camera.getIntrinsicMatrix()
#     msg = CameraInfo()
#     msg.header.stamp = ts
#     msg.header.frame_id = "left_camera"
#     msg.height = frame.shape[0]
#     msg.width = frame.shape[1]
#     msg.distortion_model = "none"
#     msg.d = []
#     msg.k = intrinsic.ravel().tolist()
#     return msg


class SpectacularAINode(Node):
    def __init__(self):
        super().__init__("spectacular_ai_node")
        self.declare_parameter("recording_folder", rclpy.Parameter.Type.STRING)
        self.declare_parameter("do_recording_and_slam", rclpy.Parameter.Type.BOOL)

        self.declare_parameter("world_frame", "world")
        self.declare_parameter("camera_frame", "left_camera")

        self.declare_parameter("publish_tf", False)
        self.declare_parameter("publish_pointcloud", False)

        self.reset_odometry_subscription = self.create_subscription(
            Empty, "/vio/reset_odometry", self.onResetOdometryCallback, 10
        )

        self.odometry_publisher = self.create_publisher(PoseStamped, "/slam/odometry", PUBLISHER_QUEUE_SIZE)
        self.keyframe_publisher = self.create_publisher(PoseStamped, "/slam/keyframe", PUBLISHER_QUEUE_SIZE)
        self.left_publisher = self.create_publisher(Image, "/slam/left", PUBLISHER_QUEUE_SIZE)
        self.camera_info_publisher = self.create_publisher(CameraInfo, "/slam/camera_info", PUBLISHER_QUEUE_SIZE)

        self.publish_tf = self.get_parameter("publish_tf").value
        if self.publish_tf:
            self.tf_broadcaster = tf2.TransformBroadcaster(self)

        self.publish_pointcloud: bool = self.get_parameter("publish_pointcloud").value
        if self.publish_pointcloud:
            self.pointcloud_publisher = self.create_publisher(PointCloud2, "/slam/pointcloud", PUBLISHER_QUEUE_SIZE)

        self.bridge = CvBridge()
        self.keyframes = {}
        self.latestOutputTimestamp = None

        self.pipeline = depthai.Pipeline()
        config = spectacularAI.depthai.Configuration()

        recording_folder = str(self.get_parameter("recording_folder").value)
        do_recording_and_slam = bool(self.get_parameter("do_recording_and_slam").value)
        if recording_folder:
            self.get_logger().info("Recording: " + recording_folder)
            config.recordingFolder = recording_folder
            config.recordingOnly = not do_recording_and_slam

        config.internalParameters = {
            "ffmpegVideoCodec": "libx264 -crf 15 -preset ultrafast",
            "computeStereoPointCloud": "true",
            "computeDenseStereoDepthKeyFramesOnly": "true",
        }
        config.useSlam = True

        self.vio_pipeline = spectacularAI.depthai.Pipeline(self.pipeline, config, self.onMappingOutput)
        self.device = depthai.Device(self.pipeline)
        self.vio_session = self.vio_pipeline.startSession(self.device)
        self.timer = self.create_timer(0, self.processOutput)

        self.get_logger().info("Spectacular AI node started.")

    def onResetOdometryCallback(self, msg: Empty):
        self.vio_session.close()
        self.vio_session = self.vio_pipeline.startSession(self.device)

    def processOutput(self):
        while self.vio_session.hasOutput():
            self.onVioOutput(self.vio_session.getOutput())

    def onVioOutput(self, vioOutput):
        camera_pose = vioOutput.getCameraPose(0).pose
        timestamp = toRosTime(camera_pose.time)
        self.latestOutputTimestamp = timestamp

        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = self.get_parameter("world_frame").value
        pose_msg.pose.position.x = camera_pose.position.x
        pose_msg.pose.position.y = camera_pose.position.y
        pose_msg.pose.position.z = camera_pose.position.z
        pose_msg.pose.orientation.x = camera_pose.orientation.x
        pose_msg.pose.orientation.y = camera_pose.orientation.y
        pose_msg.pose.orientation.z = camera_pose.orientation.z
        pose_msg.pose.orientation.w = camera_pose.orientation.w
        self.odometry_publisher.publish(pose_msg)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = timestamp
            tf.header.frame_id = self.get_parameter("world_frame").value
            tf.child_frame_id = self.get_parameter("camera_frame").value
            tf.transform.translation.x = camera_pose.position.x
            tf.transform.translation.y = camera_pose.position.y
            tf.transform.translation.z = camera_pose.position.z
            tf.transform.rotation.x = camera_pose.orientation.x
            tf.transform.rotation.y = camera_pose.orientation.y
            tf.transform.rotation.z = camera_pose.orientation.z
            tf.transform.rotation.w = camera_pose.orientation.w
            self.tf_broadcaster.sendTransform(tf)

    def onMappingOutput(self, output):
        for frame_id in output.updatedKeyFrames:
            keyFrame = output.map.keyFrames.get(frame_id)
            if not keyFrame:
                continue  # Deleted keyframe
            if not keyFrame.pointCloud:
                continue
            if not self.hasKeyframe(frame_id):
                self.newKeyFrame(frame_id, keyFrame)

    def hasKeyframe(self, frame_id):
        return frame_id in self.keyframes

    def newKeyFrame(self, frame_id, keyframe):
        if not self.latestOutputTimestamp:
            return
        timestamp = toRosTime(keyframe.frameSet.primaryFrame.cameraPose.pose.time)
        self.keyframes[frame_id] = True
        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = self.get_parameter("world_frame").value
        pose_msg.pose.position.x = keyframe.frameSet.primaryFrame.cameraPose.pose.position.x
        pose_msg.pose.position.y = keyframe.frameSet.primaryFrame.cameraPose.pose.position.y
        pose_msg.pose.position.z = keyframe.frameSet.primaryFrame.cameraPose.pose.position.z
        pose_msg.pose.orientation.x = keyframe.frameSet.primaryFrame.cameraPose.pose.orientation.x
        pose_msg.pose.orientation.y = keyframe.frameSet.primaryFrame.cameraPose.pose.orientation.y
        pose_msg.pose.orientation.z = keyframe.frameSet.primaryFrame.cameraPose.pose.orientation.z
        pose_msg.pose.orientation.w = keyframe.frameSet.primaryFrame.cameraPose.pose.orientation.w
        self.keyframe_publisher.publish(pose_msg)

        left_frame_bitmap = keyframe.frameSet.primaryFrame.image.toArray()
        left_msg = self.bridge.cv2_to_imgmsg(left_frame_bitmap, encoding="mono8")
        left_msg.header.stamp = timestamp
        left_msg.header.frame_id = self.get_parameter("camera_frame").value
        self.left_publisher.publish(left_msg)

        camera = keyframe.frameSet.primaryFrame.cameraPose.camera
        intrinsic = camera.getIntrinsicMatrix()
        camera_info_msg = CameraInfo()
        camera_info_msg.header.stamp = timestamp
        camera_info_msg.header.frame_id = self.get_parameter("camera_frame").value
        camera_info_msg.height = left_frame_bitmap.shape[0]
        camera_info_msg.width = left_frame_bitmap.shape[1]
        camera_info_msg.distortion_model = "none"
        camera_info_msg.d = []
        camera_info_msg.k = intrinsic.ravel().tolist()
        self.camera_info_publisher.publish(camera_info_msg)

        if self.publish_pointcloud:
            self.publishPointCloud(keyframe, timestamp)

    # NOTE This seems a bit slow.
    def publishPointCloud(self, keyframe, timestamp):
        camToWorld = keyframe.frameSet.rgbFrame.cameraPose.getCameraToWorldMatrix()
        positions = keyframe.pointCloud.getPositionData()
        pc = np.zeros((positions.shape[0], 6), dtype=np.float32)
        p_C = np.vstack((positions.T, np.ones((1, positions.shape[0])))).T
        pc[:, :3] = (camToWorld @ p_C[:, :, None])[:, :3, 0]

        pc_msg = PointCloud2()
        pc_msg.header.stamp = timestamp
        pc_msg.header.frame_id = self.get_parameter("world_frame").value
        if keyframe.pointCloud.hasColors():
            pc[:, 3:] = keyframe.pointCloud.getRGB24Data() * (1.0 / 255.0)
        pc_msg.point_step = 4 * 6
        pc_msg.height = 1
        pc_msg.width = pc.shape[0]
        pc_msg.row_step = pc_msg.point_step * pc.shape[0]
        pc_msg.data = pc.tobytes()
        pc_msg.is_bigendian = False
        pc_msg.is_dense = False
        ros_dtype = PointField.FLOAT32
        itemsize = np.dtype(np.float32).itemsize
        pc_msg.fields = [
            PointField(name=n, offset=i * itemsize, datatype=ros_dtype, count=1) for i, n in enumerate("xyzrgb")
        ]
        self.pointcloud_publisher.publish(pc_msg)


def main(args=None):
    rclpy.init(args=args)
    sai_node = SpectacularAINode()
    rclpy.spin(sai_node)
    sai_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
