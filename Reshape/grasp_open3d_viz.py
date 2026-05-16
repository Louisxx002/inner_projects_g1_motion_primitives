import math
import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from ultralytics import YOLO

try:
    import open3d as o3d
except ImportError:  # pragma: no cover - runtime dependency check
    o3d = None


WAIST_YAW_RAD = 0.0

CAMERA_X_M = 47.64571478 / 1000.0
CAMERA_Y_M = 0.0
CAMERA_Z_M = 482.6678553 / 1000.0
CAMERA_TILT_DEG = 42.0


def rotation_z(yaw: float) -> np.ndarray:
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy, -sy, 0.0],
            [sy, cy, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )

def make_hip_from_camera_transform(
    waist_yaw_rad: float = WAIST_YAW_RAD,
    camera_x_m: float = CAMERA_X_M,
    camera_y_m: float = CAMERA_Y_M,
    camera_z_m: float = CAMERA_Z_M,
    camera_tilt_deg: float = CAMERA_TILT_DEG,
) -> np.ndarray:
    theta = math.radians(camera_tilt_deg)

    r_hip_from_cam_yaw0 = np.array(
        [
            [0.0, -math.cos(theta), math.sin(theta)],
            [-1.0, 0.0, 0.0],
            [0.0, -math.sin(theta), -math.cos(theta)],
        ],
        dtype=np.float64,
    )

    t_hip_from_cam_yaw0 = np.array(
        [camera_x_m, camera_y_m, camera_z_m],
        dtype=np.float64,
    )

    r_yaw = rotation_z(waist_yaw_rad)
    r_hip_from_cam = r_yaw @ r_hip_from_cam_yaw0
    t_hip_from_cam = r_yaw @ t_hip_from_cam_yaw0

    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = r_hip_from_cam
    transform[:3, 3] = t_hip_from_cam
    return transform


def transform_points(transform: np.ndarray, points: np.ndarray) -> np.ndarray:
    if points.size == 0:
        return points.reshape(0, 3)

    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    return points @ rotation.T + translation


class Open3DGraspVizNode(Node):
    def __init__(self) -> None:
        super().__init__("grasp_open3d_viz")

        if o3d is None:
            raise RuntimeError("open3d is not installed. Please install it before running grasp_open3d_viz.py")

        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("model_path", "yolo26s-seg.pt")
        self.declare_parameter("target_class", "carrot")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("point_stride", 3)
        self.declare_parameter("scene_stride", 6)
        self.declare_parameter("max_depth_m", 1.5)
        self.declare_parameter("waist_yaw_rad", WAIST_YAW_RAD)
        self.declare_parameter("camera_x_m", CAMERA_X_M)
        self.declare_parameter("camera_y_m", CAMERA_Y_M)
        self.declare_parameter("camera_z_m", CAMERA_Z_M)
        self.declare_parameter("camera_tilt_deg", CAMERA_TILT_DEG)

        color_topic = self.get_parameter("color_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        model_path = self.get_parameter("model_path").value
        self.target_class = self.get_parameter("target_class").value
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.point_stride = max(1, int(self.get_parameter("point_stride").value))
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.scene_stride = max(1, int(self.get_parameter("scene_stride").value))
        waist_yaw_rad = float(self.get_parameter("waist_yaw_rad").value)
        camera_x_m = float(self.get_parameter("camera_x_m").value)
        camera_y_m = float(self.get_parameter("camera_y_m").value)
        camera_z_m = float(self.get_parameter("camera_z_m").value)
        camera_tilt_deg = float(self.get_parameter("camera_tilt_deg").value)

        self.bridge = CvBridge()
        self.model = YOLO(model_path)
        self.depth_image = None
        self.camera_info = None
        self.last_log_time = self.get_clock().now()
        self.data_lock = threading.Lock()

        self.t_hip_from_camera = make_hip_from_camera_transform(
            waist_yaw_rad=waist_yaw_rad,
            camera_x_m=camera_x_m,
            camera_y_m=camera_y_m,
            camera_z_m=camera_z_m,
            camera_tilt_deg=camera_tilt_deg,
        )

        self.latest_scene_points_world = np.zeros((0, 3), dtype=np.float64)
        self.latest_scene_colors = np.zeros((0, 3), dtype=np.float64)
        self.latest_object_points_world = np.zeros((0, 3), dtype=np.float64)
        self.latest_object_colors = np.zeros((0, 3), dtype=np.float64)
        self.latest_center_world = None

        self.create_subscription(Image, color_topic, self.color_callback, 10)
        self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)

        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="grasp_open3d_viz", width=1280, height=720)

        self.world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.30, origin=[0.0, 0.0, 0.0])
        self.camera_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.20, origin=[0.0, 0.0, 0.0])
        self.camera_frame.transform(self.t_hip_from_camera)
        self.scene_cloud = o3d.geometry.PointCloud()
        self.object_cloud = o3d.geometry.PointCloud()
        self.center_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.015)
        self.center_sphere.paint_uniform_color([1.0, 0.0, 0.0])
        self.center_sphere.translate([0.0, 0.0, -10.0])
        self.center_sphere_visible = False
        self.center_sphere_position = np.array([0.0, 0.0, -10.0], dtype=np.float64)

        self.vis.add_geometry(self.world_frame)
        self.vis.add_geometry(self.camera_frame)
        self.vis.add_geometry(self.scene_cloud)
        self.vis.add_geometry(self.object_cloud)
        self.vis.add_geometry(self.center_sphere)

        render_option = self.vis.get_render_option()
        render_option.point_size = 2.0
        render_option.background_color = np.array([0.05, 0.05, 0.05], dtype=np.float64)

        self.initialize_camera_view()

        self.create_timer(0.05, self.update_visualizer)

        self.get_logger().info(
            f"open3d viz started, color={color_topic}, depth={depth_topic}, info={camera_info_topic}"
        )

    def depth_callback(self, msg: Image) -> None:
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def color_callback(self, msg: Image) -> None:
        if self.depth_image is None or self.camera_info is None:
            self.log_throttled("waiting for depth image and camera info")
            return

        color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self.model(color_image, verbose=False)
        result = results[0]
        debug_image = result.plot()

        target = self.select_target_polygon(result, color_image.shape[:2])
        try:
            scene_points_camera, scene_colors_rgb = self.full_scene_to_point_cloud(color_image)
            scene_points_world = transform_points(self.t_hip_from_camera, scene_points_camera)
        except RuntimeError as exc:
            self.log_throttled(str(exc))
            scene_points_world = np.zeros((0, 3), dtype=np.float64)
            scene_colors_rgb = np.zeros((0, 3), dtype=np.float64)

        if target is None:
            self.log_throttled(f"no mask found for class '{self.target_class}'")
            with self.data_lock:
                self.latest_scene_points_world = scene_points_world
                self.latest_scene_colors = scene_colors_rgb
                self.latest_object_points_world = np.zeros((0, 3), dtype=np.float64)
                self.latest_object_colors = np.zeros((0, 3), dtype=np.float64)
                self.latest_center_world = None
            if self.publish_debug_image:
                cv2.imshow("grasp_open3d_detection", debug_image)
                cv2.waitKey(1)
            return

        contour, center_pixel = target
        mask = np.zeros(color_image.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [contour], 255)

        try:
            points_camera, colors_rgb = self.mask_to_point_cloud(mask, color_image)
        except RuntimeError as exc:
            self.log_throttled(str(exc))
            with self.data_lock:
                self.latest_scene_points_world = scene_points_world
                self.latest_scene_colors = scene_colors_rgb
                self.latest_object_points_world = np.zeros((0, 3), dtype=np.float64)
                self.latest_object_colors = np.zeros((0, 3), dtype=np.float64)
                self.latest_center_world = None
            if self.publish_debug_image:
                cv2.circle(debug_image, center_pixel, 6, (0, 0, 255), -1)
                cv2.imshow("grasp_open3d_detection", debug_image)
                cv2.waitKey(1)
            return

        points_world = transform_points(self.t_hip_from_camera, points_camera)

        center_world = None
        try:
            center_camera = self.pixel_to_camera_point(*center_pixel)
            center_world = transform_points(self.t_hip_from_camera, center_camera.reshape(1, 3))[0]
        except RuntimeError:
            center_world = None

        with self.data_lock:
            self.latest_scene_points_world = scene_points_world
            self.latest_scene_colors = scene_colors_rgb
            self.latest_object_points_world = points_world
            self.latest_object_colors = self.highlight_object_colors(colors_rgb)
            self.latest_center_world = center_world

        cv2.circle(debug_image, center_pixel, 6, (0, 255, 255), -1)
        cv2.putText(
            debug_image,
            f"mask center {center_pixel}",
            (center_pixel[0] + 10, max(center_pixel[1] - 10, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
        if self.publish_debug_image:
            cv2.imshow("grasp_open3d_detection", debug_image)
            cv2.waitKey(1)

        grasp_pixel = self.find_target_mask_center(result, color_image.shape[:2])
        if grasp_pixel is None:
            self.log_throttled(f"no mask found for class '{self.target_class}'")

        pixel_x, pixel_y = grasp_pixel
        try:
            point_optical = self.pixel_to_camera_point(pixel_x, pixel_y)

            point_robot = transform_points(self.t_hip_from_camera, point_optical.reshape(1, 3))[0]

            #self.log_throttled(f"visualizing {len(points_world)} object points in hip frame")
            self.log_throttled("grasp point in hip frame: "
                f"x={point_robot[0]:.4f}, y={point_robot[1]:.4f}, z={point_robot[2]:.4f}")
        except RuntimeError as exc:
            self.log_throttled(str(exc))
        

    def find_target_mask_center(
        self,
        result,
        image_shape: tuple[int, int],
    ) -> tuple[int, int] | None:
        if result.masks is None or result.boxes is None:
            return None

        classes = result.boxes.cls.cpu().numpy().astype(int)
        scores = result.boxes.conf.cpu().numpy()
        names = result.names
        polygons = result.masks.xy

        target_candidates = []
        for index, class_id in enumerate(classes):
            class_name = names[class_id]
            if class_name != self.target_class:
                continue

            polygon = polygons[index]
            if polygon is None or len(polygon) < 3:
                continue

            contour = np.round(polygon).astype(np.int32).reshape(-1, 1, 2)
            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue

            center_x = int(round(moments["m10"] / moments["m00"]))
            center_y = int(round(moments["m01"] / moments["m00"]))
            center_x = int(np.clip(center_x, 0, image_shape[1] - 1))
            center_y = int(np.clip(center_y, 0, image_shape[0] - 1))

            inside = cv2.pointPolygonTest(contour, (float(center_x), float(center_y)), False)
            if inside < 0:
                candidate_points = polygon.astype(np.float64)
                distances = np.sum((candidate_points - np.array([center_x, center_y])) ** 2, axis=1)
                nearest_index = int(np.argmin(distances))
                center_x = int(np.clip(round(candidate_points[nearest_index][0]), 0, image_shape[1] - 1))
                center_y = int(np.clip(round(candidate_points[nearest_index][1]), 0, image_shape[0] - 1))

            target_candidates.append((scores[index], center_x, center_y))

        if not target_candidates:
            return None

        _, center_x, center_y = max(target_candidates, key=lambda item: item[0])
        return center_x, center_y


    def select_target_polygon(
        self,
        result,
        image_shape: tuple[int, int],
    ) -> tuple[np.ndarray, tuple[int, int]] | None:
        if result.masks is None or result.boxes is None:
            return None

        classes = result.boxes.cls.cpu().numpy().astype(int)
        scores = result.boxes.conf.cpu().numpy()
        names = result.names
        polygons = result.masks.xy

        candidates = []
        for index, class_id in enumerate(classes):
            if names[class_id] != self.target_class:
                continue

            polygon = polygons[index]
            if polygon is None or len(polygon) < 3:
                continue

            contour = np.round(polygon).astype(np.int32).reshape(-1, 1, 2)
            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue

            center_x = int(round(moments["m10"] / moments["m00"]))
            center_y = int(round(moments["m01"] / moments["m00"]))
            center_x = int(np.clip(center_x, 0, image_shape[1] - 1))
            center_y = int(np.clip(center_y, 0, image_shape[0] - 1))

            inside = cv2.pointPolygonTest(contour, (float(center_x), float(center_y)), False)
            if inside < 0:
                candidate_points = polygon.astype(np.float64)
                distances = np.sum((candidate_points - np.array([center_x, center_y])) ** 2, axis=1)
                nearest_index = int(np.argmin(distances))
                center_x = int(np.clip(round(candidate_points[nearest_index][0]), 0, image_shape[1] - 1))
                center_y = int(np.clip(round(candidate_points[nearest_index][1]), 0, image_shape[0] - 1))

            candidates.append((scores[index], contour, (center_x, center_y)))

        if not candidates:
            return None

        _, contour, center_pixel = max(candidates, key=lambda item: item[0])
        return contour, center_pixel


    def mask_to_point_cloud(
        self,
        mask: np.ndarray,
        color_image: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        # depth_height, depth_width = self.depth_image.shape[:2]

        # if mask.shape != (depth_height, depth_width):
        #     mask = cv2.resize(mask, (depth_width, depth_height), interpolation=cv2.INTER_NEAREST)

        if self.depth_image is None or self.camera_info is None:
            raise RuntimeError("depth image or camera info is not ready")

        mask_indices = np.column_stack(np.where(mask > 0))
        if mask_indices.size == 0:
            raise RuntimeError("target mask is empty")

        sampled_indices = mask_indices[:: self.point_stride]
        pixel_y = sampled_indices[:, 0]
        pixel_x = sampled_indices[:, 1]

        depth_values = self.depth_image[pixel_y, pixel_x]
        if self.depth_image.dtype == np.uint16:
            depth_m = depth_values.astype(np.float32) / 1000.0
        else:
            depth_m = depth_values.astype(np.float32)

        valid = np.isfinite(depth_m) & (depth_m > 0.0) & (depth_m < self.max_depth_m)
        if not np.any(valid):
            raise RuntimeError("no valid depth points inside target mask")

        pixel_x = pixel_x[valid].astype(np.float64)
        pixel_y = pixel_y[valid].astype(np.float64)
        depth_m = depth_m[valid].astype(np.float64)

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        camera_x = (pixel_x - cx) * depth_m / fx
        camera_y = (pixel_y - cy) * depth_m / fy
        points_camera = np.stack([camera_x, camera_y, depth_m], axis=1)

        colors_bgr = color_image[pixel_y.astype(np.int32), pixel_x.astype(np.int32)]
        colors_rgb = colors_bgr[:, ::-1].astype(np.float64) / 255.0
        return points_camera, colors_rgb

    def full_scene_to_point_cloud(self, color_image: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        if self.depth_image is None or self.camera_info is None:
            raise RuntimeError("depth image or camera info is not ready")

        depth_height, depth_width = self.depth_image.shape[:2]
        if color_image.shape[:2] != (depth_height, depth_width):
            color_image = cv2.resize(color_image, (depth_width, depth_height), interpolation=cv2.INTER_LINEAR)

        grid_y, grid_x = np.mgrid[0:depth_height:self.scene_stride, 0:depth_width:self.scene_stride]
        pixel_y = grid_y.reshape(-1)
        pixel_x = grid_x.reshape(-1)

        depth_values = self.depth_image[pixel_y, pixel_x]
        if self.depth_image.dtype == np.uint16:
            depth_m = depth_values.astype(np.float32) / 1000.0
        else:
            depth_m = depth_values.astype(np.float32)

        valid = np.isfinite(depth_m) & (depth_m > 0.0) & (depth_m < self.max_depth_m)
        if not np.any(valid):
            raise RuntimeError("no valid depth points in full scene")

        pixel_x = pixel_x[valid].astype(np.float64)
        pixel_y = pixel_y[valid].astype(np.float64)
        depth_m = depth_m[valid].astype(np.float64)

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        camera_x = (pixel_x - cx) * depth_m / fx
        camera_y = (pixel_y - cy) * depth_m / fy
        points_camera = np.stack([camera_x, camera_y, depth_m], axis=1)

        colors_bgr = color_image[pixel_y.astype(np.int32), pixel_x.astype(np.int32)]
        colors_rgb = colors_bgr[:, ::-1].astype(np.float64) / 255.0
        return points_camera, colors_rgb

    def highlight_object_colors(self, colors_rgb: np.ndarray) -> np.ndarray:
        if colors_rgb.size == 0:
            return colors_rgb

        highlight = np.tile(np.array([[1.0, 0.35, 0.0]], dtype=np.float64), (colors_rgb.shape[0], 1))
        return 0.3 * colors_rgb + 0.7 * highlight
    
    def pixel_to_camera_point(self, pixel_x: int, pixel_y: int) -> np.ndarray:
        if self.depth_image is None or self.camera_info is None:
            raise RuntimeError("depth image or camera info is not ready")

        depth_height, depth_width = self.depth_image.shape[:2]
        if not (0 <= pixel_x < depth_width and 0 <= pixel_y < depth_height):
            raise RuntimeError(f"pixel ({pixel_x}, {pixel_y}) is outside depth image")

        depth_m = self.read_depth_meters(pixel_x, pixel_y)
        if depth_m <= 0.0 or math.isnan(depth_m):
            raise RuntimeError(f"pixel ({pixel_x}, {pixel_y}) has invalid depth")

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        camera_x = (pixel_x - cx) * depth_m / fx
        camera_y = (pixel_y - cy) * depth_m / fy
        return np.array([camera_x, camera_y, depth_m], dtype=np.float64)

    def read_depth_meters(self, pixel_x: int, pixel_y: int) -> float:
        search_radius = 3
        for radius in range(search_radius + 1):
            for offset_y in range(-radius, radius + 1):
                for offset_x in range(-radius, radius + 1):
                    sample_x = pixel_x + offset_x
                    sample_y = pixel_y + offset_y
                    if sample_x < 0 or sample_y < 0:
                        continue
                    if sample_y >= self.depth_image.shape[0] or sample_x >= self.depth_image.shape[1]:
                        continue

                    depth_value = self.depth_image[sample_y, sample_x]
                    if isinstance(depth_value, np.ndarray):
                        depth_value = depth_value.item()

                    if self.depth_image.dtype == np.uint16:
                        depth_m = float(depth_value) / 1000.0
                    else:
                        depth_m = float(depth_value)

                    if depth_m > 0.0 and not math.isnan(depth_m):
                        return depth_m

        return float("nan")

    def update_visualizer(self) -> None:
        with self.data_lock:
            scene_points = self.latest_scene_points_world.copy()
            scene_colors = self.latest_scene_colors.copy()
            object_points = self.latest_object_points_world.copy()
            object_colors = self.latest_object_colors.copy()
            center_world = None if self.latest_center_world is None else self.latest_center_world.copy()

        if scene_points.shape[0] > 0:
            self.scene_cloud.points = o3d.utility.Vector3dVector(scene_points)
            self.scene_cloud.colors = o3d.utility.Vector3dVector(scene_colors)
        else:
            self.scene_cloud.points = o3d.utility.Vector3dVector(np.zeros((0, 3), dtype=np.float64))
            self.scene_cloud.colors = o3d.utility.Vector3dVector(np.zeros((0, 3), dtype=np.float64))

        if object_points.shape[0] > 0:
            self.object_cloud.points = o3d.utility.Vector3dVector(object_points)
            self.object_cloud.colors = o3d.utility.Vector3dVector(object_colors)
        else:
            self.object_cloud.points = o3d.utility.Vector3dVector(np.zeros((0, 3), dtype=np.float64))
            self.object_cloud.colors = o3d.utility.Vector3dVector(np.zeros((0, 3), dtype=np.float64))

        if center_world is not None:
            delta = center_world - self.center_sphere_position
            self.center_sphere.translate(delta, relative=True)
            self.center_sphere_position = center_world
            self.center_sphere_visible = True
        elif self.center_sphere_visible:
            hidden_position = np.array([0.0, 0.0, -10.0], dtype=np.float64)
            delta = hidden_position - self.center_sphere_position
            self.center_sphere.translate(delta, relative=True)
            self.center_sphere_position = hidden_position
            self.center_sphere_visible = False

        self.vis.update_geometry(self.scene_cloud)
        self.vis.update_geometry(self.object_cloud)
        self.vis.update_geometry(self.center_sphere)
        self.vis.poll_events()
        self.vis.update_renderer()

    def initialize_camera_view(self) -> None:
        view_control = self.vis.get_view_control()
        camera_origin = self.t_hip_from_camera[:3, 3]
        camera_forward = self.t_hip_from_camera[:3, :3] @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
        camera_up = self.t_hip_from_camera[:3, :3] @ np.array([0.0, -1.0, 0.0], dtype=np.float64)
        lookat = camera_origin + 0.6 * camera_forward

        view_control.set_lookat(lookat.tolist())
        view_control.set_front((-camera_forward).tolist())
        view_control.set_up(camera_up.tolist())
        view_control.set_zoom(0.7)

    def log_throttled(self, text: str) -> None:
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds > 1e9:
            self.get_logger().info(text)
            self.last_log_time = now

    def destroy_node(self) -> bool:
        if o3d is not None and hasattr(self, "vis"):
            self.vis.destroy_window()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Open3DGraspVizNode()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
"""
可视化点云，相机坐标，机器人坐标，抓取点坐标（debug代码）
"""
