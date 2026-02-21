#!/usr/bin/env python3

from ultralytics import YOLO
import os
import copy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()

def _find_model():
    # 1) 环境变量
    env_path = os.environ.get("YOLO_MODEL_PATH")
    if env_path and os.path.isfile(env_path):
        return env_path
    # 2) 本包目录（src 或 install 下的 share）
    pkg_dir = os.path.dirname(os.path.abspath(__file__))
    for candidate in [
        os.path.join(pkg_dir, "best.pt"),
        os.path.join(pkg_dir, "weights", "best.pt"),
        os.path.join(os.path.dirname(pkg_dir), "best.pt"),
    ]:
        if os.path.isfile(candidate):
            return candidate
    # 3) 原仓库路径（兼容旧环境）
    legacy = os.path.join(os.environ.get("HOME", ""), "Isaac_Project/pickPlaceChatMoveitBot_ws/src/yolov8obb_object_detection/yolov8obb_object_detection/best.pt")
    if os.path.isfile(legacy):
        return legacy
    # 4) 无权重时用预训练模型，仅做占位（OBB 需自行训练 best.pt）
    return "yolov8n.pt"

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        model_path = _find_model()
        if "yolov8n" in model_path:
            self.get_logger().warn("未找到 best.pt，使用 yolov8n.pt。请将训练好的 best.pt 放到本包目录或设置 YOLO_MODEL_PATH。")
        self.model = YOLO(model_path)

        self.yolov8_inference = Yolov8Inference()

        self.subscription = self.create_subscription(
            Image,
            '/rgb',
            self.camera_callback,
            10)
        self.subscription 

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)

    def camera_callback(self, data):
        img = bridge.imgmsg_to_cv2(data, "bgr8")
        results = self.model(img, conf=0.85)

        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()  # ✅ fixed

        for r in results:
            if r.obb is not None:
                boxes = r.obb
                for box in boxes:
                    self.inference_result = InferenceResult()
                    b = box.xyxyxyxy[0].to('cpu').detach().numpy().copy()
                    c = box.cls
                    self.inference_result.class_name = self.model.names[int(c)]
                    a = b.reshape(1, 8)
                    self.inference_result.coordinates = copy.copy(a[0].tolist())
                    self.yolov8_inference.yolov8_inference.append(self.inference_result)
            else:
                self.get_logger().info("no_results")  # ✅ fixed

        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)
        self.img_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()