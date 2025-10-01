#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Author: Seungmin Lee
# Date: 2025-09-22
# Description: A unified vision perception node for autonomous driving.
#              This node performs two independent tasks in parallel:
#              1. Traffic light detection using a dedicated USB camera.
#              2. Obstacle (HENES) detection and distance estimation using an Intel RealSense depth camera.
#              3. The traffic light logic now recognizes 'yellow' lights as a non-stop signal.
#              4. Enhanced obstacle distance accuracy by using a median filter on the original depth map ROI.
#              5. Added a grace period after 'left' signal detection to ignore subsequent 'red' signals for robustness.

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import torch
from ultralytics import YOLO
import traceback
from concurrent.futures import ThreadPoolExecutor
import threading

# ROS2 메시지 타입 임포트
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

# 컬러와 뎁스 이미지 동기화를 위한 message_filters 임포트
import message_filters

# TF2 및 3D 좌표 변환을 위한 임포트
import tf2_ros
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point

# PyRealSense 라이브러리 임포트
import pyrealsense2 as rs2
if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2


class VisionPerceptionNode(Node):
    def __init__(self):
        super().__init__('vision_perception_node')
        self.get_logger().info("--- Unified Vision Perception Node v5 (Architected by Hinton) ---")
        
        self.bridge = CvBridge()
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"Using compute device: {self.device}")
        self.use_half = self.device == 'cuda'
        
        self._is_shutting_down = False

        # =====================================================================================
        # 1. 신호등 감지 모듈 (좌회전 강건성 로직 추가)
        # =====================================================================================
        self.get_logger().info("Initializing Traffic Light Detection Module...")
        self.traffic_cam_lock = threading.Lock()
        self.traffic_executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix='traffic_worker')
        
        self.declare_parameter('red_light_min_area', 1)
        self.declare_parameter('traffic_roi_top_ratio', 1.0)
        self.declare_parameter('red_light_confirmation_frames', 7)
        self.declare_parameter('red_light_loss_tolerance_frames', 13)
        self.declare_parameter('red_light_tracking_tolerance', 75)
        # [추가] 좌회전 신호 후 적색 신호를 무시할 프레임 수 파라미터
        self.declare_parameter('left_signal_grace_frames', 4)
        
        self.RED_LIGHT_MIN_AREA = self.get_parameter('red_light_min_area').get_parameter_value().integer_value
        self.TRAFFIC_ROI_TOP_RATIO = self.get_parameter('traffic_roi_top_ratio').get_parameter_value().double_value
        self.RED_LIGHT_CONFIRMATION_FRAMES = self.get_parameter('red_light_confirmation_frames').get_parameter_value().integer_value
        self.RED_LIGHT_LOSS_TOLERANCE_FRAMES = self.get_parameter('red_light_loss_tolerance_frames').get_parameter_value().integer_value
        self.RED_LIGHT_TRACKING_TOLERANCE = self.get_parameter('red_light_tracking_tolerance').get_parameter_value().integer_value
        # [추가] 파라미터 값 읽어오기
        self.LEFT_SIGNAL_GRACE_FRAMES = self.get_parameter('left_signal_grace_frames').get_parameter_value().integer_value

        self.STATE_CLEAR = 0
        self.STATE_CANDIDATE = 1
        self.STATE_CONFIRMED_STOP = 2
        # [수정] 좌회전 유예 카운터(left_grace_counter)를 추적 상태에 추가
        self.stop_signal_tracker = {'state': self.STATE_CLEAR, 'confirmation_counter': 0, 'loss_counter': 0, 'last_center': None, 'left_grace_counter': 0}
        
        self.traffic_detection_model = None
        self.traffic_model_ready = False
        try:
            self.declare_parameter('traffic_model_path', './traffic_yellow.onnx')
            # self.declare_parameter('traffic_model_path', './traffic2.onnx')
            traffic_model_path = self.get_parameter('traffic_model_path').get_parameter_value().string_value
            self.traffic_detection_model = YOLO(traffic_model_path, task='detect')
            self.traffic_model_class_names = ['green', 'left', 'red', 'yellow']
            self.traffic_model_ready = True
            self.get_logger().info("✅ Traffic Light ONNX model loaded successfully.")
            self.get_logger().info(f"Traffic light classes: {self.traffic_model_class_names}")
        except Exception as e:
            self.get_logger().error(f"Failed to load Traffic Light model: {e}. This feature will be disabled.")

        self.traffic_pub = self.create_publisher(Bool, '/traffic_stop', 10)
        self.traffic_cam_viz_pub = self.create_publisher(CompressedImage, '/unified_vision/traffic_cam/viz/compressed', 10)
        self.traffic_cam_sub = self.create_subscription(CompressedImage, 'camera2/image_raw/compressed', self.traffic_cam_callback, 10)
        self.get_logger().info("✅ Traffic Light Detection Module Initialized.")
        
        # =====================================================================================
        # 2. 장애물(HENES) 감지 모듈 (정확도 개선)
        # =====================================================================================
        self.get_logger().info("Initializing Upgraded Obstacle (HENES) Detection Module...")
        self.realsense_lock = threading.Lock()
        self.realsense_executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix='realsense_worker')

        self.declare_parameter('henes_model_path', './HENES2.onnx')
        self.declare_parameter('obstacle_distance_threshold_m', 2.0)
        self.declare_parameter('henes_conf_threshold', 0.6)
        self.declare_parameter('proc_width', 640)
        self.declare_parameter('proc_height', 480)
        self.OBSTACLE_DISTANCE_THRESHOLD = self.get_parameter('obstacle_distance_threshold_m').get_parameter_value().double_value
        self.HENES_CONF_THRESHOLD = self.get_parameter('henes_conf_threshold').get_parameter_value().double_value
        self.proc_width = self.get_parameter('proc_width').get_parameter_value().integer_value
        self.proc_height = self.get_parameter('proc_height').get_parameter_value().integer_value
        self.resized_color_yolo = np.empty((self.proc_height, self.proc_width, 3), dtype=np.uint8)
        self.intrinsics = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.henes_detection_model = None
        self.henes_model_ready = False
        try:
            henes_model_path = self.get_parameter('henes_model_path').get_parameter_value().string_value
            self.henes_detection_model = YOLO(henes_model_path, task='detect')
            self.henes_model_class_names = ['HENES']
            self.henes_model_ready = True
            self.get_logger().info("✅ HENES ONNX model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load HENES model: {e}. This feature will be disabled.")
            
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_existance', 10)
        self.realsense_viz_pub = self.create_publisher(CompressedImage, '/unified_vision/realsense/viz/compressed', 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, "/camera/color/camera_info", self.camera_info_callback, 10)
        self.get_logger().info("Waiting for CameraInfo on '/camera/color/camera_info'...")


    def camera_info_callback(self, info_msg):
        if self.intrinsics is not None:
            return
        try:
            self.get_logger().info("✅ CameraInfo received. Initializing RealSense processing pipeline.")
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = info_msg.width; self.intrinsics.height = info_msg.height
            self.intrinsics.ppx = info_msg.k[2]; self.intrinsics.ppy = info_msg.k[5]
            self.intrinsics.fx = info_msg.k[0]; self.intrinsics.fy = info_msg.k[4]
            self.intrinsics.model = rs2.distortion.brown_conrady if info_msg.distortion_model == 'plumb_bob' else rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = list(info_msg.d)
            self.initialize_image_sync()
            self.destroy_subscription(self.camera_info_sub)
            self.get_logger().info("--- Node initialization complete. Ready for perception tasks. ---")
        except Exception as e:
            self.get_logger().error(f"Failed to process CameraInfo: {e}")

    def initialize_image_sync(self):
        color_sub = message_filters.Subscriber(self, CompressedImage, 'camera/color/image_raw/compressed')
        depth_sub = message_filters.Subscriber(self, Image, 'camera/aligned_depth_to_color/image_raw')
        self.ts = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.realsense_callback)
        self.get_logger().info(f"✅ Subscribing to RealSense topics for obstacle detection.")

    def traffic_cam_callback(self, compressed_msg):
        if self._is_shutting_down: return
        if self.traffic_cam_lock.acquire(blocking=False):
            try:
                self.traffic_executor.submit(self._process_traffic_cam_data, compressed_msg)
            except Exception as e:
                self.get_logger().error(f"Exception during traffic task submission: {e}")
                self.traffic_cam_lock.release()
        else:
            self.get_logger().warn("Dropping a frame from traffic camera, processing is busy.", throttle_duration_sec=1)

    def _process_traffic_cam_data(self, compressed_msg):
        try:
            if not self.traffic_model_ready: return
            np_arr = np.frombuffer(compressed_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None: return

            tracker = self.stop_signal_tracker
            results_traffic = self.traffic_detection_model(cv_image, conf=0.6, iou=0.45, verbose=False, half=self.use_half)
            
            go_signal_found = False
            left_signal_found = False # [추가] 이번 프레임에 좌회전 신호가 있었는지 확인하는 플래그
            best_red_light_candidate_center = None
            
            h, w, _ = cv_image.shape
            for r in results_traffic:
                for box_data in r.boxes.cpu().numpy():
                    label = self.traffic_model_class_names[int(box_data.cls[0])]
                    # [수정] 좌회전 신호를 별도로 처리
                    if label == 'left':
                        go_signal_found = True
                        left_signal_found = True
                    elif label in ['green', 'yellow']:
                        go_signal_found = True
                    
                    if label == 'red' and box_data.conf[0] >= 0.7:
                        box = box_data.xyxy[0].astype(int) 
                        if (box[2] - box[0]) * (box[3] - box[1]) < self.RED_LIGHT_MIN_AREA: continue
                        cy = (box[1] + box[3]) / 2
                        if cy > h * self.TRAFFIC_ROI_TOP_RATIO: continue
                        best_red_light_candidate_center = np.array([(box[0] + box[2]) / 2, cy])
                        # [수정] Red 신호는 여러 개일 수 있으므로, 가장 가능성 높은 하나만 찾고 break하지 않습니다.
                
            red_light_detected_in_frame = False
            if best_red_light_candidate_center is not None:
                if tracker['last_center'] is None or \
                    np.linalg.norm(best_red_light_candidate_center - tracker['last_center']) < self.RED_LIGHT_TRACKING_TOLERANCE:
                    red_light_detected_in_frame = True
                    tracker['last_center'] = best_red_light_candidate_center

            # [추가] 좌회전 유예 로직 관리
            if left_signal_found:
                # 좌회전 신호가 보이면 유예 카운터를 최댓값으로 리셋
                if tracker['left_grace_counter'] == 0:
                    self.get_logger().info(f"Left signal detected! Activating red light grace period for {self.LEFT_SIGNAL_GRACE_FRAMES} frames.")
                tracker['left_grace_counter'] = self.LEFT_SIGNAL_GRACE_FRAMES
            elif tracker['left_grace_counter'] > 0:
                # 좌회전 신호가 안보이면 유예 카운터를 1씩 감소
                tracker['left_grace_counter'] -= 1
                if tracker['left_grace_counter'] == 0:
                    self.get_logger().info("Left signal grace period has ended.")

            is_in_grace_period = tracker['left_grace_counter'] > 0

            # [수정] 상태 결정 로직 수정
            if go_signal_found:
                if tracker['state'] != self.STATE_CLEAR:
                    self.get_logger().info("Go signal (Green/Left/Yellow) detected, clearing stop state.")
                tracker['state'] = self.STATE_CLEAR; tracker['confirmation_counter'] = 0; tracker['loss_counter'] = 0; tracker['last_center'] = None
            else:
                # 적색 신호가 감지되었고, '좌회전 유예 기간이 아닐 때'만 정지 로직을 처리
                if red_light_detected_in_frame and not is_in_grace_period:
                    if tracker['state'] == self.STATE_CLEAR:
                        tracker['state'] = self.STATE_CANDIDATE
                        tracker['confirmation_counter'] = 1
                    elif tracker['state'] == self.STATE_CANDIDATE:
                        tracker['confirmation_counter'] += 1
                        if tracker['confirmation_counter'] >= self.RED_LIGHT_CONFIRMATION_FRAMES:
                            tracker['state'] = self.STATE_CONFIRMED_STOP
                            self.get_logger().info("Stop signal (Red) confirmed on traffic camera.")
                    elif tracker['state'] == self.STATE_CONFIRMED_STOP:
                        tracker['loss_counter'] = 0 # 적색 신호가 계속 보이면 loss_counter 리셋
                # 적색 신호가 감지되었지만, '좌회전 유예 기간일 때'
                elif red_light_detected_in_frame and is_in_grace_period:
                    self.get_logger().info(f"Red light detected but ignored due to left signal grace period. ({tracker['left_grace_counter']} frames left)", throttle_duration_sec=1.0)
                    # 유예 기간 중에는 정지 상태로 들어가지 않음
                    if tracker['state'] != self.STATE_CLEAR:
                         # 만약 이전에 정지였다면, 유예기간동안 clear상태로 변경
                         tracker['state'] = self.STATE_CLEAR; tracker['confirmation_counter'] = 0; tracker['loss_counter'] = 0; tracker['last_center'] = None
                # 적색 신호가 감지되지 않았을 때
                else:
                    if tracker['state'] == self.STATE_CANDIDATE:
                        tracker['state'] = self.STATE_CLEAR; tracker['last_center'] = None
                    elif tracker['state'] == self.STATE_CONFIRMED_STOP:
                        tracker['loss_counter'] += 1
                        if tracker['loss_counter'] >= self.RED_LIGHT_LOSS_TOLERANCE_FRAMES:
                            tracker['state'] = self.STATE_CLEAR; tracker['last_center'] = None
                            self.get_logger().info("Stop signal (Red) lost on traffic camera.")

            msg = Bool(); is_stopped = tracker['state'] == self.STATE_CONFIRMED_STOP; msg.data = is_stopped
            self.traffic_pub.publish(msg)
            
            annotated_image = self.draw_traffic_detections(cv_image, results_traffic, is_stopped)
            # [추가] 유예 기간 시각화
            if is_in_grace_period:
                grace_text = f"LEFT GRACE: {tracker['left_grace_counter']}"
                cv2.putText(annotated_image, grace_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,0), 3)
                cv2.putText(annotated_image, grace_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 165, 0), 2)


            self.publish_compressed_viz(self.traffic_cam_viz_pub, annotated_image)
        except Exception as e:
            self.get_logger().error(f"Error in traffic cam worker: {e}\n{traceback.format_exc()}")
        finally:
            self.traffic_cam_lock.release()

    def realsense_callback(self, color_msg, depth_msg):
        if self.intrinsics is None or self._is_shutting_down: return
        if self.realsense_lock.acquire(blocking=False):
            try:
                self.realsense_executor.submit(self._process_realsense_data, color_msg, depth_msg)
            except Exception as e:
                self.get_logger().error(f"Exception during realsense task submission: {e}")
                self.realsense_lock.release()
        else:
            self.get_logger().warn("Dropping a frame from RealSense camera, processing is busy.", throttle_duration_sec=1)

    def _process_realsense_data(self, color_msg, depth_msg):
        try:
            if not self.henes_model_ready: return
            np_arr = np.frombuffer(color_msg.data, np.uint8)
            cv_color = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            cv_depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
            if cv_color is None or cv_depth_raw is None: return

            cv2.resize(cv_color, (self.proc_width, self.proc_height), dst=self.resized_color_yolo, interpolation=cv2.INTER_AREA)

            results_henes = self.henes_detection_model(self.resized_color_yolo, conf=self.HENES_CONF_THRESHOLD, verbose=False, half=self.use_half)

            obstacle_within_range = False; detected_objects = []; min_distance = float('inf')
            best_box = max(results_henes[0].boxes, key=lambda box: box.conf[0], default=None)

            if best_box is not None and self.henes_model_class_names[int(best_box.cls[0])] == 'HENES':
                rx1, ry1, rx2, ry2 = map(int, best_box.xyxy[0])
                
                orig_h, orig_w, _ = cv_color.shape
                orig_x1 = int(rx1 * orig_w / self.proc_width)
                orig_y1 = int(ry1 * orig_h / self.proc_height)
                orig_x2 = int(rx2 * orig_w / self.proc_width)
                orig_y2 = int(ry2 * orig_h / self.proc_height)
                
                roi_w = orig_x2 - orig_x1
                roi_h = orig_y2 - orig_y1
                roi_cx = orig_x1 + roi_w // 2
                roi_cy = orig_y1 + roi_h // 2
                
                roi_sample_w = max(1, int(roi_w * 0.1))
                roi_sample_h = max(1, int(roi_h * 0.1))

                roi_x_start = max(0, roi_cx - roi_sample_w // 2)
                roi_y_start = max(0, roi_cy - roi_sample_h // 2)
                roi_x_end = min(orig_w, roi_cx + roi_sample_w // 2)
                roi_y_end = min(orig_h, roi_cy + roi_sample_h // 2)

                depth_roi = cv_depth_raw[roi_y_start:roi_y_end, roi_x_start:roi_x_end]
                
                valid_depths = depth_roi[depth_roi > 0]
                
                if valid_depths.size > 0:
                    depth_in_mm = np.median(valid_depths)
                    # 3D 좌표 계산
                    optical_frame_coords = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [roi_cx, roi_cy], depth_in_mm)
                    
                    point_in_optical_frame = PointStamped()
                    point_in_optical_frame.header.frame_id = "camera_color_optical_frame"
                    point_in_optical_frame.header.stamp = depth_msg.header.stamp
                    point_in_optical_frame.point.x = optical_frame_coords[0] / 1000.0
                    point_in_optical_frame.point.y = optical_frame_coords[1] / 1000.0
                    point_in_optical_frame.point.z = optical_frame_coords[2] / 1000.0
                    
                    try:
                        # TF 변환
                        target_frame = "base_link"
                        transform = self.tf_buffer.lookup_transform(target_frame, point_in_optical_frame.header.frame_id, rclpy.time.Time())
                        point_in_target_frame = do_transform_point(point_in_optical_frame, transform)
                        distance_m = np.linalg.norm([point_in_target_frame.point.x, point_in_target_frame.point.y, point_in_target_frame.point.z])
                        
                        if distance_m < min_distance: 
                            min_distance = distance_m
                        
                        detected_objects.append({'box_original': (orig_x1, orig_y1, orig_x2, orig_y2), 'pos_3d': point_in_target_frame.point, 'distance': distance_m, 'conf': best_box.conf[0]})
                    
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                        self.get_logger().warn(f"Coordinate transform failed from '{point_in_optical_frame.header.frame_id}' to '{target_frame}': {e}", throttle_duration_sec=5.0)

            if min_distance <= self.OBSTACLE_DISTANCE_THRESHOLD: 
                obstacle_within_range = True
                
            msg = Bool(); msg.data = obstacle_within_range; self.obstacle_pub.publish(msg)
            
            annotated_image = self.draw_obstacle_detections(cv_color, detected_objects, obstacle_within_range)
            self.publish_compressed_viz(self.realsense_viz_pub, annotated_image)

        except CvBridgeError as e: self.get_logger().error(f"CvBridge Error: {e}")
        except Exception as e: self.get_logger().error(f"Error in RealSense worker: {e}\n{traceback.format_exc()}")
        finally:
            self.realsense_lock.release()

    def publish_compressed_viz(self, publisher, cv_image):
        msg = CompressedImage(format="jpeg"); msg.header.stamp = self.get_clock().now().to_msg()
        success, encoded_image = cv2.imencode('.jpg', cv_image)
        if success: msg.data = encoded_image.tobytes(); publisher.publish(msg)

    def draw_traffic_detections(self, image, results, is_stopped):
        for r in results:
            for box in r.boxes.cpu().numpy():
                cls_id = int(box.cls[0]); x1, y1, x2, y2 = map(int, box.xyxy[0])
                label = self.traffic_model_class_names[cls_id]
                if label == 'red': color = (0, 0, 255)
                elif label == 'green': color = (0, 255, 0)
                elif label == 'left': color = (255, 0, 255)
                elif label == 'yellow': color = (0, 255, 255)
                else: color = (255, 255, 0)
                cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
                cv2.putText(image, f"{label}: {box.conf[0]:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
        status_text = f"TRAFFIC STOP: {is_stopped}"; text_color = (0, 0, 255) if is_stopped else (0, 255, 0)
        cv2.putText(image, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,0), 3)
        cv2.putText(image, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, text_color, 2)
        return image

    def draw_obstacle_detections(self, image, detected_objects, obstacle_within_range):
        for obj in detected_objects:
            x1, y1, x2, y2 = obj['box_original']
            pos = obj['pos_3d']; distance = obj['distance']; conf = obj['conf']
            color = (0, 165, 255) # Orange
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
            label_text1 = f"HENES: {distance:.2f}m ({conf:.2f})"; label_text2 = f"X:{pos.x:.2f} Y:{pos.y:.2f} Z:{pos.z:.2f}"
            cv2.putText(image, label_text1, (x1, y1 - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.putText(image, label_text2, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        status_text = f"OBSTACLE EXIST: {obstacle_within_range}"
        text_color = (0, 0, 255) if obstacle_within_range else (0, 255, 0)
        cv2.putText(image, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,0), 3)
        cv2.putText(image, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, text_color, 2)
        return image

    def destroy_node(self):
        self.get_logger().info("Shutting down the node and thread pools.")
        self._is_shutting_down = True
        self.traffic_executor.shutdown(wait=True)
        self.realsense_executor.shutdown(wait=True)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VisionPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()