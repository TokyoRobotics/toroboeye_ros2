#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from argparse import Namespace
from turtle import color
from toroboeye_msgs.srv import Connect
from toroboeye_msgs.srv import Disconnect
from toroboeye_msgs.srv import Get
from toroboeye_msgs.srv import GetOne
from toroboeye_msgs.srv import Set
from toroboeye_msgs.srv import Write
from toroboeye_msgs.srv import Activate
from toroboeye_msgs.srv import Deactivate
from toroboeye_msgs.srv import Capture
from toroboeye_msgs.srv import WaitForState
from toroboeye_msgs.srv import WaitForActive
from toroboeye_msgs.srv import WaitForInactive
from toroboeye_msgs.srv import WaitForFrame
from toroboeye_msgs.srv import Stop
from toroboeye_msgs.srv import GetIntrinsics

from toroboeye_msgs.msg import Device
from sensor_msgs.msg    import Image, CameraInfo, PointCloud2, PointField

import std_msgs.msg as std_msgs

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from pytoroboeye import toroboeye
from cv_bridge import CvBridge

import numpy as np
import open3d as o3d
import cv2

import copy

class Server(Node):

    def __init__(self):
        # Two options must be True
        super().__init__('toroboeye_camera_node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self._tc                        = toroboeye.Camera()
        self._bridge                    = CvBridge()
        self._pub_color                 = self.create_publisher(Image, 'camera/color/image', 10)
        self._pub_depth                 = self.create_publisher(Image, 'camera/depth/image', 10)
        self._pub_caminfo               = self.create_publisher(CameraInfo, 'camera/camera_info', 10)
        self._pub_pcd                   = self.create_publisher(PointCloud2, 'camera/point_cloud', 10)
        self._intrinsics                = None

        self._service_connect           = self.create_service(Connect, 'connect', self._connect_callback)
        self._service_disconnect        = self.create_service(Disconnect, 'disconnect', self._disconnect_callback)
        self._service_get               = self.create_service(Get, 'get', self._get_callback)
        self._service_get_one           = self.create_service(GetOne, 'get_one', self._get_one_callback)
        self._service_set               = self.create_service(Set, 'set', self._set_callback)
        self._service_write             = self.create_service(Write, 'write', self._write_callback)
        self._service_activate          = self.create_service(Activate, 'activate', self._activate_callback)
        self._service_deactivate        = self.create_service(Deactivate, 'deactivate', self._deactivate_callback)
        self._service_capture           = self.create_service(Capture, 'capture', self._capture_callback)
        self._service_wait_for_state    = self.create_service(WaitForState, 'wait_for_state', self._wait_for_state_callback)
        self._service_wait_for_active   = self.create_service(WaitForActive, 'wait_for_active', self._wait_for_active_callback)
        self._service_wait_for_inactive = self.create_service(WaitForInactive, 'wait_for_inactive', self._wait_for_inactive_callback)
        self._service_wait_for_frame    = self.create_service(WaitForFrame, 'wait_for_frame', self._wait_for_frame_callback)
        self._service_stop              = self.create_service(Stop, 'stop', self._stop_callback)
        self._service_get_intrinsics    = self.create_service(GetIntrinsics, 'get_intrinsics', self._get_intrinsics_callback)
        # self._service_                  = self.create_service(, '', self.__callback)

        #### Capture Setting value ####
        self._device_illuminant_power           = self.get_parameter("device_illuminant_power").get_parameter_value().integer_value
        self._depth_illuminant_color            = self.get_parameter("depth_illuminant_color").get_parameter_value().integer_value
        self._depth_coding_pattern              = self.get_parameter("depth_coding_pattern").get_parameter_value().integer_value
        self._depth_accuracy                    = self.get_parameter("depth_accuracy").get_parameter_value().integer_value
        self._depth_exposure_time               = self.get_parameter("depth_exposure_time").get_parameter_value().integer_value
        self._depth_adequate_score              = self.get_parameter("depth_adequate_score").get_parameter_value().integer_value
        self._color_strobe_intensity            = self.get_parameter("color_strobe_intensity").get_parameter_value().integer_value
        self._color_exposure_time               = self.get_parameter("color_exposure_time").get_parameter_value().integer_value
        self._data                              = self.get_parameter("data").get_parameter_value().string_value
        self._success                           = self.get_parameter("success").get_parameter_value().bool_value
        self._message                           = self.get_parameter("message").get_parameter_value().string_value

        #### Depth Filter value ####
        self._adjustment_depth_scale      = self.get_parameter("adjustment_depth_scale").get_parameter_value().integer_value
        self._upper_base_limit            = self.get_parameter("upper_base_limit").get_parameter_value().integer_value
        self._lower_amplitude_limit       = self.get_parameter("lower_amplitude_limit").get_parameter_value().integer_value
        self._gradient_threshold          = self.get_parameter("gradient_threshold").get_parameter_value().integer_value
        self._edge_dilation_value         = self.get_parameter("edge_dilation_value").get_parameter_value().integer_value
        self._min_area_x                  = self.get_parameter("min_area_x").get_parameter_value().integer_value
        self._min_area_y                  = self.get_parameter("min_area_y").get_parameter_value().integer_value
        self._max_area_x                  = self.get_parameter("max_area_x").get_parameter_value().integer_value
        self._max_area_y                  = self.get_parameter("max_area_y").get_parameter_value().integer_value
        self._depth_range_min             = self.get_parameter("depth_range_min").get_parameter_value().integer_value
        self._depth_range_max             = self.get_parameter("depth_range_max").get_parameter_value().integer_value

        self._use_depth_median_filter     = self.get_parameter("use_depth_median_filter").get_parameter_value().bool_value
        self._depth_median_filter_kernel_size = self.get_parameter("depth_median_filter_kernel_size").get_parameter_value().integer_value
        self._use_depth_bilateral_filter = self.get_parameter("use_depth_bilateral_filter").get_parameter_value().bool_value
        self._depth_bilateral_filter_diameter = self.get_parameter("depth_bilateral_filter_diameter").get_parameter_value().integer_value
        self._depth_bilateral_filter_sigma_depth = self.get_parameter("depth_bilateral_filter_sigma_depth").get_parameter_value().double_value
        self._depth_bilateral_filter_sigma_space = self.get_parameter("depth_bilateral_filter_sigma_space").get_parameter_value().double_value

        #### Color Filter Value ####
        self._gain_red                    = self.get_parameter("colorbalance_gain_red").get_parameter_value().double_value
        self._gain_green                  = self.get_parameter("colorbalance_gain_green").get_parameter_value().double_value
        self._gain_blue                   = self.get_parameter("colorbalance_gain_blue").get_parameter_value().double_value
        self._input_range_min             = self.get_parameter("input_level_dark").get_parameter_value().integer_value
        self._input_range_max             = self.get_parameter("input_level_light").get_parameter_value().integer_value
        self._output_range_min            = self.get_parameter("output_level_dark").get_parameter_value().integer_value
        self._output_range_max            = self.get_parameter("output_level_light").get_parameter_value().integer_value
        self._brightness_gamma            = self.get_parameter("brightness_gamma").get_parameter_value().double_value
        self._sharpness_level             = self.get_parameter("sharpness_level").get_parameter_value().double_value

    def _connect_callback(self, req, res):
        res.success = True
        res.message = ''
        try:
            self._tc.connect(ip = req.ip, sync_time = req.sync_time)
        except Exception as e:
            self.get_logger().info(str(e))
            res.success = False
            res.message = str(e)
        return res

    def _disconnect_callback(self, req, res):
        res.success = True
        res.message = ''
        try:
            self._tc.disconnect()
        except Exception as e:
            self.get_logger().info(str(e))
            res.success = False
            res.message = str(e)
        return res

    def _get_callback(self, req, res):
        res.success = True
        res.message = ''
        try:
            res.device_illuminant_power = self._tc.get(toroboeye.Setting.DEVICE.ILLUMINANT_POWER.ID)
            res.depth_illuminant_color  = self._tc.get(toroboeye.Setting.DEPTH.ILLUMINANT_COLOR.ID)
            res.depth_coding_pattern    = self._tc.get(toroboeye.Setting.DEPTH.CODING_PATTERN.ID)
            res.depth_accuracy          = self._tc.get(toroboeye.Setting.DEPTH.ACCURACY.ID)
            res.depth_exposure_time     = self._tc.get(toroboeye.Setting.DEPTH.EXPOSURE_TIME.ID)
            res.color_strobe_intensity  = self._tc.get(toroboeye.Setting.COLOR.STROBE_INTENSITY.ID)
            res.color_exposure_time     = self._tc.get(toroboeye.Setting.COLOR.EXPOSURE_TIME.ID)
            #add
            res.device_configuration    = Device() 
            dc                          = self._tc.get(toroboeye.Info.DEVICE.CONFIGURATION.ID)
            res.device_configuration.camera_product_name          = dc["camera_product_name"]
            res.device_configuration.camera_serial_number         = dc["camera_serial_number"]
            res.device_configuration.controller_serial_number     = dc["controller_serial_number"]
            res.device_configuration.controller_firmware_version  = dc["controller_firmware_version"]
            res.device_configuration.api_version                  = dc["api_version"]
            res.state_activation        = self._tc.get(toroboeye.State.ACTIVATION.ID)
            res.state_processing        = self._tc.get(toroboeye.State.PROCESSING.ID)

        except Exception as e:
            res.success = False
            res.message = str(e)
            self.get_logger().info(str(e))
        return res

    def _get_one_callback(self, req, res):
        res.success = True
        res.message = ''
        try:
            res.state = self._tc.get(id)
        except Exception as e:
            res.success = False
            res.message = str(e)
            self.get_logger().info(str(e))
        return res

    def _set_callback(self, req, res):
        res.success = True
        res.message = ''
        try:
            if self._device_illuminant_power != None:
                self._tc.set(toroboeye.Setting.DEVICE.ILLUMINANT_POWER.ID    , self._device_illuminant_power      , req.data)
            else:
                self._tc.set(toroboeye.Setting.DEVICE.ILLUMINANT_POWER.ID    , req.device_illuminant_power      , req.data)

            if self._device_illuminant_power != None:
                self._tc.set(toroboeye.Setting.DEPTH.ILLUMINANT_COLOR.ID     , self._depth_illuminant_color       , req.data)
            else:
                self._tc.set(toroboeye.Setting.DEPTH.ILLUMINANT_COLOR.ID     , req.depth_illuminant_color       , req.data)

            if self._device_illuminant_power != None:
                self._tc.set(toroboeye.Setting.DEPTH.CODING_PATTERN.ID       , self._depth_coding_pattern         , req.data)
            else:
                self._tc.set(toroboeye.Setting.DEPTH.CODING_PATTERN.ID       , req.depth_coding_pattern         , req.data)

            if self._device_illuminant_power != None:
                self._tc.set(toroboeye.Setting.DEPTH.ACCURACY.ID             , self._depth_accuracy               , req.data)
            else:
                self._tc.set(toroboeye.Setting.DEPTH.ACCURACY.ID             , req.depth_accuracy               , req.data)

            if self._device_illuminant_power != None:
                self._tc.set(toroboeye.Setting.DEPTH.EXPOSURE_TIME.ID        , self._depth_exposure_time          , req.data)
            else:
                self._tc.set(toroboeye.Setting.DEPTH.EXPOSURE_TIME.ID        , req.depth_exposure_time          , req.data)

            if self._device_illuminant_power != None:
                self._tc.set(toroboeye.Setting.DEPTH.ADEQUATE_SCORE.ID       , self._depth_adequate_score         , req.data)
            else:
                self._tc.set(toroboeye.Setting.DEPTH.ADEQUATE_SCORE.ID       , req.depth_adequate_score         , req.data)

            if self._device_illuminant_power != None:
                self._tc.set(toroboeye.Setting.COLOR.STROBE_INTENSITY.ID     , self._color_strobe_intensity       , req.data)
            else:
                self._tc.set(toroboeye.Setting.COLOR.STROBE_INTENSITY.ID     , req.color_strobe_intensity       , req.data)

            if self._device_illuminant_power != None:
                self._tc.set(toroboeye.Setting.COLOR.EXPOSURE_TIME.ID        , self._color_exposure_time          , req.data)
            else:
                self._tc.set(toroboeye.Setting.COLOR.EXPOSURE_TIME.ID        , req.color_exposure_time          , req.data)
            
            self.get_logger().info("[success : set]")
        except Exception as e:
            res.success = False
            res.message = str(e)
            self.get_logger().info(str(e))
        return res

    def _write_callback(self, req, res):
        res.success = True
        res.message = ''
        try:
            self._tc.write()
        except Exception as e:
            res.success = False
            res.message = str(e)
            self.get_logger().info(str(e))
        return res

    def _activate_callback(self, req, res):
        res.success = True
        res.message = ''
        try:
            self._tc.activate()
            self.get_logger().info("[success : activate 1]")
        except Exception as e:
            res.success = False
            res.message = str(e)
            self.get_logger().info(str(e))
        return res

    def _deactivate_callback(self, req, res):
        res.success = True
        res.message = ''
        try:
            self._tc.deactivate()
            self.get_logger().info("[success : deactivate]")
        except Exception as e:
            res.success = False
            res.message = str(e)
            self.get_logger().info(str(e))
        return res

    def _capture_callback(self, req, res):
        res.success = True
        res.message = ''
        try:
            self._tc.capture(oneshot = req.oneshot)
            self.get_logger().info("[success : capture 1]")
        except Exception as e:
            res.success = False
            res.message = str(e)
            self.get_logger().info(str(e))
        return res

    def _wait_for_state_callback(self, req, res):
        res.success = True
        res.message = ''
        try:
            self._tc.wait_for_state(req.activation, req.processing, req.timeout, callback = None)
        except Exception as e:
            res.success = False
            res.message = str(e)
            self.get_logger().info(str(e))
        return res

    def _wait_for_active_callback(self, req, res):
        res.success = True
        res.message = ''
        try:
            self._tc.wait_for_active(callback=None)
            self.get_logger().info("[success : activate 2]")
        except Exception as e:
            res.success = False
            res.message = str(e)
            self.get_logger().info(str(e))
        return res

    def _wait_for_inactive_callback(self, req, res):
        res.success = True
        res.message = ''
        try:
            self._tc.wait_for_inactive(callback=None)
            self.get_logger().info("[success : deactivate 2]")
        except Exception as e:
            res.success = False
            res.message = str(e)
            self.get_logger().info(str(e))
        return res

    def __make_pcd_np(self, rgb, depth):
        tmp = self._tc.get_intrinsics()
        points = np.zeros((int(tmp.height) * int(tmp.width), 3), dtype=np.float32)
        xx, yy = np.meshgrid(np.arange(tmp.width), np.arange(tmp.height))
        xx = xx.reshape(-1)
        yy = yy.reshape(-1)
        points[:, 2] = depth.reshape(-1)
        points[:, 0] = points[:, 2] * (xx - tmp.cx) / tmp.fx
        points[:, 1] = points[:, 2] * (yy - tmp.cy) / tmp.fy
        color = np.zeros((int(tmp.height) * int(tmp.width), 4), dtype=np.uint8)
        color[:, :3] = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB).reshape(-1, 3).astype(np.uint8)
        color[:, 3] = 255
        color = color.view("uint32")
        return points, color

    def __make_pcd_o3d(self, rgb, depth):
        # convert color (BGR -> RGB) first
        o3d_rgb = o3d.geometry.Image(cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB))
        o3d_depth = o3d.geometry.Image(depth)
        # default: convert_rgb_to_intensity = True
        rgbd_img = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_rgb, o3d_depth, convert_rgb_to_intensity=False)

        tmp = self._tc.get_intrinsics()
        intrinsic = o3d.camera.PinholeCameraIntrinsic(int(tmp.width), int(tmp.height), tmp.fx, tmp.fy, tmp.cx, tmp.cy)
        camera = o3d.camera.PinholeCameraParameters()
        camera.intrinsic = intrinsic
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
                rgbd_img,
                camera.intrinsic
        )
        points = np.asarray(pcd.points).astype(np.float32)
        color = np.zeros((points.shape[0], 4), dtype=np.uint8)
        color[:, :3] = (np.asarray(pcd.colors) * 255).astype(np.uint8)
        # alpha = 255
        color[:, 3] = 255
        color = color.view("uint32")

        return points, color

    def __make_PointCloud2(self, rgb, depth):
        # points, color = self.__make_pcd_o3d(rgb, depth)
        points, color = self.__make_pcd_np(rgb, depth)
        
        # https://blog.csdn.net/huyaoyu/article/details/103193523
        data = np.zeros(
            (points.shape[0], 1),
            dtype={
                "names": ("x", "y", "z", "rgba"),
                "formats": ("f4", "f4", "f4", "u4")
            }
        )
        data["x"] = points[:, 0].reshape((-1, 1))
        data["y"] = points[:, 1].reshape((-1, 1))
        data["z"] = points[:, 2].reshape((-1, 1))
        data["rgba"] = color

        # PointCloud2 data
        msg = PointCloud2()
        msg.header = std_msgs.Header()
        msg.height = 1
        msg.width = points.shape[0]
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * points.shape[0]
        msg.is_dense = bool(np.isfinite(points).all())
        msg.data = data.tobytes()

        return msg

    def __process_frame(self, frame):

        color_image = copy.deepcopy(frame.color_image)
        depth_image = copy.deepcopy(frame.depth_image)
        score_image = copy.deepcopy(frame.score_image)

        color_image = toroboeye.utility.color.image_enhance(color_image,
                                                            colorbalance_gain=[self._gain_red, self._gain_green, self._gain_blue],
                                                            contrast_range=[self._input_range_min, self._input_range_max, self._output_range_min, self._output_range_max],
                                                            brightness_gamma=self._brightness_gamma,
                                                            sharpness_level=self._sharpness_level)

        #### cut reflection ####
        depth_image = toroboeye.utility.depth.cutoff_by_reflection_properties(depth_image, score_image, amplitude_limit=self._upper_base_limit, base_limit=self._lower_amplitude_limit)

        #### cut noise by depth gradient ####
        depth_image = toroboeye.utility.depth.cutoff_by_depth_gradient(depth_image, gradient_threshold=self._gradient_threshold, edge_dilation=self._edge_dilation_value)

        #### crop captured area #### 
        depth_image = toroboeye.utility.depth.crop_area(depth_image, self._min_area_x, self._min_area_y, self._max_area_x, self._max_area_y)

        #### median filter ####
        if self._use_depth_median_filter:
            kernel_size = self._depth_median_filter_kernel_size
            depth_image = toroboeye.utility.depth.median_filter(depth_image, kernel_size=kernel_size)

        #### bilateral filter ####
        if self._use_depth_bilateral_filter:
            diameter    = self._depth_bilateral_filter_diameter
            sigma_depth = self._depth_bilateral_filter_sigma_depth
            sigma_space = self._depth_bilateral_filter_sigma_space
            depth_image = toroboeye.utility.depth.bilateral_filter(depth_image, diameter=diameter, sigma_depth=sigma_depth, sigma_space=sigma_space)

        #### cut off depth calue by specific range ####
        depth_image = toroboeye.utility.depth.cutoff_out_of_depth_range(depth_image, self._depth_range_min, self._depth_range_max)  

        depth_image /= self._adjustment_depth_scale
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        
        pcd = self.__make_PointCloud2(color_image, depth_image)

        depth_image = self._bridge.cv2_to_imgmsg(depth_image, "32FC1")        
        color_image = self._bridge.cv2_to_imgmsg(color_image, "rgb8")
        score_image = self._bridge.cv2_to_imgmsg(score_image, "rgb8")

        depth_image.header = std_msgs.Header()
        color_image.header = std_msgs.Header()

        return color_image, depth_image, pcd

    def _wait_for_frame_callback(self, req, res):
        res.success = True
        res.message = ''
        try:
            frame = self._tc.wait_for_frame(timeout=req.timeout)
            if frame.timestamp is None:
                res.success = False
                res.message = "Fail to capture"
            else:
                self.get_logger().info("[success : capture 2]")
        except Exception as e:
            res.success = False
            res.message = str(e)
            self.get_logger().info(str(e))
        else:
            res.color_image, res.depth_image, self.pcd = self.__process_frame(frame)
            res.timestamp = int(frame.timestamp)

            # for image_depth_proc to work
            res.color_image .header.frame_id = "toroboeye/camera_optical_frame"
            res.depth_image .header.frame_id = "toroboeye/camera_optical_frame"
            self._intrinsics.header.frame_id = "toroboeye/camera_optical_frame"
            self.pcd        .header.frame_id = "toroboeye/camera_optical_frame"

            # for rviz
            self._pub_color.publish(res.color_image)
            self._pub_depth.publish(res.depth_image)
            self._pub_caminfo.publish(self._intrinsics)
            self._pub_pcd.publish(self.pcd)
        return res

    def _stop_callback(self, req, res):
        res.success = True
        res.message = ''
        try:
            self._tc.stop()
        except Exception as e:
            res.success = False
            res.message = str(e)
            self.get_logger().info(str(e))
        return res

    def _get_intrinsics_callback(self, req, res):
        res.success = True
        res.message = ''
        try:
            tmp = self._tc.get_intrinsics()
            fx = tmp.fx
            fy = tmp.fy
            cx = tmp.cx
            cy = tmp.cy

            res.intrinsics.height               = int(tmp.height)
            res.intrinsics.width                = int(tmp.width)
            res.intrinsics.distortion_model     = tmp.model
            res.intrinsics.d                    = tmp.dist_coeffs
            res.intrinsics.k                    = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            res.intrinsics.r                    = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0] #eye(3)
            res.intrinsics.p                    = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0] #3*4

            self._intrinsics = res.intrinsics

        except Exception as e:
            res.success = False
            res.message = str(e)
            self.get_logger().info(str(e))
        return res


class Client:

    def __init__(self, node):
        self._node = node

    def wait_for_service(self):
        self._node.get_logger().info("[torobo_eye client] wait_for_service")

        timeout = 1.0
        client_connect                      = self._node.create_client(Connect, 'connect')
        client_disconnect                   = self._node.create_client(Disconnect, 'disconnect')
        client_get                          = self._node.create_client(Get, 'get')
        client_get_one                      = self._node.create_client(GetOne, 'get_one')
        client_set                          = self._node.create_client(Set, 'set')
        client_write                        = self._node.create_client(Write, 'write')
        client_activate                     = self._node.create_client(Activate, 'activate')
        client_deactivate                   = self._node.create_client(Deactivate, 'deactivate')
        client_capture                      = self._node.create_client(Capture, 'capture')
        client_wait_for_state               = self._node.create_client(WaitForState, 'wait_for_state')
        client_wait_for_active              = self._node.create_client(WaitForActive, 'wait_for_active')
        client_wait_for_inactive            = self._node.create_client(WaitForInactive, 'wait_for_inactive')
        client_wait_for_frame               = self._node.create_client(WaitForFrame, 'wait_for_frame')
        client_stop                         = self._node.create_client(Stop, 'stop')
        client_get_intrinsics               = self._node.create_client(GetIntrinsics, 'get_intrinsics')

        while not client_connect.wait_for_service(timeout_sec = timeout):
            self._node.get_logger().info("service not abailable, waiting again...")
        while not client_disconnect.wait_for_service(timeout_sec = timeout):
            self._node.get_logger().info("service not abailable, waiting again...")
        # while not client_get.wait_for_service(timeout_sec = timeout):
            self._node.get_logger().info("service not abailable, waiting again...")
        while not client_set.wait_for_service(timeout_sec = timeout):
            self._node.get_logger().info("service not abailable, waiting again...")
        while not client_write.wait_for_service(timeout_sec = timeout):
            self._node.get_logger().info("service not abailable, waiting again...")
        while not client_activate.wait_for_service(timeout_sec = timeout):
            self._node.get_logger().info("service not abailable, waiting again...")
        while not client_deactivate.wait_for_service(timeout_sec = timeout):
            self._node.get_logger().info("service not abailable, waiting again...")
        while not client_capture.wait_for_service(timeout_sec = timeout):
            self._node.get_logger().info("service not abailable, waiting again...")
        while not client_wait_for_state.wait_for_service(timeout_sec = timeout):
            self._node.get_logger().info("service not abailable, waiting again...")
        while not client_wait_for_active.wait_for_service(timeout_sec = timeout):
            self._node.get_logger().info("service not abailable, waiting again...")
        while not client_wait_for_inactive.wait_for_service(timeout_sec = timeout):
            self._node.get_logger().info("service not abailable, waiting again...")
        while not client_wait_for_frame.wait_for_service(timeout_sec = timeout):
            self._node.get_logger().info("service not abailable, waiting again...")
        while not client_stop.wait_for_service(timeout_sec = timeout):
            self._node.get_logger().info("service not abailable, waiting again...")
        while not client_get_intrinsics.wait_for_service(timeout_sec = timeout):
            self._node.get_logger().info("service not abailable, waiting again...")


    def connect(self, ip):
        self._node.get_logger().info("[torobo_eye client] connect")

        client_connect           = self._node.create_client(Connect, 'connect')
        req = Connect.Request()
        req.ip = ip
        req.sync_time = True
        future = client_connect.call_async(req)

    #    rclpy.spin_until_future_complete(self._node, future)
        rclpy.spin_until_future_complete(self._node, future)
        if future.done():
            try:
                result = future.result()
            except Exception as e:
                self._node.get_logger().info('Service call failed %r' % (e,))
                raise
            else:
                if result.success:
                    self._node.get_logger().info('success')
                else:
                    self._node.get_logger().info(result.message)
                    raise Exception(result.message)
        else:
            raise Exception("service is not completed")


    def disconnect(self):
        self._node.get_logger().info("[torobo_eye client] disconnect")

        client = self._node.create_client(Disconnect, 'disconnect')
        req = Disconnect.Request()
        future = client.call_async(req)

        rclpy.spin_until_future_complete(self._node, future)
        if future.done():
            try:
                result = future.result()
            except Exception as e:
                self._node.get_logger().info('Service call failed %r' % (e,))
                raise
            else:
                if result.success:
                    self._node.get_logger().info('success')
                else:
                    self._node.get_logger().info(result.message)
                    raise Exception(result.message)
        else:
            raise Exception("service is not completed")


    def get(self):
        self._node.get_logger().info("[torobo_eye client] get")

        client = self._node.create_client(Get, 'get')
        req = Get.Request()
        future = client.call_async(req)

        rclpy.spin_until_future_complete(self._node, future)
        if future.done():
            try:
                result = future.result()
            except Exception as e:
                self._node.get_logger().info('Service call failed %r' % (e,))
                raise
            else:
                if result.success:
                    self._node.get_logger().info('success')
                else:
                    self._node.get_logger().info(result.message)
                    raise Exception(result.message)
        else:
            raise Exception("service is not completed")


    def get_one(self, id):
        self._node.get_logger().info("[torobo_eye client] get_one")

        client = self._node.create_client(GetOne, 'get_one')
        req = GetOne.Request()
        req.id = id
        future = client.call_async(req)

        rclpy.spin_until_future_complete(self._node, future)
        if future.done():
            try:
                result = future.result()
            except Exception as e:
                self._node.get_logger().info('Service call failed %r' % (e,))
                raise
            else:
                if result.success:
                    self._node.get_logger().info('success')
                else:
                    self._node.get_logger().info(result.message)
                    raise Exception(result.message)
        else:
            raise Exception("service is not completed")


    def set(self,
            device_illuminant_power         = 8,
            depth_illuminant_color          = toroboeye.Setting.DEPTH.ILLUMINANT_COLOR.RED,
            depth_coding_pattern            = toroboeye.Setting.DEPTH.CODING_PATTERN.GRAYCODE_BASE,
            depth_accuracy                  = 2,
            depth_exposure_time             = 1,
            depth_adequate_score            = 40,
            color_strobe_intensity          = 4,
            color_exposure_time             = 1
            ):
        self._node.get_logger().info("[torobo_eye client] set")

        client = self._node.create_client(Set, 'set')

        req = Set.Request()
        req.device_illuminant_power         = device_illuminant_power 
        req.depth_illuminant_color          = depth_illuminant_color  
        req.depth_coding_pattern            = depth_coding_pattern    
        req.depth_accuracy                  = depth_accuracy          
        req.depth_exposure_time             = depth_exposure_time
        req.depth_adequate_score            = depth_adequate_score
        req.color_strobe_intensity          = color_strobe_intensity  
        req.color_exposure_time             = color_exposure_time
        req.data                            = ''

        future = client.call_async(req)

        rclpy.spin_until_future_complete(self._node, future)
        if future.done():
            try:
                result = future.result()
            except Exception as e:
                self._node.get_logger().info('Service call failed %r' % (e,))
                raise
            else:
                if result.success:
                    self._node.get_logger().info('success')
                else:
                    self._node.get_logger().info(result.message)
                    raise Exception(result.message)
        else:
            raise Exception("service is not completed")


    def write(self):
        self._node.get_logger().info("[torobo_eye client] write")

        client = self._node.create_client(Write, 'write')
        req = Write.Request()
        future = client.call_async(req)

        rclpy.spin_until_future_complete(self._node, future)
        if future.done():
            try:
                result = future.result()
            except Exception as e:
                self._node.get_logger().info('Service call failed %r' % (e,))
                raise
            else:
                if result.success:
                    self._node.get_logger().info('success')
                else:
                    self._node.get_logger().info(result.message)
                    raise Exception(result.message)
        else:
            raise Exception("service is not completed")


    def activate(self):
        self._node.get_logger().info("[torobo_eye client] activate")

        client = self._node.create_client(Activate, 'activate')
        req = Activate.Request()
        future = client.call_async(req)

        rclpy.spin_until_future_complete(self._node, future)
        if future.done():
            try:
                result = future.result()
            except Exception as e:
                self._node.get_logger().info('Service call failed %r' % (e,))
                raise
            else:
                if result.success:
                    self._node.get_logger().info('success')
                else:
                    self._node.get_logger().info(result.message)
                    raise Exception(result.message)
        else:
            raise Exception("service is not completed")


    def deactivate(self):
        self._node.get_logger().info("[torobo_eye client] deactivate")

        client = self._node.create_client(Deactivate, 'deactivate')
        req = Deactivate.Request()
        future = client.call_async(req)

        rclpy.spin_until_future_complete(self._node, future)
        if future.done():
            try:
                result = future.result()
            except Exception as e:
                self._node.get_logger().info('Service call failed %r' % (e,))
                raise
            else:
                if result.success:
                    self._node.get_logger().info('success')
                else:
                    self._node.get_logger().info(result.message)
                    raise Exception(result.message)
        else:
            raise Exception("service is not completed")


    def capture(self, oneshot = True):
        self._node.get_logger().info("[torobo_eye client] capture")

        client = self._node.create_client(Capture, 'capture')
        req = Capture.Request()
        req.oneshot = oneshot
        future = client.call_async(req)

        rclpy.spin_until_future_complete(self._node, future)
        if future.done():
            try:
                result = future.result()
            except Exception as e:
                self._node.get_logger().info('Service call failed %r' % (e,))
                raise
            else:
                if result.success:
                    self._node.get_logger().info('success')
                else:
                    self._node.get_logger().info(result.message)
                    raise Exception(result.message)
        else:
            raise Exception("service is not completed")


    def wait_for_state(self, activation, processing, timeout = None):
        self._node.get_logger().info("[torobo_eye client] wait_for_state")

        client = self._node.create_client(WaitForState, 'wait_for_state')
        req = WaitForState.Request()
        req.activation = activation
        req.processing = processing
        #TODO
        #req.timeout = timeout
        future = client.call_async(req)

        rclpy.spin_until_future_complete(self._node, future)
        if future.done():
            try:
                result = future.result()
            except Exception as e:
                self._node.get_logger().info('Service call failed %r' % (e,))
                raise
            else:
                if result.success:
                    self._node.get_logger().info('success')
                else:
                    self._node.get_logger().info(result.message)
                    raise Exception(result.message)
        else:
            raise Exception("service is not completed")


    def wait_for_active(self, timeout=None):
        self._node.get_logger().info("[torobo_eye client] wait_for_active")

        client = self._node.create_client(WaitForActive, 'wait_for_active')
        req = WaitForActive.Request()
        #TODO
        #req.timeout = timeout
        #TODO
        future = client.call_async(req)

        rclpy.spin_until_future_complete(self._node, future)
        if future.done():
            try:
                result = future.result()
            except Exception as e:
                self._node.get_logger().info('Service call failed %r' % (e,))
                raise
            else:
                if result.success:
                    self._node.get_logger().info('success')
                else:
                    self._node.get_logger().info(result.message)
                    raise Exception(result.message)
        else:
            raise Exception("service is not completed")


    def wait_for_inactive(self, timeout = None):
        self._node.get_logger().info("[torobo_eye client] wait_for_inactive")

        client = self._node.create_client(WaitForInactive, 'wait_for_inactive')
        req = WaitForInactive.Request()
        #TODO
        #req.timeout = timeout
        future = client.call_async(req)

        rclpy.spin_until_future_complete(self._node, future)
        if future.done():
            try:
                result = future.result()
            except Exception as e:
                self._node.get_logger().info('Service call failed %r' % (e,))
                raise
            else:
                if result.success:
                    self._node.get_logger().info('success')
                else:
                    self._node.get_logger().info(result.message)
                    raise Exception(result.message)
        else:
            raise Exception("service is not completed")


    def wait_for_frame(self, timeout = 5.0):
        self._node.get_logger().info("[torobo_eye client] wait_for_frame")

        client = self._node.create_client(WaitForFrame, 'wait_for_frame')
        req = WaitForFrame.Request()
        req.timeout = timeout
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        if future.done():
            try:
                result = future.result()
            except Exception as e:
                self._node.get_logger().info('Service call failed %r' % (e,))
                raise
            else:
                if result.success:
                    self._node.get_logger().info('success')
                    return result
                else:
                    self._node.get_logger().info(result.message)
                    raise Exception(result.message)
        else:
            raise Exception("service is not completed")


    def stop(self):
        self._node.get_logger().info("[torobo_eye client] stop")

        client = self._node.create_client(Stop, 'stop')
        req = Stop.Request()
        future = client.call_async(req)

        rclpy.spin_until_future_complete(self._node, future)
        if future.done():
            try:
                result = future.result()
            except Exception as e:
                self._node.get_logger().info('Service call failed %r' % (e,))
                raise
            else:
                if result.success:
                    self._node.get_logger().info('success')
                else:
                    self._node.get_logger().info(result.message)
                    raise Exception(result.message)
        else:
            raise Exception("service is not completed")


    def get_intrinsics(self):
        self._node.get_logger().info("[torobo_eye client] get_intrinsics")

        client = self._node.create_client(GetIntrinsics, 'get_intrinsics')
        req = GetIntrinsics.Request()
        future = client.call_async(req)

        rclpy.spin_until_future_complete(self._node, future)
        if future.done():
            try:
                result = future.result()
            except Exception as e:
                self._node.get_logger().info('Service call failed %r' % (e,))
                raise
            else:
                if result.success:
                    self._node.get_logger().info('success')
                else:
                    self._node.get_logger().info(result.message)
                    raise Exception(result.message)
        else:
            raise Exception("service is not completed")


    def full_capture(self):
        self._node.get_logger().info("[torobo_eye client] full_capture")

        self.capture()
        frame = self.wait_for_frame()
        bridge = CvBridge()
        color_image = bridge.imgmsg_to_cv2(frame.color_image, 'rgb8')
        depth_image = bridge.imgmsg_to_cv2(frame.depth_image, '32FC1')
        timestamp = frame.timestamp
        #cv2.imwrite('color_image.jpg', color_image)
        #cv2.imwrite('depth_image.png', depth_image)



def main(args=None):
    rclpy.init(args=args)

    camera_server = Server()

    rclpy.spin(camera_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()