#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
init.py - 系统组件初始化模块 (Ultralytics SAM3 升级版)
功能：完成机器人、双相机、手眼标定以及 Ultralytics SAM3 多模态预测器的初始化
"""
import torch
from bsp.robot_bsp.UR_Robot import UR_Robot
from bsp.camera_bsp.realsenseD415 import Camera
from hand_in_eye_calibration import HandInEyeCalibration
from hand_out_eye_calibration import HandOutEyeCalibration

# 1. 【核心修改】导入 Ultralytics SAM3 预测器接口
from ultralytics.models.sam import SAM3SemanticPredictor, SAM3VideoSemanticPredictor
import config

def init_robot():
    """初始化UR机器人（逻辑保持不变）"""
    robot = UR_Robot(robot_ip=config.ROBOT_IP)
    robot.moveL(config.GRASP_HOME)
    robot.grip(position=2000, speed=100, force=40)
    return robot

def init_cameras():
    """初始化双相机（逻辑保持不变）"""
    hi_cam = Camera(serial=config.HAND_IN_EYE_CAMERA_SERIAL, width=1280, height=720, fps=30)
    ho_cam = Camera(serial=config.HAND_OUT_EYE_CAMERA_SERIAL, width=640, height=480, fps=30)
    print("📷 眼在手上相机内参：\n", hi_cam.intrinsics)
    print("📷 眼在手外相机内参：\n", ho_cam.intrinsics)
    return hi_cam, ho_cam, hi_cam.intrinsics, ho_cam.intrinsics

def init_calibration(hi_cam, ho_cam, robot):
    """初始化手眼标定（逻辑保持不变）"""
    hi_calib = HandInEyeCalibration(cam2end_path=config.HAND_IN_EYE_CALIB_PATH, cam_depth_scale=hi_cam.scale)
    ho_calib = HandOutEyeCalibration(robot=robot, calib_path=config.HAND_OUT_EYE_CALIB_PATH, cam_depth_scale=config.HAND_OUT_EYE_DEPTH_SCALE)
    return hi_calib, ho_calib

# 2. 【核心修改】重构 SAM3 模型初始化函数，预留多模态接口
def init_sam3_model():
    """
    初始化 Ultralytics SAM3 预测器组件
    预留接口：文本分割、图像范例分割、特征重用推理、视频语义跟踪
    """
    local_checkpoint = "./sam3/sam3.pt"  # 权重文件路径
    device = "cuda" if torch.cuda.is_available() else "cpu"
    
    print(f"🔧 正在加载 Ultralytics SAM3 预测器 (Device: {device})...")

    # 统一的配置参数
    common_overrides = dict(
        conf=0.25,
        task="segment",
        mode="predict",
        model=local_checkpoint,
        half=True if device == "cuda" else False, # CUDA 环境下启用 FP16 加速
        device=device,
        verbose=False
    )

    # --- 接口 1 & 2 & 4：初始化语义预测器 (SAM3SemanticPredictor) ---
    # 该实例支持：1. 文本提示分割；2. 图像范例(BBox)分割；4. 特征重用推理(inference_features)
    sam3_predictor = SAM3SemanticPredictor(overrides=common_overrides)
    sam3_predictor.setup_model() # 预加载模型到显存
    
    # --- 接口 3：初始化视频跟踪预测器 (SAM3VideoSemanticPredictor) ---
    # 该实例支持：3. 结合语义查询的视频实时跟踪（适用于未来路径规划）
    sam3_video_predictor = SAM3VideoSemanticPredictor(overrides=common_overrides)

    print("✅ Ultralytics SAM3 多模态组件加载完成！")
    
    # 为了保持 main.py 逻辑兼容，返回两个预测器对象
    # 第一个用于主要的静态图识别，第二个作为视频跟踪储备接口
    return sam3_predictor, sam3_video_predictor