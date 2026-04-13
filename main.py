 #!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
main.py - UR机器人视觉抓取主程序入口
功能：导入核心类、初始化系统、启动主流程
说明：集成语音识别模块，支持语音指令控制抓取/装配
修复点：
1. 调用init_calibration时传入self.robot实例，避免重复创建
2. 删除异常处理中冗余的机器人创建和复位代码，解决三次初始化问题
3. 保留所有原有语音识别和核心抓取逻辑，仅修改这两处
4. 修正draw_masks和save_detection_image的导入来源（从utils导入，而非detect_thread）
"""
# ====================== 1. 导入系统内置/第三方依赖 ======================
import numpy as np
import cv2
import time, math
import os
from threading import Thread, Lock
# ---------------------- 新增导入语音识别和队列模块 ----------------------
import queue  # 线程安全队列，用于语音任务传递
import sys
from detect_voice import ChineseTaskMatcher, VADVoskASR  # 导入语音识别核心类

# ====================== 2. 导入自定义配置/常量 ======================
import config  # 所有配置参数（机器人IP、相机串口、SAM3阈值等）
from constants import Mode, TaskPhase  # 枚举类（模式/任务阶段）

# ====================== 3. 导入初始化相关模块 ======================
from init import (
    init_robot,        # 机器人初始化      
    init_cameras,      # 双相机初始化
    init_calibration,  # 手眼标定初始化（已修复）
    init_sam3_model    # SAM3模型初始化
)

# ====================== 4. 导入线程相关模块 ======================
from camera_thread import camera_loop  # 相机采集线程
from detect_thread import detect_loop  # SAM3检测线程（仅导入detect_loop，不再导入工具函数）
from coord_thread import coord_loop    # 坐标转换线程
from main_loop import _main_loop       # 主循环（按键/画面显示）
from grasp_place_thread import grasp_and_place_thread  # 抓取放置线程
# 新增：从utils导入工具函数（修复导入错误）
from utils import draw_masks, save_detection_image

# ====================== 5. 导入机器人/标定相关第三方模块 ======================
from bsp.robot_bsp.UR_Robot import UR_Robot
from bsp.camera_bsp.realsenseD415 import Camera
from hand_in_eye_calibration import HandInEyeCalibration
from hand_out_eye_calibration import HandOutEyeCalibration
import torch
from PIL import Image
# from sam3.model_builder import build_sam3_image_model
# from sam3.model.sam3_image_processor import Sam3Processor

# ====================== 6. 核心系统类（整合所有模块逻辑） ======================
class VisionGraspSystem:
    def __init__(self):
        # -------------------------- 绑定配置参数（供抓取/放置线程访问） --------------------------
        self.TOOL_ORIENTATION = config.TOOL_ORIENTATION
        self.BOX_POSITION = config.BOX_POSITION
        self.GRASP_HOME = config.GRASP_HOME
        self.SAFE_HEIGHT = config.SAFE_HEIGHT
        self.GRASP_DEPTH_OFFSET = config.GRASP_DEPTH_OFFSET
        self.COORD_X_OFFSET = config.COORD_X_OFFSET
        self.COORD_Y_OFFSET = config.COORD_Y_OFFSET
        self.ANGLE_THRESHOLD = config.ANGLE_THRESHOLD
        self.GRIP_FORCE = config.GRIP_FORCE
        self.GRIP_SPEED = config.GRIP_SPEED

        # -------------------------- 枚举类绑定（方便内部调用） --------------------------
        self.Mode = Mode
        self.TaskPhase = TaskPhase

        # -------------------------- 机器人初始化（调用initialization模块） --------------------------
        self.robot = init_robot()  # 第一次创建：唯一的机器人实例
        self.open_pos, self.close_pos = 1000 , 11000 # 夹爪初始位置

        # -------------------------- 相机/标定/SAM3初始化（调用initialization模块） --------------------------
        self.hi_cam, self.ho_cam, self.hi_intr, self.ho_intr = init_cameras()
        # 修复点：调用init_calibration时传入self.robot实例（核心，避免二次创建）
        self.hi_calib, self.ho_calib = init_calibration(self.hi_cam, self.ho_cam, self.robot)
        self.sam3_predictor, self.sam3_video_predictor = init_sam3_model()

        # -------------------------- 系统状态初始化 --------------------------
        self.running = True
        self.data_lock = Lock()
        self.trigger_detect = False  # 触发检测标志
        self.detecting = False        # 检测中标志
        self.has_detection_result = False  # 是否有检测结果

        # -------------------------- 抓取/放置线程控制 --------------------------
        self.grasping = False
        self.grasp_lock = Lock()
        self.robot_lock = Lock()

        # -------------------------- 数据缓存（图像/检测/坐标/角度） --------------------------
        self.hi_img = None
        self.hi_depth = None
        self.ho_img = None
        self.ho_depth = None
        self.hi_dets = []
        self.ho_dets = []
        self.hi_masks = []
        self.ho_masks = []
        self.hi_xyz = None  # 眼在手上目标坐标
        self.hi_uv = None   # 眼在手上目标像素坐标
        self.ho_xyz = None  # 眼在手外目标坐标
        self.ho_uv = None   # 眼在手外目标像素坐标
        self.angle = None   # 物体角度
        self.click_z = None  # 目标深度
        
        self.current_target_bboxes = None  # 存储框选坐标，默认为空时走文本提示模式

        # -------------------------- SAM3检测状态：独立的抓取/放置目标 --------------------------
        self.current_grasp_index = 0  # 抓取目标索引
        self.current_grasp_class = config.GRASP_PROMPTS[self.current_grasp_index]  # 当前抓取目标
        self.current_place_index = 0  # 放置目标索引
        self.current_place_class = config.PLACE_PROMPTS[self.current_place_index]  # 当前放置目标

        # -------------------------- 核心新增：任务阶段标记 --------------------------
        self.current_task_phase = TaskPhase.IDLE
        self.phase_lock = Lock()

        # -------------------------- 绑定外部模块的函数（方便内部调用） --------------------------
        self.draw_masks = draw_masks          # 掩码绘制
        self.save_detection_image = save_detection_image  # 图片保存
        self.camera_loop = camera_loop          # 相机采集线程
        self.detect_loop = detect_loop          # SAM3检测线程
        self.coord_loop = coord_loop            # 坐标转换线程
        self._main_loop = _main_loop            # 主循环

        # ---------------------- 新增语音任务相关属性（核心！） ----------------------
        # 1. 语音任务队列（最大存储10个任务，避免堆积）
        self.task_queue = queue.Queue(maxsize=10)
        # 2. 语音识别线程和ASR实例
        self.voice_thread = None
        self.asr_system = None
        # 3. 任务运行标记（避免并发执行多个抓取/装配任务）
        self.is_task_running = False
        self.task_lock = Lock()  # 任务锁，保证线程安全
        # 4. 当前任务配置（从config.TASK_TARGET_CONFIG读取）
        self.current_task_config = None
        # 5. 装配任务的基座目标（额外存储，供抓取线程使用）
        self.current_base_class = None  # 装配时的基座目标（如白色塑料件）

        self.current_mode = Mode.HAND_OUT_EYE  # 对应constants.py中的Mode.HAND_OUT_EYE=2
        # 语音识别模型配置（可根据实际路径调整，也可放到config.py中）



    # ---------------------- 新增启动语音识别的方法 ----------------------
    def _start_voice_recognition(self):
        """初始化并启动语音识别线程（原有逻辑不变）"""

        VOSK_MODEL_PATH = "vosk-model-cn-0.22"
        SBERT_MODEL_PATH = "./paraphrase-multilingual-MiniLM-L12-v2"
        MIC_INDEX = 1  # 你的麦克风索引
        VAD_MODE = 2
        SBERT_THRESHOLD = 0.65

        try:
            # 1. 初始化SBERT语义匹配器
            sbert_matcher = ChineseTaskMatcher()
            # 2. 初始化Vosk ASR系统（传入任务队列）
            self.asr_system = VADVoskASR(
                vosk_model_path=VOSK_MODEL_PATH, 
                sbert_matcher=sbert_matcher,
                task_queue=self.task_queue,  # 关键：将任务队列传入ASR
                vad_mode=2,
                mic_index=2  #有线1 无线2
            )
            # 3. 启动语音识别线程（守护线程，主程序退出时自动终止）
            self.voice_thread = Thread(target=self.asr_system.start_recognition, daemon=True)
            self.voice_thread.start()
            print("\n✅ 语音识别模块启动成功！等待语音指令...")
        except Exception as e:
            print(f"\n❌ 语音识别模块启动失败：{e}")
            # 语音识别失败不影响核心抓取逻辑，继续运行系统

    # ---------------------- 新增语音任务处理方法（核心！） ----------------------
    def _handle_voice_task(self, task_name):
        """处理语音任务（原有逻辑不变）"""
        # 1. 加锁保证线程安全，检查是否有任务正在执行
        with self.task_lock:
            if self.is_task_running:
                print(f"\n⚠️ 当前有任务正在执行，请等待完成后再发送指令！")
                return
            # 标记任务开始运行
            self.is_task_running = True

        try:
            # 2. 检查任务名是否在config的TASK_TARGET_CONFIG中
            if task_name not in config.TASK_TARGET_CONFIG:
                print(f"\n⚠️ 任务「{task_name}」无对应配置，请检查config.py的TASK_TARGET_CONFIG！")
                self.is_task_running = False
                return

            # 3. 获取当前任务的配置
            self.current_task_config = config.TASK_TARGET_CONFIG[task_name]
            task_type = self.current_task_config["task_type"]
            print(f"\n📌 收到语音任务：{task_name} | 任务类型：{task_type}")
            print(f"🔧 任务配置：{self.current_task_config}")

            # 4. 根据任务类型设置SAM3的检测目标
            if task_type == "grasp_place":
                # 普通抓取放置（螺丝刀/胶带）：设置抓取目标
                grasp_prompt = self.current_task_config["grasp_prompt"]
                # 更新SAM3的当前抓取目标（替换原有按键选择的目标）
                self.current_grasp_class = grasp_prompt
                # 找到抓取目标对应的索引（可选，保留原有索引逻辑兼容）
                if grasp_prompt in config.GRASP_PROMPTS:
                    self.current_grasp_index = config.GRASP_PROMPTS.index(grasp_prompt)
                print(f"\n🎯 SAM3检测目标已设置为：{self.current_grasp_class}")

            elif task_type == "assembly":
                # 装配任务（红色塑料件→白色塑料件）：设置抓取目标+基座目标
                grasp_prompt = self.current_task_config["grasp_prompt"]
                base_prompt = self.current_task_config["base_prompt"]
                # 更新SAM3的抓取目标
                self.current_grasp_class = grasp_prompt
                if grasp_prompt in config.GRASP_PROMPTS:
                    self.current_grasp_index = config.GRASP_PROMPTS.index(grasp_prompt)
                # 存储装配的基座目标（供抓取线程使用）
                self.current_base_class = base_prompt
                print(f"\n🎯 SAM3抓取目标已设置为：{self.current_grasp_class}")
                print(f"🎯 装配基座目标已设置为：{self.current_base_class}")

            else:
                print(f"\n⚠️ 不支持的任务类型：{task_type}")
                self.is_task_running = False
                return

            # 5. 触发抓取放置线程（核心：执行实际的抓取/装配动作）
            print(f"\n🤖 开始执行「{task_name}」任务...")
            # 启动抓取放置线程（守护线程）
            Thread(target=grasp_and_place_thread, args=(self,), daemon=True).start()

        except Exception as e:
            print(f"\n❌ 处理语音任务失败：{e}")
            # 异常时重置任务状态
            self.is_task_running = False
            self.current_task_config = None
            self.current_base_class = None

    # ---------------------- 新增任务完成后的复位方法（供抓取线程调用） ----------------------
    def reset_task_status(self):
        """任务完成/失败后复位状态（原有逻辑不变）"""
        with self.task_lock:
            self.is_task_running = False
        self.current_task_config = None
        self.current_base_class = None
        self.current_task_phase = TaskPhase.IDLE
        print(f"\n🔚 任务状态已复位，等待新的语音指令...")

    def start(self):
        """启动所有子线程+主循环+语音识别（原有逻辑不变）"""
        # 启动子线程（调用外部模块的线程函数）
        Thread(target=self.camera_loop, args=(self,), daemon=True).start()
        Thread(target=self.detect_loop, args=(self,), daemon=True).start()
        Thread(target=self.coord_loop, args=(self,), daemon=True).start()

        # 启动语音识别模块
        self._start_voice_recognition()

        # 启动主循环（按键控制+画面显示+语音任务处理）
        self._main_loop(self)

# ====================== 7. 程序入口 ======================
if __name__ == '__main__':
    try:
        # 初始化视觉抓取系统
        vision_system = VisionGraspSystem()
        # 启动系统
        vision_system.start()
    except Exception as e:
        print(f'❌ 程序初始化失败: {e}')
        # 修复点：删除冗余的机器人创建和复位代码，改为提示用户（避免三次创建）
        print("⚠️  请检查机器人连接或禁用工业协议后重新运行程序！")
        # 原有冗余代码已删除，不再创建新的机器人实例