#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
main_loop.py - 主循环模块
功能：处理用户按键输入、实时显示相机画面、叠加状态信息、处理语音任务队列
说明：保留原按键逻辑，新增语音任务队列处理
"""
import cv2
import numpy as np
import config
from grasp_place_thread import grasp_and_place_thread
# ---------------------- 【修改1：导入queue模块，处理队列空异常】 ----------------------
import queue

def _main_loop(self):
    """
    主循环（按键控制+画面显示+语音任务处理）
    :param self: VisionGraspSystem实例
    """
    # 创建显示窗口
    cv2.namedWindow('Eye-In-Hand', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Eye-Out-Hand', cv2.WINDOW_NORMAL)
    
    # 打印操作说明（【修改2：新增语音指令说明】）
    key_desc = "="*60 + "\n📋 操作说明：\n"
    key_desc += "1-眼在手上模式  |  2-眼在手外模式  |  3-双模式\n"
    key_desc += "a/s/d/f-切换抓取目标  |  z/x/c-切换放置目标  |  空格键-触发检测\n"
    key_desc += "g-触发完整抓取-放置流程  |  q-退出程序\n"
    key_desc += "🎙️ 支持语音指令：拿十字螺丝刀、拿胶带、装配红色塑料件\n"
    key_desc += "📁 检测结果图片将自动保存到: " + config.SAVE_FOLDER + "\n"
    key_desc += "="*60
    print(key_desc)
    
    while self.running:
        # ---------------------- 【修改3：核心新增：非阻塞读取语音任务队列】 ----------------------
        try:
            # 非阻塞读取队列（timeout=0表示立即返回，不阻塞主循环）
            task_name = self.task_queue.get(block=False)
            # 调用main.py中的_handle_voice_task处理语音任务
            self._handle_voice_task(task_name)
        except queue.Empty:
            # 队列为空时，不做任何操作，继续主循环
            pass

        # 读取按键输入（1ms超时）
        key = cv2.waitKey(1) & 0xFF
        # 加锁读取当前抓取/放置目标
        with self.data_lock:
            current_grasp_class = self.current_grasp_class
            current_place_class = self.current_place_class
        # 加锁读取当前任务阶段
        with self.phase_lock:
            current_phase = self.current_task_phase

        # -------------------------- 原有按键逻辑（保留，作为备用） --------------------------
        if key == ord('q'):
            # 退出程序
            self.running = False
        elif key == ord('1'):
            # 切换到眼在手上模式
            self.current_mode = self.Mode.HAND_IN_EYE
            with self.data_lock:
                self.ho_dets = []
                self.ho_masks = []
                self.has_detection_result = False
            print('🔄 已切换到【眼在手上】模式（检测放置目标）')
        elif key == ord('2'):
            # 切换到眼在手外模式
            self.current_mode = self.Mode.HAND_OUT_EYE
            with self.data_lock:
                self.hi_dets = []
                self.hi_masks = []
                self.has_detection_result = False
            print('🔄 已切换到【眼在手外】模式（检测抓取目标）')
        elif key == ord('3'):
            # 切换到双模式
            self.current_mode = self.Mode.BOTH
            print('🔄 已启用【双模式】（抓取+放置目标检测）')
        elif key == ord('g'):
            # 触发完整抓取-放置流程（保留，作为手动备用）
            grasp_and_place_thread(
                self, self.Mode, current_grasp_class, current_place_class, selected_idx=1
            )
            print(f"🚀 已触发手动抓取-放置流程！抓取目标：{current_grasp_class} | 放置目标：{current_place_class}")
        elif key == ord(' '):
            # 空格键触发检测
            if not self.detecting:
                with self.data_lock:
                    self.trigger_detect = True
                    self.has_detection_result = False
                print("⚠️  已触发检测，请等待...")
            else:
                print("⚠️  正在检测中，请勿重复触发！")
        elif key in config.GRASP_KEY_MAP:
            # a/s/d/f切换抓取目标（保留，作为手动备用）
            new_index = config.GRASP_KEY_MAP[key]
            self.current_grasp_index = new_index
            self.current_grasp_class = config.GRASP_PROMPTS[new_index]
            with self.data_lock:
                self.hi_dets = []
                self.ho_dets = []
                self.hi_masks = []
                self.ho_masks = []
                self.has_detection_result = False
            print(f'🔄 已切换抓取目标：{self.current_grasp_class}')
        elif key in config.PLACE_KEY_MAP:
            # z/x/c切换放置目标（保留，作为手动备用）
            new_index = config.PLACE_KEY_MAP[key]
            self.current_place_index = new_index
            self.current_place_class = config.PLACE_PROMPTS[new_index]
            with self.data_lock:
                self.hi_dets = []
                self.ho_dets = []
                self.hi_masks = []
                self.ho_masks = []
                self.has_detection_result = False
            print(f'🔄 已切换放置目标：{self.current_place_class}')

        # -------------------------- 原有画面显示逻辑（补全截断的代码） --------------------------
        # 加锁读取显示所需数据
        with self.data_lock:
            hi_img_show = self.hi_img.copy() if self.hi_img is not None else None
            ho_img_show = self.ho_img.copy() if self.ho_img is not None else None
            hi_dets_show = self.hi_dets.copy()
            ho_dets_show = self.ho_dets.copy()
            hi_masks_show = self.hi_masks.copy()
            ho_masks_show = self.ho_masks.copy()
            hi_xyz_show, hi_uv_show = self.hi_xyz, self.hi_uv
            ho_xyz_show, ho_uv_show = self.ho_xyz, self.ho_uv
            detecting = self.detecting
            has_result = self.has_detection_result

        # 眼在手上画面（显示当前阶段的目标）
        if hi_img_show is not None:
            hi_bgr = cv2.cvtColor(hi_img_show, cv2.COLOR_RGB2BGR)
            if has_result:
                hi_bgr = self.draw_masks(hi_bgr, hi_dets_show, hi_masks_show)
            # 显示模式、当前阶段、目标
            mode_text = 'ON' if self.current_mode in (self.Mode.HAND_IN_EYE, self.Mode.BOTH) else 'OFF'
            phase_text = 'GRASPING' if current_phase == self.TaskPhase.GRASPING else ('PLACING' if current_phase == self.TaskPhase.PLACING else 'IDLE')
            cv2.putText(hi_bgr, f'Eye-In-Hand: {mode_text}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(hi_bgr, f'Phase: {phase_text}', (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
            cv2.putText(hi_bgr, f'Grasp Target: {current_grasp_class[:15]}...', (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(hi_bgr, f'Place Target: {current_place_class[:15]}...', (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 165, 0), 2)
            cv2.putText(hi_bgr, 'Detect: Press SPACE' if not detecting else 'Detecting...', (10, 150),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if not detecting else (255, 0, 0), 2)
            # 显示坐标（补全原有截断的代码）
            if hi_uv_show is not None and hi_xyz_show is not None:
                px, py = hi_uv_show
                x, y, z = hi_xyz_show
                cv2.putText(hi_bgr, f'XYZ: ({x:.3f}, {y:.3f}, {z:.3f})', (10, 180),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.circle(hi_bgr, (int(px), int(py)), 5, (0, 255, 0), -1)
            cv2.imshow('Eye-In-Hand', hi_bgr)

        # 眼在手外画面（同眼在手上逻辑，补全）
        if ho_img_show is not None:
            ho_bgr = cv2.cvtColor(ho_img_show, cv2.COLOR_RGB2BGR)
            if has_result:
                ho_bgr = self.draw_masks(ho_bgr, ho_dets_show, ho_masks_show)
            # 显示模式、当前阶段、目标
            mode_text = 'ON' if self.current_mode in (self.Mode.HAND_OUT_EYE, self.Mode.BOTH) else 'OFF'
            phase_text = 'GRASPING' if current_phase == self.TaskPhase.GRASPING else ('PLACING' if current_phase == self.TaskPhase.PLACING else 'IDLE')
            cv2.putText(ho_bgr, f'Eye-Out-Hand: {mode_text}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(ho_bgr, f'Phase: {phase_text}', (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
            cv2.putText(ho_bgr, f'Grasp Target: {current_grasp_class[:15]}...', (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(ho_bgr, f'Place Target: {current_place_class[:15]}...', (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 165, 0), 2)
            cv2.putText(ho_bgr, 'Detect: Press SPACE' if not detecting else 'Detecting...', (10, 150),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if not detecting else (255, 0, 0), 2)
            # 显示坐标
            if ho_uv_show is not None and ho_xyz_show is not None:
                px, py = ho_uv_show
                x, y, z = ho_xyz_show
                cv2.putText(ho_bgr, f'XYZ: ({x:.3f}, {y:.3f}, {z:.3f})', (10, 180),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.circle(ho_bgr, (int(px), int(py)), 5, (0, 255, 0), -1)
            cv2.imshow('Eye-Out-Hand', ho_bgr)

    # 释放窗口
    cv2.destroyAllWindows()
    print("👋 程序已退出，窗口已释放！")