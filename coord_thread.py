#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
coord_thread.py - 坐标转换线程模块
功能：将检测到的像素坐标转换为机器人基座坐标系下的三维坐标
说明：严格保留原代码的坐标转换逻辑，无任何修改
"""
import numpy as np
import time
from hand_in_eye_calibration import HandInEyeCalibration

def coord_loop(self):
    """
    坐标转换线程工作函数（原代码的_coord_loop方法）
    :param self: VisionGraspSystem实例
    """
    while self.running:
        # 加锁读取检测结果和深度图
        with self.data_lock:
            hi_dets_copy = self.hi_dets.copy()
            ho_dets_copy = self.ho_dets.copy()
            hi_depth_copy = self.hi_depth.copy() if self.hi_depth is not None else None
            ho_depth_copy = self.ho_depth.copy() if self.ho_depth is not None else None
            current_grasp_class = self.current_grasp_class
            current_place_class = self.current_place_class
        # 加锁读取当前任务阶段
        with self.phase_lock:
            current_phase = self.current_task_phase
        
        # 确定当前坐标转换的目标类
        if current_phase == self.TaskPhase.GRASPING:
            target_class = current_grasp_class
        elif current_phase == self.TaskPhase.PLACING:
            target_class = current_place_class
        else:
            target_class = current_grasp_class if self.current_mode == self.Mode.HAND_OUT_EYE else current_place_class
        
        # -------------------------- 眼在手上坐标转换（精定位） --------------------------
        if (self.current_mode in (self.Mode.HAND_IN_EYE, self.Mode.BOTH) 
            and hi_dets_copy 
            and hi_depth_copy is not None
            and hi_dets_copy[0]['class_name'] == target_class):
            
            top_det = hi_dets_copy[0]
            x, y = top_det['center']
            try:
                # 读取相机内参
                fx, fy = self.hi_intr[0, 0], self.hi_intr[1, 1]
                cx, cy = self.hi_intr[0, 2], self.hi_intr[1, 2]
                # 计算深度值
                self.click_z = float(hi_depth_copy[y][x]) * self.hi_calib.cam_depth_scale
                if self.click_z <= 0:
                    continue
                # 像素坐标→相机坐标
                click_x = (x - cx) * self.click_z / fx
                click_y = (y - cy) * self.click_z / fy
                obj_in_cam = np.array([click_x, click_y, self.click_z])
                
                # 相机坐标→机器人基座坐标
                with self.robot_lock:
                    time.sleep(0.01)
                    tcp_pose = self.robot.get_actual_tcp_pose()
                x_end, y_end, z_end, rx_end, ry_end, rz_end = tcp_pose
                end2base = np.eye(4, dtype=np.float32)
                end2base[:3, :3] = HandInEyeCalibration.euler_to_rotation_matrix(rx_end, ry_end, rz_end)
                end2base[:3, 3] = [x_end, y_end, z_end]
                cam2base = self.hi_calib.calc_cam2base(end2base)
                obj_in_base = self.hi_calib.cam_to_base_coords(cam2base, obj_in_cam)
                
                # 更新坐标和角度（加锁）
                with self.data_lock:
                    self.hi_xyz = obj_in_base
                    self.hi_uv = (int(x), int(y))
                    self.angle = top_det['angle']
            except Exception as e:
                print(f"❌ 眼在手上坐标转换错误：{e}")

        # -------------------------- 眼在手外坐标转换（粗定位） --------------------------
        if (self.current_mode in (self.Mode.HAND_OUT_EYE, self.Mode.BOTH) 
            and ho_dets_copy 
            and ho_depth_copy is not None):
            top_det = ho_dets_copy[0]
            if top_det['class_name'] == target_class:
                x, y = top_det['center']
                try:
                    # 调用眼在手外标定的坐标转换方法
                    obj_in_base = self.ho_calib.pixel_to_robot_coords(x, y, ho_depth_copy)
                    if obj_in_base is not None:
                        with self.data_lock:
                            self.ho_xyz = obj_in_base
                            self.ho_uv = (int(x), int(y))
                except Exception as e:
                    print(f"❌ 眼在手外坐标转换错误：{e}")
        # 休眠50ms降低CPU占用
       # time.sleep(0.05)