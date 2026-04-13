#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
camera_thread.py - 相机采集线程模块
功能：持续采集眼在手上/眼在手外相机的彩色图和深度图，存入数据缓存
说明：严格保留原代码的采集逻辑，仅拆分线程函数，无任何修改
"""
import time

def camera_loop(self):
    """
    相机采集线程工作函数
    :param self: VisionGraspSystem实例（用于访问数据缓存和锁）
    """
    while self.running:
        try:
            # 采集眼在手上相机数据
            hi_color, hi_depth = self.hi_cam.get_data()
            # 采集眼在手外相机数据
            ho_color, ho_depth = self.ho_cam.get_data()
            # 加锁更新数据缓存
            with self.data_lock:
                self.hi_img = hi_color
                self.hi_depth = hi_depth
                self.ho_img = ho_color
                self.ho_depth = ho_depth
        except Exception as e:
            print('📷 相机采集错误:', e)
        # 控制采集频率（10ms休眠）
        time.sleep(0.01)