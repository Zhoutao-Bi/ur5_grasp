# -*- coding: utf-8 -*-
"""
工具函数（掩码绘制/图片保存/角度计算）- 独立模块，供detect_thread调用
"""
import cv2
import numpy as np
import time  # 补充：原始代码中用到了time模块，需导入
import os
# 从config导入所需常量（确保config.py中定义了这些常量）
from config import MASK_ALPHA, MASK_COLORS, GRASP_PROMPTS, PLACE_PROMPTS, SAVE_FOLDER

def save_detection_image(img, det_type, target_class):
    """保存带掩码的检测图片（复刻原始逻辑）"""
    try:
        clean_class = target_class.replace(" ", "_").replace("/", "_").replace("\\", "_")
        timestamp = time.strftime("%Y%m%d_%H%M%S_") + str(int(time.time() * 1000) % 1000)
        filename = f"{det_type}_{clean_class}_{timestamp}.jpg"
        filepath = os.path.join(SAVE_FOLDER, filename)
        cv2.imwrite(filepath, img)
        print(f"💾 保存检测图片: {filepath}")
        return True
    except Exception as e:
        print(f"❌ 保存图片失败: {e}")
        return False

def draw_masks(img, dets, masks):
    """绘制掩码（复刻原始逻辑）"""
    if img is None or not dets or not masks or len(masks) != len(dets):
        return img
    
    img_with_mask = img.copy()
    for i, (det, mask) in enumerate(zip(dets, masks)):
        prompt = det['class_name']
        # 区分抓取/放置目标的掩码颜色
        if prompt in GRASP_PROMPTS:
            color = MASK_COLORS[GRASP_PROMPTS.index(prompt)]
        elif prompt in PLACE_PROMPTS:
            color = (255, 165, 0)  # 放置目标用橙色
        else:
            color = (128, 128, 128) # 未知目标用灰色
        
        if mask.dtype != np.uint8:
            mask = (mask > 0.5).astype(np.uint8)
        
        # 绘制半透明掩码
        color_mask = np.zeros_like(img_with_mask, dtype=np.uint8)
        color_mask[mask == 1] = color
        img_with_mask = cv2.addWeighted(img_with_mask, 1 - MASK_ALPHA, color_mask, MASK_ALPHA, 0)
        
        # 绘制中心、标签、角度
        center_x, center_y = det['center']
        label = f'{prompt[:15]}... {det["confidence"]:.2f}'
        if det['angle'] is not None:
            label += f' | 角:{det["angle"]:.1f}°'
        cv2.circle(img_with_mask, (center_x, center_y), 5, (255, 255, 255), -1)
        cv2.putText(img_with_mask, label, (center_x + 10, center_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    
    return img_with_mask

def calculate_angle(binary_mask, angle_threshold):
    """计算物体角度（最小外接矩形-长边角度，复刻原始逻辑）"""
    contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    angle = 0.0
    if contours:
        cnt = max(contours, key=cv2.contourArea)
        rect = cv2.minAreaRect(cnt)
        (cx, cy), (width, height), raw_angle = rect
        
        # 强制保证width=短边、height=长边
        if width > height:
            width, height = height, width
            raw_angle += 90
        
        # 计算长边角度
        long_side_angle = raw_angle + 90
        if long_side_angle > 90:
            long_side_angle -= 180
        if long_side_angle < 0:
            long_side_angle += 90
        if abs(long_side_angle) < angle_threshold:
            long_side_angle = 0.0
        angle = long_side_angle
    return angle


def get_location_tags(x, y, geom_config):
    """
    三态分类判定函数：
    - RECT: 只要在矩形里就有这个标签
    - CIRCLE: 在圆里就有这个标签
    - RECT_OUTSIDE_CIRCLE: 在矩形里且在圆外
    """
    # 自动适配你 config 里可能的命名 (RECT_TOTAL 或 RECT)
    rect = geom_config.get("RECT_TOTAL", geom_config.get("RECT", {}))
    circ = geom_config.get("CIRCLE_INNER", geom_config.get("CIRCLE", {}))
    
    if not rect or not circ:
        return ["OUTSIDE"]

    tags = []
    # 1. 基础判定：是否在矩形里
    in_rect = (rect['x'] <= x <= rect['x'] + rect['w']) and \
              (rect['y'] <= y <= rect['y'] + rect['h'])
    
    if in_rect:
        tags.append("RECT") # 状态：矩形里
        
        # 2. 进阶判定：是在圆里还是圆外
        dist_sq = (x - circ['cx'])**2 + (y - circ['cy'])**2
        if dist_sq <= circ['r']**2:
            tags.append("CIRCLE") # 状态：圆形里
        else:
            tags.append("RECT_OUTSIDE_CIRCLE") # 状态：圆形外矩形里
    else:
        tags.append("OUTSIDE") # 状态：矩形外
        
    return tags