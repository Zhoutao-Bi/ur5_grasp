# -*- coding: utf-8 -*-
"""
SAM3检测线程 (Ultralytics 升级版) - 支持文本提示、特征重用与多模态预留
"""
import time
import cv2
import numpy as np
import config
from config import SAM3_CONF_THRESH, ANGLE_THRESHOLD
from constants import TaskPhase
from utils import draw_masks, save_detection_image, calculate_angle, get_location_tags

def detect_loop(vision_system):
    """
    SAM3检测线程函数 (Ultralytics 适配版)
    """
    data_lock = vision_system.data_lock
    phase_lock = vision_system.phase_lock
    # 核心修改：使用你在 init.py 中初始化好的新预测器
    sam3_predictor = vision_system.sam3_predictor 

    while vision_system.running:
        # 1. 快速检查触发信号
        with data_lock:
            trigger = vision_system.trigger_detect
        
        if not trigger or vision_system.detecting:
            time.sleep(0.02)
            continue

        vision_system.detecting = True
        
        # 2. 获取任务上下文
        with phase_lock:
            current_phase = vision_system.current_task_phase
        
        task_cfg = getattr(vision_system, 'current_task_config', None)
        
        # [调试] 打印任务配置
        if task_cfg:
             print(f"\n[调试-任务配置] 抓取合法标签：{task_cfg.get('grasp_legal_tag')} | 放置合法标签：{task_cfg.get('place_legal_tag')}")

        # 确定检测方案
        with data_lock:
            # 预留接口：如果外部传入了具体的 bboxes (图像范例模式)，优先使用
            target_bboxes = getattr(vision_system, 'current_target_bboxes', None)
            
            if current_phase == TaskPhase.GRASPING:
                target_prompt = vision_system.current_grasp_class
                print(f"\n📸 触发 Ultralytics-SAM3【抓取阶段】，目标提示词：{target_prompt}")
            elif current_phase == TaskPhase.PLACING:
                target_prompt = vision_system.current_place_class
                print(f"\n📸 触发 Ultralytics-SAM3【放置阶段】，目标提示词：{target_prompt}")
            else:
                target_prompt = vision_system.current_grasp_class
                print(f"\n📸 触发 Ultralytics-SAM3【空闲阶段】，默认目标：{target_prompt}")

            hi_img_copy = vision_system.hi_img.copy() if vision_system.hi_img is not None else None
            ho_img_copy = vision_system.ho_img.copy() if vision_system.ho_img is not None else None
            vision_system.trigger_detect = False

        hi_dets_temp = []
        hi_masks_temp = []
        ho_dets_temp = []
        ho_masks_temp = []

        # ========================== 处理函数：适配 Ultralytics 结果 ==========================
        def process_yolo_sam_results(img, prompt, bboxes=None):
            """
            核心逻辑：提取特征一次，根据提示词获取结果
            支持：文本提示、描述性短语、图像范例(BBox)
            """
            # 模式 4：基于特征的推理以提高效率 (提取特征一次)
            sam3_predictor.set_image(img) 
            
            # 模式 1 & 2：使用文本提示或描述性短语进行 segment
            # 模式 3：使用图像范例 (bboxes) 进行 segment (如果传入了 bboxes)
            results = sam3_predictor(
                text=[prompt] if isinstance(prompt, str) else prompt,
                bboxes=bboxes,
                conf=SAM3_CONF_THRESH
            )
            return results[0] if results else None

        # ========================== 眼在手上检测 (Eye-In-Hand) ==========================
        if vision_system.current_mode in (vision_system.Mode.HAND_IN_EYE, vision_system.Mode.BOTH) and hi_img_copy is not None:
            try:
                # 调用新版推理
                result = process_yolo_sam_results(hi_img_copy, target_prompt, bboxes=target_bboxes)
                
                if result and result.masks is not None:
                    # 转换结果为原有格式，保持对 grasp_place_thread.py 的兼容
                    masks = result.masks.data.cpu().numpy() # [N, H, W]
                    boxes = result.boxes.xyxy.cpu().numpy() # [N, 4]
                    scores = result.boxes.conf.cpu().numpy()

                    for i in range(len(masks)):
                        mask_np = masks[i]
                        score = scores[i]
                        
                        binary_mask = (mask_np > 0.5).astype(np.uint8)
                        M = cv2.moments(binary_mask)
                        if M["m00"] == 0: continue
                        center_x, center_y = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])

                        # 30% 中心区域过滤 (原有逻辑)
                        h, w = hi_img_copy.shape[:2]
                        if not (w * 0.35 < center_x < w * 0.65 and h * 0.35 < center_y < h * 0.65):
                            print(f"[调试-眼在手上] 目标{i+1} | 中心({center_x},{center_y})超出中心区域，过滤")
                            continue 

                        # 计算角度与标签
                        angle = calculate_angle(binary_mask, ANGLE_THRESHOLD)
                        loc_tags = get_location_tags(center_x, center_y, config.REGIONS_GEOMETRY)

                        print(f"[调试-眼在手上] 目标{i+1} | 中心({center_x},{center_y}) | 角度{angle:.1f}° | 置信度：{score:.3f}")

                        hi_dets_temp.append({
                            'class_name': target_prompt,
                            'confidence': float(score),
                            'center': (center_x, center_y),
                            'bbox': tuple(boxes[i]),
                            'angle': angle,
                            'location_tags': loc_tags
                        })
                        hi_masks_temp.append(mask_np)

                hi_dets_temp.sort(key=lambda d: (-d['center'][1], -d['center'][0]))
                if hi_dets_temp:
                    vision_system.angle = hi_dets_temp[0]['angle']

            except Exception as e:
                print(f"❌ 眼在手上检测出错: {e}")

        # ========================== 眼在手外检测 (Eye-Out-Hand) ==========================
        if vision_system.current_mode in (vision_system.Mode.HAND_OUT_EYE, vision_system.Mode.BOTH) and ho_img_copy is not None:
            try:
                # 调用新版推理
                result = process_yolo_sam_results(ho_img_copy, target_prompt, bboxes=target_bboxes)
                
                if result and result.masks is not None:
                    masks = result.masks.data.cpu().numpy()
                    boxes = result.boxes.xyxy.cpu().numpy()
                    scores = result.boxes.conf.cpu().numpy()

                    print(f"[调试-眼在手外] SAM3原始目标数：{len(masks)}")

                    for i in range(len(masks)):
                        mask_np = masks[i]
                        score = scores[i]
                        
                        binary_mask = (mask_np > 0.5).astype(np.uint8)
                        M = cv2.moments(binary_mask)
                        if M["m00"] == 0: continue
                        center_x, center_y = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                        
                        loc_tags = get_location_tags(center_x, center_y, config.REGIONS_GEOMETRY)

                        # 合法性检查 (原有逻辑)
                        is_legal, target_tag = True, None
                        if task_cfg:
                            target_tag = task_cfg.get("grasp_legal_tag") if current_phase == TaskPhase.GRASPING else task_cfg.get("place_legal_tag")
                            if target_tag and target_tag not in loc_tags:
                                is_legal = False

                        print(f"[调试-眼在手外] 目标{i+1} | 置信度:{score:.3f} | 坐标:({center_x},{center_y}) | 合法:{is_legal}")

                        if is_legal:
                            ho_dets_temp.append({
                                'class_name': target_prompt,
                                'confidence': float(score),
                                'center': (center_x, center_y),
                                'bbox': tuple(boxes[i]),
                                'angle': None, 
                                'location_tags': loc_tags
                            })
                            ho_masks_temp.append(mask_np)

                ho_dets_temp.sort(key=lambda d: (d['center'][0], -d['center'][1]))
                print(f"[调试-眼在手外] 最终合法目标数：{len(ho_dets_temp)}")

            except Exception as e:
                print(f"❌ 眼在手外检测出错: {e}")

        # ========================== 更新结果 ==========================
        with data_lock:
            vision_system.hi_dets = hi_dets_temp
            vision_system.hi_masks = hi_masks_temp
            vision_system.ho_dets = ho_dets_temp
            vision_system.ho_masks = ho_masks_temp
            vision_system.has_detection_result = bool(hi_dets_temp) or bool(ho_dets_temp)

        vision_system.detecting = False
        print(f"✅ 检测完成 | 手上:{len(hi_dets_temp)} | 手外:{len(ho_dets_temp)}")