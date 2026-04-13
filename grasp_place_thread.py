#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
grasp_place_thread.py - UR机器人抓取+放置完整线程模块
功能：实现完整的抓取-放置流程，支持语音任务配置（普通抓取/装配）、粗定位、精定位、角度调整、夹爪控制
修复：解决普通抓取无法识别目标的问题（目标类设置、序号逻辑修正、检测流程优化）
"""
import numpy as np
import time, math
import config  # 确保导入config模块

# ---------------------- 保留枚举定义（恢复原有Mode名称，仅保留兜底导入逻辑） ----------------------
try:
    from constants import Mode, TaskPhase
except ImportError:
    # 兜底定义枚举（防止导入失败，保证代码能运行）
    class Mode:
        HAND_OUT_EYE = 0  # 眼在手外
        HAND_IN_EYE = 1   # 眼在手上

    class TaskPhase:
        IDLE = 0
        GRASPING = 1
        PLACING = 2


if hasattr(config, 'HEIGHT_CONFIG'):
    HEIGHT_CONFIG = config.HEIGHT_CONFIG
    # 打印：读取到了config的HEIGHT_CONFIG，以及它的内容
    print(f"【整体配置来源】从config.py中读取到HEIGHT_CONFIG，内容：{HEIGHT_CONFIG}")
else:
    HEIGHT_CONFIG = {}
    # 打印：未读取到，使用空字典
    print(f"【整体配置来源】config.py中无HEIGHT_CONFIG，使用空字典")

# 2. 定义默认高度关联键（防止配置缺失）
DEFAULT_GRASP_KEY = "normal_grasp"
DEFAULT_PLACE_KEY = "normal_place"
DEFAULT_HAND_PLACE_KEY = "hand_place"  # 替换原本地的hand键

# 3. 硬编码兜底配置（极端情况：config中无HEIGHT_CONFIG或默认键缺失时，使用硬编码值）
# 包含原本地配置的核心兜底项，确保程序不会因配置缺失崩溃
FALLBACK_HEIGHT_CONFIG = {
    "normal_grasp": {"safe": 0.30, "final": 0.07},    # 普通抓取兜底
    "normal_place": {"safe": 0.30, "final": 0.018},    # 普通放置兜底
    "grasp_screwdriver": {"safe": 0.30, "final": 0.02},  # 抓螺丝刀兜底
    "grasp_tape": {"safe": 0.30, "final": 0.03},    # 抓胶布兜底
    "hand_place": {"safe": 0.25, "final": 0.08}       # 手放置兜底（对应原本地的hand）
}

# ---------------------- 核心修改：简化参数，从vision_system读取所有关键配置（解决参数传递脱节问题） ----------------------
def grasp_and_place_thread(
    vision_system,
    mode=Mode.HAND_OUT_EYE,  # 保留参数兼容原有逻辑，内部优先用vision_system的current_mode
    GRASP_TARGET_CLASS="default_grasp_target",  # 保留参数兼容原有逻辑，内部替换为配置值
    selected_idx=0  # 核心修改：默认值改为0（匹配Python索引），内部替换为config的选中序号
):
    """
    完整抓取-放置线程（支持语音任务配置、选中序号）
    :param vision_system: 视觉系统实例（必需）
    :param mode: 模式枚举（Mode.HAND_OUT_EYE/Mode.HAND_IN_EYE，默认：HAND_OUT_EYE）
    :param GRASP_TARGET_CLASS: 抓取目标类（默认：default_grasp_target）
    :param selected_idx: 选中的目标序号（默认0，匹配Python索引）
    """
    if not hasattr(vision_system, 'robot') or not hasattr(vision_system, 'data_lock'):
        print(f"❌ 无效的vision_system实例，任务终止")
        return

    # 标记是否抓取成功（用于放置阶段异常兜底）
    is_grasp_success = False
    # -------------------------- 关键修改1：初始化场景化高度配置（使用兜底逻辑） --------------------------
    # 默认使用普通抓取/放置的高度配置（双层兜底：config→硬编码）
    current_grasp_config = HEIGHT_CONFIG.get(DEFAULT_GRASP_KEY, FALLBACK_HEIGHT_CONFIG[DEFAULT_GRASP_KEY])
    current_place_config = HEIGHT_CONFIG.get(DEFAULT_PLACE_KEY, FALLBACK_HEIGHT_CONFIG[DEFAULT_PLACE_KEY])
    
    current_hand_place_config = HEIGHT_CONFIG.get(DEFAULT_HAND_PLACE_KEY, FALLBACK_HEIGHT_CONFIG[DEFAULT_HAND_PLACE_KEY])
    # 安全高度：优先用vision_system内置参数，无则用配置的safe值（后续分抓取/放置单独处理）
    safe_height = getattr(vision_system, 'SAFE_HEIGHT', current_grasp_config['safe'])

    # -------------------------- 新增：从config读取手检测的全局参数（仅用于手放置场景） --------------------------
    hand_detect_config = config.HAND_DETECTION_CONFIG
    hand_retry_count = hand_detect_config["retry_count"]  # 手检测重试次数（替换硬编码的2）
    hand_timeout = hand_detect_config["timeout"]          # 手检测超时时间（替换硬编码的10）
    hand_selected_idx = hand_detect_config["selected_idx"]  # 手的选中序号（默认0，匹配Python索引）
    # 【核心修复1】：不再全局覆盖selected_idx，仅在手动放置场景中使用hand_selected_idx
   
    try:
        ####################################################################
        # 第一部分：抓取物体（支持选中序号，语音任务和原有逻辑一致，完全保留）
        ####################################################################
        print("\n" + "="*60)
        # ---------------------- 核心修改2：从vision_system读取任务配置，替换原有GRASP_TARGET_CLASS ----------------------
        current_task_config = getattr(vision_system, 'current_task_config', None)
        current_base_class = getattr(vision_system, 'current_base_class', None)
        is_voice_task = current_task_config is not None

        if is_voice_task:
            GRASP_TARGET_CLASS = current_task_config.get('grasp_prompt', GRASP_TARGET_CLASS)
        # 修复：task_name缺失问题（从config的TASK_TARGET_CONFIG中反向查找任务名）
        task_name = ""
        if is_voice_task:
            grasp_prompt = current_task_config.get('grasp_prompt', '')
            # 反向遍历config的任务配置，匹配grasp_prompt获取任务名
            for name, cfg in config.TASK_TARGET_CONFIG.items():
                if cfg.get('grasp_prompt') == grasp_prompt:
                    task_name = name
                    break

        # 【核心修复2】：打印调试信息，显示当前检测的目标类和序号
        print(f"📌 【抓取阶段】开始抓取目标 | 选中序号：{selected_idx} | 目标类型：{GRASP_TARGET_CLASS} | 任务类型：{'语音任务' if is_voice_task else '普通抓取'}")
        # ---------------------- 修复配置语法错误+语音任务逻辑 ----------------------
        if current_task_config and 'assembly_offset' in current_task_config:
            assembly_offset = current_task_config['assembly_offset']
            current_task_config['assembly_offset'] = [x for x in assembly_offset if isinstance(x, (int, float))]
        task_type = current_task_config.get('task_type') if (current_task_config and 'task_type' in current_task_config) else None

        # -------------------------- 新增：从config读取当前任务的手配置 --------------------------
        use_hand_placement = False  # 默认不启用手放置
        hand_prompt = "hand"        # 默认手检测关键词
        if is_voice_task:
            use_hand_placement = current_task_config.get('use_hand_placement', False)
            hand_prompt = current_task_config.get('hand_prompt', 'hand')
            task_type = current_task_config.get('task_type')

            # ✅ 修复逻辑：根据任务类型正确分配“放置类目标”
            with vision_system.data_lock:
                if task_type == 'assembly':
                    # 如果是装配任务，放置目标应该是配置里的 base_prompt (即红色物体)
                    vision_system.current_place_class = current_task_config.get('base_prompt', 'red plastic object')
                elif use_hand_placement:
                    # 只有在明确启用手放置时，才设为 hand
                    vision_system.current_place_class = hand_prompt
                else:
                    # 其他情况（如普通放置）
                    vision_system.current_place_class = "red plastic object"
            print(f"🔍 调试：task_name={task_name} | grasp_prompt={GRASP_TARGET_CLASS} | hand_prompt={hand_prompt}")  # 优化调试日志

        # -------------------------- 关键修改2：替换为通过高度关联键读取配置（语音任务） --------------------------
        if is_voice_task:
            # 1. 从任务配置中读取高度关联键（兜底使用默认键）
            grasp_key = current_task_config.get("grasp_height_key", DEFAULT_GRASP_KEY)
            place_key = current_task_config.get("place_height_key", DEFAULT_PLACE_KEY)
            hand_place_key = current_task_config.get("hand_place_height_key", DEFAULT_HAND_PLACE_KEY)

            # 2. 根据关联键获取高度配置（双层兜底：config→硬编码）
            # 抓取配置兜底：先从config的HEIGHT_CONFIG取，再从硬编码FALLBACK取
            current_grasp_config = HEIGHT_CONFIG.get(grasp_key, FALLBACK_HEIGHT_CONFIG.get(grasp_key, FALLBACK_HEIGHT_CONFIG[DEFAULT_GRASP_KEY]))
            # 放置配置：启用手放置则用hand_place，否则用place_key；同样双层兜底
            if use_hand_placement:
                current_place_config = HEIGHT_CONFIG.get(hand_place_key, FALLBACK_HEIGHT_CONFIG.get(hand_place_key, FALLBACK_HEIGHT_CONFIG[DEFAULT_HAND_PLACE_KEY]))
            else:
                current_place_config = HEIGHT_CONFIG.get(place_key, FALLBACK_HEIGHT_CONFIG.get(place_key, FALLBACK_HEIGHT_CONFIG[DEFAULT_PLACE_KEY]))

            # 3. 打印调试信息（替换原有匹配日志）
            print(f"✅ 语音任务高度配置匹配成功：")
            print(f"   - 抓取高度键：{grasp_key} → 配置：{current_grasp_config}")
            print(f"   - 放置高度键：{hand_place_key if use_hand_placement else place_key} → 配置：{current_place_config}")
            print(f"   - 手放置启用：{use_hand_placement}")

            # 核心修改：分抓取/放置单独设置安全高度（解决高度冲突问题）
            grasp_safe_height = getattr(vision_system, 'SAFE_HEIGHT', current_grasp_config['safe'])
            place_safe_height = getattr(vision_system, 'SAFE_HEIGHT', current_place_config['safe'])
            print(f"🎙️ 语音任务类型：{task_type} | 配置：{current_task_config} | 最终匹配抓取高度：{current_grasp_config} | 放置高度：{current_place_config}")
            print(f"🎙️ 手放置配置：启用={use_hand_placement} | 检测关键词={hand_prompt} | 重试次数={hand_retry_count} | 超时时间={hand_timeout}s")
        else:
            # -------------------------- 非语音任务的高度配置（保留原有逻辑，补充兜底） --------------------------
            # 非语音任务（普通抓取）：使用默认高度配置（从config读取，硬编码兜底）
            grasp_safe_height = safe_height
            place_safe_height = safe_height
            # 【核心修复3】：普通抓取时强制使用默认配置，双层兜底
            current_grasp_config = HEIGHT_CONFIG.get(DEFAULT_GRASP_KEY, FALLBACK_HEIGHT_CONFIG[DEFAULT_GRASP_KEY])
            current_place_config = HEIGHT_CONFIG.get(DEFAULT_PLACE_KEY, FALLBACK_HEIGHT_CONFIG[DEFAULT_PLACE_KEY])
            print(f"✅ 普通抓取：使用默认高度配置 | 抓取高度：{current_grasp_config} | 放置高度：{current_place_config}")
        print("="*60)

        with vision_system.phase_lock:
            vision_system.current_task_phase = TaskPhase.GRASPING

        with vision_system.grasp_lock:
            if vision_system.grasping:
                print("⚠️  已有抓取/放置任务在执行，忽略本次触发")
                return
            vision_system.grasping = True

        # -------------------------- 阶段1：眼在手外粗定位抓取目标（支持序号） --------------------------
        print("\n📌 阶段1：眼在手外粗定位（抓取目标 | 选中序号：{}）".format(selected_idx))
        # 核心修改：使用vision_system的current_mode，兼容传入的mode参数
        vision_system.current_mode = getattr(vision_system, 'current_mode', mode)
        vision_system.current_mode = Mode.HAND_OUT_EYE
        with vision_system.data_lock:
            # 【核心修复4】：设置检测目标类为GRASP_TARGET_CLASS，这是识别的关键！
            vision_system.current_target_class = GRASP_TARGET_CLASS
            vision_system.ho_dets = []
            vision_system.ho_xyz = None
            vision_system.has_detection_result = False
        # time.sleep(2)  # 延长休眠，确保配置生效

        img_ready = False
        for _ in range(10):
            with vision_system.data_lock:
                img_ready = vision_system.ho_img is not None
            if img_ready:
                break
            print(f"⌛ 等待眼在手外相机就绪...（{(_+1)*0.5:.1f}s）")
            time.sleep(0.2)
        if not img_ready:
            raise RuntimeError("眼在手外相机未就绪，无法执行抓取")

        max_retry = 3  # 【优化】普通抓取增加重试次数，提高识别成功率
        retry_count = 0
        target_xyz_coarse = None
        while retry_count < max_retry:
            print(f"\n🔄 眼在手外第{retry_count+1}次检测（抓取目标 | 选中序号：{selected_idx}）")
            with vision_system.data_lock:
                vision_system.trigger_detect = True
                vision_system.has_detection_result = False
            #time.sleep(0.2)

            detect_timeout = 0
            max_timeout = 20  # 【优化】普通抓取增加超时时间
            has_result = False
            detecting = False
            while detect_timeout < max_timeout:
                with vision_system.data_lock:
                    has_result = vision_system.has_detection_result
                    detecting = vision_system.detecting
                    ho_dets_count = len(vision_system.ho_dets) if vision_system.ho_dets is not None else 0
                if has_result and ho_dets_count >= (selected_idx + 1) and not detecting:  # 修复：索引从0开始，判断>=idx+1
                    break
                print(f"⌛ 等待检测结果...（{detect_timeout*0.2:.1f}s）| 已检测目标数：{ho_dets_count}")
                time.sleep(0.2)
                detect_timeout += 1

            with vision_system.data_lock:
                temp_xyz = vision_system.ho_xyz.copy() if vision_system.ho_xyz is not None else None
                ho_dets_count = len(vision_system.ho_dets) if vision_system.ho_dets is not None else 0
            
            if temp_xyz is not None and ho_dets_count >= (selected_idx + 1):  # 修复：索引从0开始
                target_xyz_coarse = temp_xyz
                print(f"✅ 眼在手外粗定位坐标（序号{selected_idx}）：{np.round(target_xyz_coarse, 3)}")
                break
            else:
                retry_count += 1
                print(f"❌ 第{retry_count}次检测未找到序号{selected_idx}的抓取目标 | 已检测目标数：{ho_dets_count}")
                if retry_count < max_retry:
                    time.sleep(0.5)

        if target_xyz_coarse is None:
            raise RuntimeError(f"眼在手外多次检测未找到序号{selected_idx}的抓取目标，抓取终止")

        # 粗定位后移动到目标上空（使用配置的抓取安全高度）
        tcp_current = vision_system.robot.get_actual_tcp_pose()
        grasp_overhead_coarse = [
            target_xyz_coarse[0],
            target_xyz_coarse[1] + 0.1,
            grasp_safe_height,  # 核心修改：使用抓取安全高度
            tcp_current[3], tcp_current[4], tcp_current[5]
        ]
        vision_system.robot.moveL(grasp_overhead_coarse, speed=0.15)
       # time.sleep(0.5)
        print(f"✅ 到达抓取粗定位上空（安全高度Z={grasp_safe_height}m）：{np.round(grasp_overhead_coarse, 3)}")

        # -------------------------- 阶段2：眼在手上精定位抓取目标（支持序号） --------------------------
        print("\n📌 阶段2：眼在手上精定位（抓取目标 | 选中序号：{}）".format(selected_idx))
        vision_system.current_mode = Mode.HAND_IN_EYE
        with vision_system.data_lock:
            # 【核心修复5】：再次设置检测目标类，确保眼在手上检测的是正确目标
            vision_system.current_target_class = GRASP_TARGET_CLASS
            vision_system.hi_dets = []
            vision_system.hi_xyz = None
            vision_system.has_detection_result = False
        #time.sleep(2)  # 延长休眠，确保配置生效

        img_ready = False
        for _ in range(4):
            with vision_system.data_lock:
                img_ready = vision_system.hi_img is not None
            if img_ready:
                break
            print(f"⌛ 等待眼在手上相机就绪...（{(_+1)*0.5:.1f}s）")
            time.sleep(0.2)
        if not img_ready:
            raise RuntimeError("眼在手上相机未就绪，无法执行抓取")

        max_retry = 3  # 【优化】增加重试次数
        retry_count = 0
        target_xyz_precise = None
        target_angle = None
        while retry_count < max_retry:
            print(f"\n🔄 眼在手上第{retry_count+1}次检测（抓取目标 | 选中序号：{selected_idx}）")
            with vision_system.data_lock:
                vision_system.trigger_detect = True
                vision_system.has_detection_result = False
            #time.sleep(0.2)

            detect_timeout = 0
            max_timeout = 20  # 【优化】增加超时时间
            has_result = False
            detecting = False
            while detect_timeout < max_timeout:
                with vision_system.data_lock:
                    has_result = vision_system.has_detection_result
                    detecting = vision_system.detecting
                    hi_dets_count = len(vision_system.hi_dets) if vision_system.hi_dets is not None else 0
                if has_result and hi_dets_count >= (selected_idx + 1) and not detecting:  # 修复：索引从0开始
                    break
                print(f"⌛ 等待检测结果...（{detect_timeout*0.2:.1f}s）| 已检测目标数：{hi_dets_count}")
                time.sleep(0.2)
                detect_timeout += 1

            with vision_system.data_lock:
                temp_xyz = vision_system.hi_xyz.copy() if vision_system.hi_xyz is not None else None
                temp_angle = vision_system.angle
                hi_dets_count = len(vision_system.hi_dets) if vision_system.hi_dets is not None else 0
            
            if temp_xyz is not None and hi_dets_count >= (selected_idx + 1):  # 修复：索引从0开始
                target_xyz_precise = temp_xyz
                target_angle = temp_angle
                print(f"✅ 眼在手上精定位坐标（序号{selected_idx}）：{np.round(target_xyz_precise, 3)}")
                print(f"✅ 检测到物体角度：{target_angle:.1f}°" if target_angle else "❌ 无角度")
                break
            else:
                retry_count += 1
                print(f"❌ 第{retry_count}次检测未找到序号{selected_idx}的抓取目标 | 已检测目标数：{hi_dets_count}")
                if retry_count < max_retry:
                    time.sleep(0.5)

        if target_xyz_precise is None:
            raise RuntimeError(f"眼在手上多次检测未找到序号{selected_idx}的抓取目标，抓取终止")

        # 坐标微调+角度修正
        target_xyz_precise[0] += getattr(vision_system, 'COORD_X_OFFSET', 0)
        target_xyz_precise[1] += getattr(vision_system, 'COORD_Y_OFFSET', 0)
        angle_grasp = target_angle if target_angle is not None else 0.0
        if angle_grasp > 45:
            angle_grasp = 90 - angle_grasp
        if abs(angle_grasp) < getattr(vision_system, 'ANGLE_THRESHOLD', 5):
            angle_grasp = 0.0
        print(f"✅ 微调后抓取坐标：{np.round(target_xyz_precise, 3)}")
        print(f"✅ 修正后角度：{angle_grasp:.1f}°")

        # 关节5角度调整
        if angle_grasp != 0.0:
            angle_rad = math.radians(angle_grasp)
            joint_pose = vision_system.robot.get_actual_joint_position()
            joint_pose[5] -= angle_rad
            vision_system.robot.moveJ(joint_pose, speed=0.15)
            #time.sleep(1)
            print(f"✅ 关节5调整完成（偏移：{angle_grasp:.1f}°）")

        # -------------------------- 阶段3：执行抓取（取消过渡高度，直接使用配置高度） --------------------------
        print("\n📌 阶段3：执行抓取动作（使用专属高度配置）")
        tcp_detect = vision_system.robot.get_actual_tcp_pose()
        # 移动到精定位上空（使用配置的抓取安全高度）
        grasp_pos = [
            target_xyz_precise[0],
            target_xyz_precise[1],
            grasp_safe_height,  # 核心修改：使用抓取安全高度
            tcp_detect[3], tcp_detect[4], tcp_detect[5]
        ]
        vision_system.robot.moveL(grasp_pos, speed=0.15)
        #time.sleep(1)
        print(f"✅ 到达精定位上空（安全高度Z={grasp_safe_height}m）：{np.round(grasp_pos, 3)}")

        # 关键修改3：直接下降到配置的最终高度（螺丝刀0.035m，胶带0.08m）
        grasp_pos[2] = current_grasp_config['final']
        vision_system.robot.moveL(grasp_pos, speed=0.1)
        #time.sleep(1)
        print(f"✅ 下降到最终抓取高度Z={current_grasp_config['final']}m：{np.round(grasp_pos, 3)}")

        # 闭合夹爪
        vision_system.robot.grip(
            position=vision_system.close_pos,
            speed=vision_system.GRIP_SPEED,
            force=vision_system.GRIP_FORCE
        )
        #time.sleep(1)
        print("✅ 夹爪闭合，物体抓取成功！")

        # 上升回到安全高度（使用配置的抓取安全高度）
        grasp_pos[2] = grasp_safe_height  # 核心修改：使用抓取安全高度
        vision_system.robot.moveL(grasp_pos, speed=0.1)
        #time.sleep(1)
        print(f"✅ 上升回安全高度Z={grasp_safe_height}m")

        # 恢复关节5姿态
        if angle_grasp != 0.0:
            joint_pose = vision_system.robot.get_actual_joint_position()
            joint_pose[5] += math.radians(angle_grasp)
            vision_system.robot.moveJ(joint_pose, speed=0.15)
            #time.sleep(1)

        # 标记抓取成功
        is_grasp_success = True
        print(f"\n🎉 抓取完成！序号{selected_idx}的物体已抓起，停留在安全高度（Z={grasp_safe_height}m）")

        ####################################################################
        # 第二部分：放置物体（完全保留原手部逻辑 + 新增默认位置分支）
        ####################################################################
        if is_voice_task and task_type == 'grasp_place':
            # -------------------------- 核心逻辑判断：是否使用手放置 --------------------------
            if use_hand_placement:
                # 情况1：语音任务且启用手放置（复刻你提供的完整手部识别逻辑）
                print("\n" + "="*60)
                grasp_prompt_original = current_task_config.get('grasp_prompt', '未知目标')
                print(f"📌 【放置阶段】语音任务：识别手的位置放置 | 目标：{grasp_prompt_original} | 检测关键词：{hand_prompt}")
                print("="*60)

                with vision_system.phase_lock:
                    vision_system.current_task_phase = TaskPhase.PLACING

                # -------------------------- 核心修改：传递hand_prompt给视觉系统 --------------------------
                print("\n📌 阶段1：眼在手外粗定位（手的位置 | 选中序号：{}）".format(hand_selected_idx))
                vision_system.current_mode = Mode.HAND_OUT_EYE  # 仅用眼在手外
                with vision_system.data_lock:
                    vision_system.current_target_class = hand_prompt  # 检测手的目标类
                    vision_system.ho_dets = []
                    vision_system.ho_xyz = None
                    vision_system.has_detection_result = False

                # 等待相机就绪
                img_ready = False
                for _ in range(3):
                    with vision_system.data_lock:
                        img_ready = vision_system.ho_img is not None
                    if img_ready:
                        break
                    print(f"⌛ 等待眼在手外相机就绪...（{(_+1)*0.5:.1f}s）")
                    time.sleep(0.2)
                if not img_ready:
                    raise RuntimeError("眼在手外相机未就绪，无法识别手的位置")

                # -------------------------- 执行手检测重试循环 --------------------------
                retry_count = 0
                hand_xyz_coarse = None
                while retry_count < hand_retry_count:
                    print(f"\n🔄 眼在手外第{retry_count+1}次检测（手的位置 | 选中序号：{hand_selected_idx}）")
                    with vision_system.data_lock:
                        vision_system.trigger_detect = True
                        vision_system.has_detection_result = False

                    detect_timeout = 0
                    while detect_timeout < hand_timeout:
                        with vision_system.data_lock:
                            has_result = vision_system.has_detection_result
                            detecting = vision_system.detecting
                            ho_dets_count = len(vision_system.ho_dets) if vision_system.ho_dets is not None else 0
                        if has_result and ho_dets_count >= (hand_selected_idx + 1) and not detecting:
                            break
                        print(f"⌛ 等待手的检测结果...（{detect_timeout*0.2:.1f}s）| 已检测目标数：{ho_dets_count}")
                        time.sleep(0.2)
                        detect_timeout += 1

                    with vision_system.data_lock:
                        temp_xyz = vision_system.ho_xyz.copy() if vision_system.ho_xyz is not None else None
                        ho_dets_count = len(vision_system.ho_dets) if vision_system.ho_dets is not None else 0
                    
                    if temp_xyz is not None and ho_dets_count >= (hand_selected_idx + 1):
                        hand_xyz_coarse = temp_xyz
                        print(f"✅ 眼在手外粗定位手的坐标（序号{hand_selected_idx}）：{np.round(hand_xyz_coarse, 3)}")
                        break
                    else:
                        retry_count += 1
                        print(f"❌ 第{retry_count}次检测未找到序号{hand_selected_idx}的手的位置 | 已检测目标数：{ho_dets_count}")
                        if retry_count < hand_retry_count:
                            time.sleep(0.5)

                # -------------------------- 确定目标上空位置（带手部识别兜底） --------------------------
                place_overhead = None
                if hand_xyz_coarse is None:
                    print("⚠️ 未识别到手的位置，兜底使用固定位姿放置")
                    place_pose = current_task_config.get('place_pose', [0,0,0,0,0,0])
                    place_overhead = [
                        place_pose[0], place_pose[1],
                        place_safe_height, 
                        place_pose[3], place_pose[4], place_pose[5]
                    ]
                else:
                    tcp_current = vision_system.robot.get_actual_tcp_pose()
                    place_overhead = [
                        hand_xyz_coarse[0], hand_xyz_coarse[1],
                        place_safe_height, 
                        tcp_current[3], tcp_current[4], tcp_current[5]
                    ]

            else:
                # 情况2：不使用手放置 -> 直接读取配置中的默认位置
                print("\n" + "="*60)
                print(f"📌 【放置阶段】use_hand_placement为False，直接前往默认位置")
                print("="*60)
                
                with vision_system.phase_lock:
                    vision_system.current_task_phase = TaskPhase.PLACING

                place_pose = current_task_config.get('place_pose', [0,0,0,0,0,0])
                place_overhead = [
                    place_pose[0], place_pose[1],
                    place_safe_height, 
                    place_pose[3], place_pose[4], place_pose[5]
                ]

            # -------------------------- 统一执行物理动作：放置 --------------------------
            # 1. 移动到上空
            vision_system.robot.moveL(place_overhead, speed=0.15)
            print(f"✅ 到达目标上空（安全高度Z={place_safe_height}m）：{np.round(place_overhead, 3)}")

            # 2. 下降到最终高度
            place_pos = place_overhead.copy()
            place_pos[2] = current_place_config['final']  # 此时的高度配置已由前文的key决定
            vision_system.robot.moveL(place_pos, speed=0.1)
            print(f"✅ 下降到放置最终高度Z={current_place_config['final']}m：{np.round(place_pos, 3)}")

            # 3. 释放夹爪
            vision_system.robot.grip(
                position=vision_system.open_pos,
                speed=vision_system.GRIP_SPEED,
                force=vision_system.GRIP_FORCE
            )
            time.sleep(0.5)
            print("✅ 夹爪打开，物体放置成功！")

            # 4. 抬升并回Home位
            place_pos[2] = place_safe_height
            vision_system.robot.moveL(place_pos, speed=0.1)
            vision_system.robot.moveL(vision_system.GRASP_HOME, speed=0.15)

            print("\n" + "="*60)
            print("🎉 任务总结：")
            print(f"✅ 放置模式：{'视觉找手' if use_hand_placement else '默认固定位置'}")
            print(f"✅ 最终高度Z：{current_place_config['final']}m")
            print("✅ 机器人已返回Home位")
            print("="*60 + "\n")

        elif is_voice_task and task_type == 'assembly':
            # 情况2：语音装配任务（红色塑料件→白色塑料件）→ 保留原有逻辑，使用配置的放置高度
            print("\n" + "="*60)
            print(f"📌 【放置阶段】语音装配任务：放置到基座目标 | 基座类型：{current_base_class}")
            print("="*60)

            with vision_system.phase_lock:
                vision_system.current_task_phase = TaskPhase.PLACING

            # -------------------------- 阶段1：眼在手外粗定位基座目标 --------------------------
            print("\n📌 阶段1：眼在手外粗定位（装配基座 | 选中序号：{}）".format(selected_idx))
            vision_system.current_mode = Mode.HAND_OUT_EYE
            with vision_system.data_lock:
                # 【核心修复6】：设置检测目标类为基座类
                vision_system.current_target_class = current_base_class
                vision_system.ho_dets = []
                vision_system.ho_xyz = None
                vision_system.has_detection_result = False
            #time.sleep(2)

            img_ready = False
            for _ in range(3):
                with vision_system.data_lock:
                    img_ready = vision_system.ho_img is not None
                if img_ready:
                    break
                print(f"⌛ 等待眼在手外相机就绪...（{(_+1)*0.5:.1f}s）")
                time.sleep(0.2)
            if not img_ready:
                raise RuntimeError("眼在手外相机未就绪，无法识别装配基座")

            max_retry = 3  # 优化：增加重试次数
            retry_count = 0
            place_xyz_coarse = None
            while retry_count < max_retry:
                print(f"\n🔄 眼在手外第{retry_count+1}次检测（装配基座 | 选中序号：{selected_idx}）")
                with vision_system.data_lock:
                    vision_system.trigger_detect = True
                    vision_system.has_detection_result = False
               # time.sleep(0.2)

                detect_timeout = 0
                max_timeout = 15  # 优化：增加超时时间
                has_result = False
                detecting = False
                while detect_timeout < max_timeout:
                    with vision_system.data_lock:
                        has_result = vision_system.has_detection_result
                        detecting = vision_system.detecting
                        ho_dets_count = len(vision_system.ho_dets) if vision_system.ho_dets is not None else 0
                    if has_result and ho_dets_count >= (selected_idx + 1) and not detecting:  # 修复：索引从0开始
                        break
                    print(f"⌛ 等待检测结果...（{detect_timeout*0.2:.1f}s）| 已检测目标数：{ho_dets_count}")
                    time.sleep(0.2)
                    detect_timeout += 1

                with vision_system.data_lock:
                    temp_xyz = vision_system.ho_xyz.copy() if vision_system.ho_xyz is not None else None
                    ho_dets_count = len(vision_system.ho_dets) if vision_system.ho_dets is not None else 0
                
                if temp_xyz is not None and ho_dets_count >= (selected_idx + 1):  # 修复：索引从0开始
                    place_xyz_coarse = temp_xyz
                    print(f"✅ 眼在手外粗定位基座坐标（序号{selected_idx}）：{np.round(place_xyz_coarse, 3)}")
                    break
                else:
                    retry_count += 1
                    print(f"❌ 第{retry_count}次检测未找到序号{selected_idx}的装配基座 | 已检测目标数：{ho_dets_count}")
                    if retry_count < max_retry:
                        time.sleep(0.5)

            if place_xyz_coarse is None:
                raise RuntimeError(f"眼在手外多次检测未找到序号{selected_idx}的装配基座，放置终止")

            # 移动到基座粗定位上空（使用配置的放置安全高度）
            tcp_current = vision_system.robot.get_actual_tcp_pose()
            place_overhead_coarse = [
                place_xyz_coarse[0],
                place_xyz_coarse[1] + 0.1,
                place_safe_height,  # 核心修改：使用放置安全高度
                tcp_current[3], tcp_current[4], tcp_current[5]
            ]
            vision_system.robot.moveL(place_overhead_coarse, speed=0.15)
            #time.sleep(0.5)
            print(f"✅ 到达基座粗定位上空：{np.round(place_overhead_coarse, 3)}")

            # -------------------------- 阶段2：眼在手上精定位基座目标 --------------------------
            print("\n📌 阶段2：眼在手上精定位（装配基座 | 选中序号：{}）".format(selected_idx))
            vision_system.current_mode = Mode.HAND_IN_EYE
            with vision_system.data_lock:
                # 【核心修复7】：设置检测目标类为基座类
                vision_system.current_target_class = current_base_class
                vision_system.hi_dets = []
                vision_system.hi_xyz = None
                vision_system.has_detection_result = False
            #time.sleep(2)

            img_ready = False
            for _ in range(4):
                with vision_system.data_lock:
                    img_ready = vision_system.hi_img is not None
                if img_ready:
                    break
                print(f"⌛ 等待眼在手上相机就绪...（{(_+1)*0.5:.1f}s）")
                time.sleep(0.2)
            if not img_ready:
                raise RuntimeError("眼在手上相机未就绪，无法精确定位装配基座")

            max_retry = 3  # 优化：增加重试次数
            retry_count = 0
            place_xyz_precise = None
            place_angle = None
            while retry_count < max_retry:
                print(f"\n🔄 眼在手上第{retry_count+1}次检测（装配基座 | 选中序号：{selected_idx}）")
                with vision_system.data_lock:
                    vision_system.trigger_detect = True
                    vision_system.has_detection_result = False
                #time.sleep(0.2)

                detect_timeout = 0
                max_timeout = 15  # 优化：增加超时时间
                has_result = False
                detecting = False
                while detect_timeout < max_timeout:
                    with vision_system.data_lock:
                        has_result = vision_system.has_detection_result
                        detecting = vision_system.detecting
                        hi_dets_count = len(vision_system.hi_dets) if vision_system.hi_dets is not None else 0
                    if has_result and hi_dets_count >= (selected_idx + 1) and not detecting:  # 修复：索引从0开始
                        break
                    print(f"⌛ 等待检测结果...（{detect_timeout*0.2:.1f}s）| 已检测目标数：{hi_dets_count}")
                    time.sleep(0.2)
                    detect_timeout += 1

                with vision_system.data_lock:
                    temp_xyz = vision_system.hi_xyz.copy() if vision_system.hi_xyz is not None else None
                    temp_angle = vision_system.angle
                    hi_dets_count = len(vision_system.hi_dets) if vision_system.hi_dets is not None else 0
                
                if temp_xyz is not None and hi_dets_count >= (selected_idx + 1):  # 修复：索引从0开始
                    place_xyz_precise = temp_xyz
                    place_angle = temp_angle
                    print(f"✅ 眼在手上精定位基座坐标（序号{selected_idx}）：{np.round(place_xyz_precise, 3)}")
                    print(f"✅ 检测到基座角度：{place_angle:.1f}°" if place_angle else "❌ 无角度")
                    break
                else:
                    retry_count += 1
                    print(f"❌ 第{retry_count}次检测未找到序号{selected_idx}的装配基座 | 已检测目标数：{hi_dets_count}")
                    if retry_count < max_retry:
                        time.sleep(0.5)

            if place_xyz_precise is None:
                raise RuntimeError(f"眼在手上多次检测未找到序号{selected_idx}的装配基座，放置终止")

            # 坐标微调+角度修正（叠加装配偏移量）
            place_xyz_precise[0] += getattr(vision_system, 'COORD_X_OFFSET', 0)
            place_xyz_precise[1] += getattr(vision_system, 'COORD_Y_OFFSET', 0)
            assembly_offset = current_task_config.get('assembly_offset', [0,0,0,0,0,0])
            place_xyz_precise[0] += assembly_offset[0]
            place_xyz_precise[1] += assembly_offset[1]
            place_xyz_precise[2] += assembly_offset[2]

            angle_place = place_angle if place_angle is not None else 0.0
            if angle_place > 45:
                angle_place = 90 - angle_place
            if abs(angle_place) < getattr(vision_system, 'ANGLE_THRESHOLD', 5):
                angle_place = 0.0
            print(f"✅ 微调后基座坐标（含装配偏移）：{np.round(place_xyz_precise, 3)}")
            print(f"✅ 修正后基座角度：{angle_place:.1f}°")

            # 关节5角度调整
            if angle_place != 0.0:
                angle_rad = math.radians(angle_place)
                joint_pose = vision_system.robot.get_actual_joint_position()
                joint_pose[5] -= angle_rad
                vision_system.robot.moveJ(joint_pose, speed=0.15)
                #time.sleep(1)
                print(f"✅ 关节5调整完成（偏移：{angle_place:.1f}°）")

            # -------------------------- 阶段3：执行装配放置（使用配置的放置高度） --------------------------
            print("\n📌 阶段3：执行装配放置动作")
            tcp_detect = vision_system.robot.get_actual_tcp_pose()
            # 移动到基座精定位上空（使用配置的放置安全高度）
            place_pos = [
                place_xyz_precise[0],
                place_xyz_precise[1],
                place_safe_height,  # 核心修改：使用放置安全高度
                tcp_detect[3], tcp_detect[4], tcp_detect[5]
            ]
            vision_system.robot.moveL(place_pos, speed=0.15)
            #time.sleep(1)

            # 关键修改5：下降到配置的放置最终高度
            place_pos[2] = current_place_config['final']
            vision_system.robot.moveL(place_pos, speed=0.1)
           # time.sleep(1)
            print(f"✅ 下降到装配放置最终高度Z={current_place_config['final']}m：{np.round(place_pos, 3)}")

            # 打开夹爪释放物体
            vision_system.robot.grip(
                position=vision_system.open_pos,
                speed=vision_system.GRIP_SPEED,
                force=vision_system.GRIP_FORCE
            )
           # time.sleep(1)
            print("✅ 夹爪打开，装配放置成功！")

            # 上升回到安全高度
            place_pos[2] = place_safe_height
            vision_system.robot.moveL(place_pos, speed=0.1)
          #  time.sleep(1)

            # 恢复关节5姿态
            if angle_place != 0.0:
                joint_pose = vision_system.robot.get_actual_joint_position()
                joint_pose[5] += math.radians(angle_place)
                vision_system.robot.moveJ(joint_pose, speed=0.15)
               # time.sleep(1)

            # 回Home位
            vision_system.robot.moveL(vision_system.GRASP_HOME, speed=0.15)
            #time.sleep(1)

            grasp_prompt_original = current_task_config.get('grasp_prompt', '未知目标')
            print("\n" + "="*60)
            print("🎉 语音装配任务完成！")
            print(f"✅ 抓取目标：{grasp_prompt_original}")
            print(f"✅ 抓取高度配置：{current_grasp_config}")
            print(f"✅ 装配基座：{current_base_class}")
            print("✅ 机器人已返回Home位")
            print("="*60 + "\n")

        else:
            # 情况3：原有逻辑（手动按键触发/普通抓取）→ 使用配置的放置高度
            print("\n" + "="*60)
            print(f"📌 【放置阶段】开始放置到目标位置 | 选中序号：{selected_idx} | 目标类型：{GRASP_TARGET_CLASS}")
            print("="*60)

            with vision_system.phase_lock:
                vision_system.current_task_phase = TaskPhase.PLACING

            # -------------------------- 阶段1：眼在手外粗定位放置位置 --------------------------
            print("\n📌 阶段1：眼在手外粗定位（放置位置 | 选中序号：{}）".format(selected_idx))
            vision_system.current_mode = Mode.HAND_OUT_EYE
            with vision_system.data_lock:
                # 【核心修复8】：放置阶段也设置检测目标类，确保识别正确
                vision_system.current_target_class = GRASP_TARGET_CLASS
                vision_system.ho_dets = []
                vision_system.ho_xyz = None
                vision_system.has_detection_result = False
            #time.sleep(2)

            img_ready = False
            for _ in range(3):
                with vision_system.data_lock:
                    img_ready = vision_system.ho_img is not None
                if img_ready:
                    break
                print(f"⌛ 等待眼在手外相机就绪...（{(_+1)*0.5:.1f}s）")
                time.sleep(0.2)
            if not img_ready:
                raise RuntimeError("眼在手外相机未就绪，无法识别放置位置")

            max_retry = 3  # 优化：增加重试次数
            retry_count = 0
            place_xyz_coarse = None
            while retry_count < max_retry:
                print(f"\n🔄 眼在手外第{retry_count+1}次检测（放置位置 | 选中序号：{selected_idx}）")
                with vision_system.data_lock:
                    vision_system.trigger_detect = True
                    vision_system.has_detection_result = False
                #time.sleep(0.2)

                detect_timeout = 0
                max_timeout = 15  # 优化：增加超时时间
                has_result = False
                detecting = False
                while detect_timeout < max_timeout:
                    with vision_system.data_lock:
                        has_result = vision_system.has_detection_result
                        detecting = vision_system.detecting
                        ho_dets_count = len(vision_system.ho_dets) if vision_system.ho_dets is not None else 0
                    if has_result and ho_dets_count >= (selected_idx + 1) and not detecting:  # 修复：索引从0开始
                        break
                    print(f"⌛ 等待检测结果...（{detect_timeout*0.2:.1f}s）| 已检测目标数：{ho_dets_count}")
                    time.sleep(0.2)
                    detect_timeout += 1

                with vision_system.data_lock:
                    temp_xyz = vision_system.ho_xyz.copy() if vision_system.ho_xyz is not None else None
                    ho_dets_count = len(vision_system.ho_dets) if vision_system.ho_dets is not None else 0
                
                if temp_xyz is not None and ho_dets_count >= (selected_idx + 1):  # 修复：索引从0开始
                    place_xyz_coarse = temp_xyz
                    print(f"✅ 眼在手外粗定位放置坐标（序号{selected_idx}）：{np.round(place_xyz_coarse, 3)}")
                    break
                else:
                    retry_count += 1
                    print(f"❌ 第{retry_count}次检测未找到序号{selected_idx}的放置位置 | 已检测目标数：{ho_dets_count}")
                    if retry_count < max_retry:
                        time.sleep(0.5)

            if place_xyz_coarse is None:
                raise RuntimeError(f"眼在手外多次检测未找到序号{selected_idx}的放置位置，放置终止")

            # 移动到放置粗定位上空（使用配置的放置安全高度）
            tcp_current = vision_system.robot.get_actual_tcp_pose()
            place_overhead_coarse = [
                place_xyz_coarse[0],
                place_xyz_coarse[1] + 0.1,
                place_safe_height,  # 核心修改：使用放置安全高度
                tcp_current[3], tcp_current[4], tcp_current[5]
            ]
            vision_system.robot.moveL(place_overhead_coarse, speed=0.15)
           # time.sleep(0.5)
            print(f"✅ 到达放置粗定位上空：{np.round(place_overhead_coarse, 3)}")

            # -------------------------- 阶段2：眼在手上精定位放置位置 --------------------------
            print("\n📌 阶段2：眼在手上精定位（放置位置 | 选中序号：{}）".format(selected_idx))
            vision_system.current_mode = Mode.HAND_IN_EYE
            with vision_system.data_lock:
                # 【核心修复9】：放置阶段眼在手上也设置检测目标类
                vision_system.current_target_class = GRASP_TARGET_CLASS
                vision_system.hi_dets = []
                vision_system.hi_xyz = None
                vision_system.has_detection_result = False
            #time.sleep(2)

            img_ready = False
            for _ in range(4):
                with vision_system.data_lock:
                    img_ready = vision_system.hi_img is not None
                if img_ready:
                    break
                print(f"⌛ 等待眼在手上相机就绪...（{(_+1)*0.5:.1f}s）")
                time.sleep(0.2)
            if not img_ready:
                raise RuntimeError("眼在手上相机未就绪，无法精确定位放置位置")

            max_retry = 3  # 优化：增加重试次数
            retry_count = 0
            place_xyz_precise = None
            place_angle = None
            while retry_count < max_retry:
                print(f"\n🔄 眼在手上第{retry_count+1}次检测（放置位置 | 选中序号：{selected_idx}）")
                with vision_system.data_lock:
                    vision_system.trigger_detect = True
                    vision_system.has_detection_result = False
                #time.sleep(0.2)

                detect_timeout = 0
                max_timeout = 15  # 优化：增加超时时间
                has_result = False
                detecting = False
                while detect_timeout < max_timeout:
                    with vision_system.data_lock:
                        has_result = vision_system.has_detection_result
                        detecting = vision_system.detecting
                        hi_dets_count = len(vision_system.hi_dets) if vision_system.hi_dets is not None else 0
                    if has_result and hi_dets_count >= (selected_idx + 1) and not detecting:  # 修复：索引从0开始
                        break
                    print(f"⌛ 等待检测结果...（{detect_timeout*0.2:.1f}s）| 已检测目标数：{hi_dets_count}")
                    time.sleep(0.2)
                    detect_timeout += 1

                with vision_system.data_lock:
                    temp_xyz = vision_system.hi_xyz.copy() if vision_system.hi_xyz is not None else None
                    temp_angle = vision_system.angle
                    hi_dets_count = len(vision_system.hi_dets) if vision_system.hi_dets is not None else 0
                
                if temp_xyz is not None and hi_dets_count >= (selected_idx + 1):  # 修复：索引从0开始
                    place_xyz_precise = temp_xyz
                    place_angle = temp_angle
                    print(f"✅ 眼在手上精定位放置坐标（序号{selected_idx}）：{np.round(place_xyz_precise, 3)}")
                    print(f"✅ 检测到放置位角度：{place_angle:.1f}°" if place_angle else "❌ 无角度")
                    break
                else:
                    retry_count += 1
                    print(f"❌ 第{retry_count}次检测未找到序号{selected_idx}的放置位置 | 已检测目标数：{hi_dets_count}")
                    if retry_count < max_retry:
                        time.sleep(0.5)

            if place_xyz_precise is None:
                raise RuntimeError(f"眼在手上多次检测未找到序号{selected_idx}的放置位置，放置终止")

            # 坐标微调+角度修正
            place_xyz_precise[0] += getattr(vision_system, 'COORD_X_OFFSET', 0)
            place_xyz_precise[1] += getattr(vision_system, 'COORD_Y_OFFSET', 0)
            angle_place = place_angle if place_angle is not None else 0.0
            if angle_place > 45:
                angle_place = 90 - angle_place
            if abs(angle_place) < getattr(vision_system, 'ANGLE_THRESHOLD', 5):
                angle_place = 0.0
            print(f"✅ 微调后放置坐标：{np.round(place_xyz_precise, 3)}")
            print(f"✅ 修正后放置角度：{angle_place:.1f}°")

            # 关节5角度调整
            if angle_place != 0.0:
                angle_rad = math.radians(angle_place)
                joint_pose = vision_system.robot.get_actual_joint_position()
                joint_pose[5] -= angle_rad
                vision_system.robot.moveJ(joint_pose, speed=0.15)
                #time.sleep(1)
                print(f"✅ 关节5调整完成（偏移：{angle_place:.1f}°）")

            # -------------------------- 阶段3：执行放置（使用配置的放置高度） --------------------------
            print("\n📌 阶段3：执行放置动作")
            tcp_detect = vision_system.robot.get_actual_tcp_pose()
            # 移动到放置精定位上空（使用配置的放置安全高度）
            place_pos = [
                place_xyz_precise[0],
                place_xyz_precise[1],
                place_safe_height,  # 核心修改：使用放置安全高度
                tcp_detect[3], tcp_detect[4], tcp_detect[5]
            ]
            vision_system.robot.moveL(place_pos, speed=0.15)
           # time.sleep(1)

            # 关键修改6：下降到配置的放置最终高度
            place_pos[2] = current_place_config['final']
            vision_system.robot.moveL(place_pos, speed=0.1)
           # time.sleep(1)
            print(f"✅ 下降到放置最终高度Z={current_place_config['final']}m：{np.round(place_pos, 3)}")

            # 打开夹爪释放物体
            vision_system.robot.grip(
                position=vision_system.open_pos,
                speed=vision_system.GRIP_SPEED,
                force=vision_system.GRIP_FORCE
            )
           # time.sleep(1)
            print("✅ 夹爪打开，物体放置成功！")

            # 上升回到安全高度
            place_pos[2] = place_safe_height
            vision_system.robot.moveL(place_pos, speed=0.1)
          #  time.sleep(1)

            # 恢复关节5姿态
            if angle_place != 0.0:
                joint_pose = vision_system.robot.get_actual_joint_position()
                joint_pose[5] += math.radians(angle_place)
                vision_system.robot.moveJ(joint_pose, speed=0.15)
            #    time.sleep(1)

            # 回Home位
            vision_system.robot.moveL(vision_system.GRASP_HOME, speed=0.15)
           # time.sleep(1)

            print("\n" + "="*60)
            print("🎉 完整抓取-放置任务完成！")
            print(f"✅ 选中序号：{selected_idx}")
            print(f"✅ 抓取目标：{GRASP_TARGET_CLASS}")
            print(f"✅ 抓取高度配置：{current_grasp_config}")
            print(f"✅ 放置目标：{GRASP_TARGET_CLASS}")
            print("✅ 机器人已返回Home位")
            print("="*60 + "\n")

    except Exception as e:
        # 异常处理
        print(f"\n❌ 任务异常（选中序号{selected_idx}）：{str(e)}")
        try:
            # 1. 先回Home位
            vision_system.robot.moveL(vision_system.GRASP_HOME, speed=0.15)
            #time.sleep(1)
            # 2. 打开夹爪
            vision_system.robot.grip(
                position=vision_system.open_pos,
                speed=vision_system.GRIP_SPEED,
                force=vision_system.GRIP_FORCE
            )
            #time.sleep(1)
            if is_grasp_success:
                print(f"⚠️  已释放序号{selected_idx}的抓取物体（放置失败兜底）")
            else:
                print(f"⚠️  确保夹爪打开（序号{selected_idx}抓取失败兜底）")
            print("⚠️  机器人已紧急复位到Home位！")
        except Exception as reset_e:
            print(f"❌ 紧急复位失败：{str(reset_e)}")
        print("="*60 + "\n")

    finally:
        # 复位语音任务状态
        if is_voice_task:
            try:
                vision_system.reset_task_status()
                print("✅ 语音任务状态已复位，等待新指令...")
            except AttributeError:
                print("⚠️  未找到reset_task_status方法，跳过状态复位")

        # 重置任务阶段为空闲
        with vision_system.phase_lock:
            vision_system.current_task_phase = TaskPhase.IDLE
        # 释放锁，重置状态
        with vision_system.data_lock:
            vision_system.ho_dets = []
            vision_system.hi_dets = []
            vision_system.ho_xyz = None
            vision_system.hi_xyz = None
        with vision_system.grasp_lock:
            vision_system.grasping = False