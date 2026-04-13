#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
config.py - 系统全局配置参数模块
功能：存储所有固定配置参数，包括机器人、相机、标定、SAM3、夹爪、按键映射、语音任务、抓取高度等
"""
import os

# -------------------------- 机器人基础配置 --------------------------
ROBOT_IP = '192.168.1.35'
TOOL_ORIENTATION = [3.141, 0.000, 0.000]
BOX_POSITION = [-0.200, -0.3 , 0.018, 3.141, 0.000, 0.000]  # 备用放置位置
GRASP_HOME = [-0.200, -0.22, 0.25, 3.141, 0.000, 0.000]   # 初始Home位
SAFE_HEIGHT = 0.30  # 安全高度
GRASP_DEPTH_OFFSET = 0.225  # 抓取下降深度
COORD_X_OFFSET = 0.003      # x轴微调（m）
COORD_Y_OFFSET = -0.004         # y轴微调（m）
GRIP_FORCE = 60            # 夹爪力度
GRIP_SPEED = 50             # 夹爪速度
ANGLE_THRESHOLD = 3         # 角度修正阈值（±3视为0°）

# -------------------------- 【新增】抓取放置高度配置（按任务/类型区分） --------------------------
# 键：高度配置标识，值：安全高度（safe）、最终高度（final）
HEIGHT_CONFIG = {
    "normal_grasp": {"safe": 0.30,  "final": 0.07},    # 普通抓取（如装配零件）
    "normal_place": {"safe": 0.30,  "final": 0.105},     # 普通放置（如装配零件）
    "item_place":   {"safe": 0.30,  "final": 0.018},
    "grasp_screwdriver": {"safe": 0.30,  "final": 0.01},  # 抓螺丝刀（最终高度微调）
    "grasp_tape": {"safe": 0.30,  "final": 0.02},      # 抓胶布（最终高度微调）
    "hand_place": {"safe": 0.25,  "final": 0.08},       # 手放置的高度：safe=手的上空安全高度，final=释放高度
    "grasp_motor": {"safe": 0.30,  "final": 0.015},      # 抓电机（黑色圆形，体积更大，最终高度高于螺丝刀）
    "grasp_flight_control": {"safe": 0.30,  "final": 0.015},  # 抓飞控（黑色方形，最终高度适配方形结构）
    "grasp_esc": {"safe": 0.30,  "final": 0.015},        # 抓电调（红色圆形，和电机高度一致）
    "grasp_battery": {"safe": 0.30,  "final": 0.01}     # 抓电池
}

# -------------------------- 【修改1：新增固定放置位姿（针对三个核心任务）】 --------------------------
SCREWDRIVER_PLACE_POSE = [0.280, -0.300, 0.10, 3.141, 0.000, 0.000]
TAPE_PLACE_POSE = [0.280, -0.300, 0.10, 3.141, 0.000, 0.000]
ASSEMBLY_OFFSET = [0.0, 0.0, 0.00, 0.0, 0.0, 0.0]  
MOTOR_PLACE_POSE = [0.280, -0.300, 0.018, 3.141, 0.000, 0.000]              
FLIGHT_CONTROL_PLACE_POSE = [0.280, -0.300, 0.018, 3.141, 0.000, 0.000]
ESC_PLACE_POSE = [0.280, -0.300, 0.018, 3.141, 0.000, 0.000]    
BATTERY_PLACE_POSE = [0.280, -0.300, 0.018, 3.141, 0.000, 0.000]# 电调放置位姿（需你定义）


# -------------------------- 【修改2：新增语音任务-目标核心配置字典（核心！）】 --------------------------
# 键：语音任务名（与detect.py中的tasks键一致）
# 值：包含视觉识别关键词、放置位姿、任务类型、装配基座关键词等
# config.py - 修改后的TASK_TARGET_CONFIG

REGIONS_GEOMETRY = {
    "RECT_TOTAL": {"x": 46, "y": 51, "w": 458, "h": 426}, # 整个大矩形
    "CIRCLE_INNER": {"cx": 314, "cy": 219, "r": 130}       # 核心圆
}

TASK_TARGET_CONFIG = {
    "拿十字螺丝刀": {
        # 新增：语音指令列表（原ChineseTaskMatcher中的指令）
        "voice_instructions": [
            "把十字螺丝刀拿过来", "帮我拿一下十字螺丝刀", "递一下十字螺丝刀",
            "给我十字螺丝刀", "请把十字螺丝刀递给我",
            "拿十字螺丝刀到指定位置", "把十字螺丝刀放到指定位姿", "递十字螺丝刀到指定位置",
            "取十字螺丝刀", "拿一下螺丝刀", "十字螺丝刀",  
            "把螺丝刀放到指定位置", "将十字螺丝刀放在指定地方"
        ],
        # 原有视觉/机器人配置
        "grasp_prompt": "Screwdriver",
        "grasp_legal_tag": "RECT_OUTSIDE_CIRCLE",
        "place_pose": SCREWDRIVER_PLACE_POSE,
        "task_type": "grasp_place",
        "use_hand_placement": True,
        "hand_prompt": "hand",
        "place_legal_tag": "RECT",
        # 【新增】高度配置关联键（对应HEIGHT_CONFIG的键，不破坏原有结构）
        "grasp_height_key": "grasp_screwdriver",  # 抓取螺丝刀的高度配置
        "place_height_key": "item_place",       # 放置高度配置
        "hand_place_height_key": "hand_place"     # 手放置的高度配置
    },
    "拿胶带": {
        # 新增：语音指令列表
        "voice_instructions": [
            "把胶布拿过来", "帮我拿一下胶布", "递一下胶布", "给我胶布", "请把胶布递给我",
            "拿胶布到指定位置", "把胶布放到指定位置", "递胶布到指定地方",
            "取胶布", "拿一下胶布", "胶布",  
            "将胶布放在指定位置", "帮我拿胶布过去"
        ],
        # 原有视觉/机器人配置
        "grasp_prompt": "Adhesive tape",
        "grasp_legal_tag": "RECT_OUTSIDE_CIRCLE",
        "place_pose": TAPE_PLACE_POSE,
        "task_type": "grasp_place",
        "use_hand_placement": True,
        "hand_prompt": "hand",
        "place_legal_tag": "RECT",
        # 【新增】高度配置关联键
        "grasp_height_key": "grasp_tape",         # 抓取胶带的高度配置
        "place_height_key": "normal_place",       # 放置高度配置
        "hand_place_height_key": "hand_place"     # 手放置的高度配置
    },
    "装配白色塑料件": {
        # 新增：语音指令列表
        "voice_instructions": [
            "帮我装配", "开始装配", "启动装配操作", "进行装配", "请执行装配",
            "装配一下", "开始装配工序", "帮我完成装配",
            "把白色塑料装到红色物体上", "装配白色塑料件到红色物体", "将白色塑料放在红色物体上",
            "执行装配任务", "启动白色塑料件装配", "装配白色和红色物体",
            "把白色物体放到红色物体上面", "夹取白色塑料放到红色物体上"
        ],
        # 原有视觉/机器人配置
        "grasp_prompt": "White plastic object with brushless motor",

        "grasp_legal_tag": "RECT_OUTSIDE_CIRCLE",
        "base_prompt": "red plastic object",
        "place_legal_tag": "CIRCLE",
        "assembly_offset": ASSEMBLY_OFFSET,
        "task_type": "assembly",
        "use_hand_placement": False,
        # 【新增】高度配置关联键
        "grasp_height_key": "normal_grasp",       # 普通抓取高度配置（装配零件）
        "place_height_key": "normal_place",       # 普通放置高度配置（装配零件）
        "hand_place_height_key": "hand_place"     # 备用：手放置高度配置（实际不会用到）
    },
    # ========== 新增：拿电机（黑色圆形塑料物体） ==========
    "拿电机": {
        # 仅修改语音指令：删除错误词+加入全称“无刷电机”，其余完全保留
        "voice_instructions": [
            "帮我拿无刷电机过来", "把无刷电机拿过来", "递一下无刷电机", "给我无刷电机", "请把无刷电机递给我",
            "拿无刷电机到指定位置", "把无刷电机放到指定位置", "递无刷电机到指定地方",
            "取无刷电机", "拿一下无刷电机", "无刷电机",
            # 保留你原有所有电机相关指令（除错误词）
            "帮我拿电机过来", "把电机拿过来", "递一下电机", "给我电机", "请把电机递给我",
            "拿电机到指定位置", "把电机放到指定位置", "递电机到指定地方",
            "取电机", "拿一下电机", "电机",
            "将电机放在指定位置", "帮我拿电机过去"
        ],
        # 以下所有配置一字不改（包括你原有的yellow提示词）
        "grasp_prompt": "red round plastic object ",  # SAM3识别红色圆形塑料物体  with brushless motor
        "grasp_legal_tag": "RECT_OUTSIDE_CIRCLE",      # 从装配区外抓取
        "place_pose": MOTOR_PLACE_POSE,                # 电机放置位姿
        "task_type": "grasp_place",                    # 抓取-放置类型
        "use_hand_placement": False,                    # 支持手放置（和螺丝刀/胶带一致）
        "hand_prompt": "hand",                         # 手检测提示词
        "place_legal_tag": "RECT",                     # 放置到矩形区域
        # 高度配置关联键（新增电机专属抓取高度）
        "grasp_height_key": "grasp_motor",            # 电机抓取高度配置
        "place_height_key": "item_place",           # 普通放置高度
        "hand_place_height_key": "hand_place"         # 手放置高度
    },
    # ========== 新增：拿飞控（黑色方形物体） ==========
    "拿飞控": {
        # 仅修改语音指令：加入全称“飞行控制器”，其余完全保留
        "voice_instructions": [
            "帮我拿飞行控制器过来", "把飞行控制器拿过来", "递一下飞行控制器", "给我飞行控制器", "请把飞行控制器递给我",
            "拿飞行控制器到指定位置", "把飞行控制器放到指定位置", "递飞行控制器到指定地方",
            "取飞行控制器", "拿一下飞行控制器", "飞行控制器",
            # 保留你原有所有飞控相关指令
            "帮我拿飞控过来", "把飞控拿过来", "递一下飞控", "给我飞控", "请把飞控递给我",
            "拿飞控到指定位置", "把飞控放到指定位置", "递飞控到指定地方",
            "取飞控", "拿一下飞控", "飞控",
            "将飞控放在指定位置", "帮我拿飞控过去"
        ],
        # 以下所有配置一字不改
        "grasp_prompt": "yellow square plastic object",  # SAM3识别黄色方形塑料物体
        "grasp_legal_tag": "RECT_OUTSIDE_CIRCLE",       # 从装配区外抓取
        "place_pose": FLIGHT_CONTROL_PLACE_POSE,        # 飞控放置位姿（需你定义）
        "task_type": "grasp_place",
        "use_hand_placement": False,
        "hand_prompt": "hand",
        "place_legal_tag": "RECT",
        # 高度配置关联键（新增飞控专属抓取高度）
        "grasp_height_key": "grasp_flight_control",    # 飞控抓取高度配置
        "place_height_key": "item_place",            # 普通放置高度
        "hand_place_height_key": "hand_place"          # 手放置高度
    },
    # ========== 新增：拿电调（红色圆形物体） ==========
    "拿电调": {
        # 仅修改语音指令：删除错误词“链条”+加入全称“电子调速器”，其余完全保留
        "voice_instructions": [
            "帮我拿电子调速器过来", "把电子调速器拿过来", "递一下电子调速器", "给我电子调速器", "请把电子调速器递给我",
            "拿电子调速器到指定位置", "把电子调速器放到指定位置", "递电子调速器到指定地方",
            "取电子调速器", "拿一下电子调速器", "电子调速器",
            # 保留你原有所有电调相关指令（删除“链条”）
            "帮我拿电调过来", "把电调拿过来", "递一下电调", "给我电调", "请把电调递给我",
            "拿电调到指定位置", "把电调放到指定位置", "递电调到指定地方",
            "取电调", "拿一下电调", "电调",
            "将电调放在指定位置", "帮我拿电调过去"
        ],
        # 以下所有配置一字不改（包括你原有的black提示词）
        "grasp_prompt": "yellow round plastic object",     # SAM3识别黑色圆形塑料物体
        "grasp_legal_tag": "RECT_OUTSIDE_CIRCLE",       # 从装配区外抓取
        "place_pose": ESC_PLACE_POSE,                   # 电调放置位姿（需你定义）
        "task_type": "grasp_place",
        "use_hand_placement": False,
        "hand_prompt": "hand",
        "place_legal_tag": "RECT",
        # 高度配置关联键（新增电调专属抓取高度）
        "grasp_height_key": "grasp_esc",               # 电调抓取高度配置
        "place_height_key": "item_place",            # 普通放置高度
        "hand_place_height_key": "hand_place"          # 手放置高度
    },
    # ========== 新增：拿电池 ==========
    "拿电池": {
        # 语音指令完全保留（无错误+无需要新增全称）
        "voice_instructions": [
            "帮我拿供电电源过来", "把供电电源拿过来", "递一下供电电源", "给我供电电源", "请把供电电源递给我",
            "拿供电电源到指定位置", "把供电电源放到指定位置", "递供电电源到指定地方",
            "取供电电源", "拿一下供电电源", "供电电源",
            "帮我拿电池过来", "把电池拿过来", "递一下电池", "给我电池", "请把电池递给我",
            "拿电池到指定位置", "把电池放到指定位置", "递电池到指定地方",
            "取电池", "拿一下电池", "电池",
            "将电池放在指定位置", "帮我拿电池过去"
        ],
        # 以下所有配置一字不改
        "grasp_prompt": "green rectangle",                # SAM3识别电池（通用且高识别率）
        "grasp_legal_tag": "RECT_OUTSIDE_CIRCLE",       # 从装配区外抓取
        "place_pose": BATTERY_PLACE_POSE,               # 电池放置位姿（需你定义）
        "task_type": "grasp_place",
        "use_hand_placement": True,
        "hand_prompt": "hand",
        "place_legal_tag": "RECT",
        # 高度配置关联键（新增电池专属抓取高度）
        "grasp_height_key": "grasp_battery",            # 电池抓取高度配置
        "place_height_key": "item_place",             # 放置高度
        "hand_place_height_key": "hand_place"           # 手放置高度
    }
}

# 新增：语音匹配的全局阈值（把语音模块的阈值也移到config，统一管理）
VOICE_MATCH_THRESHOLD = 0.85

# -------------------------- 【新增：完整的语音识别相关配置（统一管理）】 --------------------------
# Vosk模型路径
VOSK_MODEL_PATH = "vosk-model-cn-0.22"
# SBERT语义匹配模型路径
SBERT_MODEL_PATH = "./paraphrase-multilingual-MiniLM-L12-v2"
# 麦克风索引（根据实际设备调整）
MIC_INDEX = 1
# VAD语音活动检测模式（0-3，数值越大越严格）
VAD_MODE = 2

# -------------------------- 【新增：手检测与手放置的全局配置】 --------------------------
# 手检测的核心参数（全局通用）
HAND_DETECTION_CONFIG = {
    "conf_thresh": 0.5,          # 手检测的置信度阈值（过滤低置信度结果）
    "retry_count": 2,            # 手检测失败后的重试次数
    "timeout": 5,               # 手检测的超时时间（秒）
    "selected_idx": 0,           # 选择第几个检测到的手（0为第一个，适配多手场景）
    "safe_z": 0.25,              # 手放置的安全高度（移动到手上方的高度）
    "final_z": 0.05              # 手放置的最终高度（下降到手附近的高度）
}

# -------------------------- 相机/标定配置 --------------------------
HAND_IN_EYE_CAMERA_SERIAL = "215222074676"
HAND_OUT_EYE_CAMERA_SERIAL = "215122257404"
HAND_IN_EYE_CALIB_PATH = "eyeinhand_cam2end_12_5_v2.txt"
HAND_OUT_EYE_CALIB_PATH = "eyeouthand_cam2base_01_11.txt"
HAND_OUT_EYE_DEPTH_SCALE = 9.918212890616126889e-04  # 眼在手外深度缩放系数

# -------------------------- SAM3专属配置 --------------------------
# 抓取目标列表（a/s/d/f切换，保留作为备用，与语音任务的关键词对应）
GRASP_PROMPTS = [
    "white plastic object",   # a键切换：白色塑料件（装配基座）
    "red plastic object",     # s键切换：红色塑料件（装配抓取目标）
    "Screwdriver",            # d键切换：十字螺丝刀
    "black square plastic object",          # f键切换
    "black round plastic object"                    # g键切换：手
]
# 放置目标列表（z/x/c切换，保留作为备用）
PLACE_PROMPTS = [
    "red plastic object",        # z键切换：放置盒
    "red txray",             # x键切换：红色托盘
    "rectangular platform",  # c键切换：矩形平台
    "hand"                   # v键切换：手
]
SAM3_CONF_THRESH = 0.4       # SAM3置信度阈值
MASK_ALPHA = 0.4             # 掩码透明度
MASK_COLORS = [(0,255,0), (255,0,0), (0,0,255), (255,255,0), (128,0,128)]  # 【新增】第五个颜色（紫色）用于手的掩码

# -------------------------- 按键映射配置 --------------------------
# 【修改3：标注按键与语音任务的对应关系，保留作为备用】
GRASP_KEY_MAP = {ord('a'):0, ord('s'):1, ord('d'):2, ord('f'):3, ord('g'):4}  # 【新增】g键对应手（备用）
# 备注：d键→十字螺丝刀，f键→胶带，s键→红色塑料件，a键→白色塑料件，g键→手
PLACE_KEY_MAP = {ord('z'):0, ord('x'):1, ord('c'):2, ord('v'):3}              # 【新增】v键对应手（备用）

# -------------------------- 图片保存配置 --------------------------
SAVE_FOLDER = "detection_results"  # 检测结果保存文件夹
if not os.path.exists(SAVE_FOLDER):
    os.makedirs(SAVE_FOLDER)
    print(f"📁 创建保存文件夹: {SAVE_FOLDER}")
