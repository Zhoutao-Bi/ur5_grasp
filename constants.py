# constants.py 新增内容
class TaskType:
    GRASP_PLACE = "grasp_place"  # 普通抓取放置
    ASSEMBLY = "assembly"        # 装配任务

# 原有枚举（保留）
class TaskPhase:
    IDLE = 0
    GRASPING = 1
    PLACING = 2

class Mode:
    HAND_IN_EYE = 1
    HAND_OUT_EYE = 2
    BOTH = 3