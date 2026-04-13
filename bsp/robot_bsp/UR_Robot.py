import time
import rtde_control
import rtde_receive
import minimalmodbus
import numpy as np
import threading
import math
import os

# 从旧代码引入相机驱动
try:
    from ..camera_bsp.realsenseD415 import Camera
except ImportError:
    # 兼容性处理：如果路径不对，请确保目录结构正确
    print("⚠️ 未能通过 ..camera_bsp 找到相机驱动，请检查路径")

lock = threading.Lock()

class UR_Robot:
    def __init__(self, robot_ip="192.168.1.35", 
                 gripper_port='COM10', gripper_baudrate=115200, gripper_address=1,
                 workspace_limits=None, is_use_gripper=True, is_use_camera=True):
        
        # -------------------------- 1. 机器人与工作空间初始化 --------------------------
        if workspace_limits is None:
            workspace_limits = [[-1, 1], [-1, 1], [0.003, 0.9]]
        self.workspace_limits = workspace_limits
        
        try:
            self.rtde_c = rtde_control.RTDEControlInterface(robot_ip)
            self.rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
            print(f"✅ UR机器人RTDE连接成功（IP：{robot_ip}）")
        except Exception as e:
            raise ConnectionError(f"❌ 机器人连接失败: {e}")

        # 运动默认参数
        self.home_joint_config = [0.0, -(90/360.0)*2*np.pi, 0.0, -(90/360.0)*2*np.pi, 0.0, 0.0]

        # -------------------------- 2. 相机初始化（保留旧代码功能） --------------------------
        self.is_use_camera = is_use_camera
        self.camera = None
        self.cam_intrinsics = None
        self.cam_pose = None
        self.cam_depth_scale = None
        if self.is_use_camera:
            self.init_camera()

        # -------------------------- 3. 夹爪初始化与寄存器设定 --------------------------
        self.is_use_gripper = is_use_gripper
        self.REG_WRITE_POS = 0x0102      
        self.REG_WRITE_SPEED = 0x0104    
        self.REG_WRITE_FORCE = 0x0105    
        self.REG_MOTION_TRIGGER = 0x0108 
        self.REG_READ_SPEED = 0x060B     
        self.REG_READ_POS = 0x0609       

        if self.is_use_gripper:
            try:
                self.instrument = minimalmodbus.Instrument(gripper_port, gripper_address)
                self.instrument.serial.baudrate = gripper_baudrate
                self.instrument.serial.timeout = 0.5
                print(f"✅ 夹爪串口 {gripper_port} 连接成功")
            except Exception as e:
                print(f"⚠️ 夹爪连接失败: {e}")
                self.instrument = None

    # -------------------------- 机器人状态监控 (修正后的稳定判定) --------------------------
    def wait_robot_steady(self):
        """通过读取实际 TCP 速度矢量模长判定是否停止，替代不存在的 isSteady()"""
        while True:
            vel = self.rtde_r.getActualTCPSpeed()
            speed = np.linalg.norm(vel)
            if speed < 0.001: 
                break
            time.sleep(0.01)

    # -------------------------- 机器人运动控制 --------------------------
    def moveL(self, target_pose, speed=0.1, acceleration=0.1):
        self.rtde_c.moveL(target_pose, speed, acceleration)
        self.wait_robot_steady()

    def moveJ(self, target_joint, speed=0.2, acceleration=0.2):
        self.rtde_c.moveJ(target_joint, speed, acceleration)
        self.wait_robot_steady()

    def go_home(self):
        self.moveJ(self.home_joint_config)

    def get_actual_tcp_pose(self):
        return self.rtde_r.getActualTCPPose()

    def get_actual_joint_position(self):
        return self.rtde_r.getActualQ()

    # -------------------------- 夹爪控制逻辑 (您手动优化的版本) --------------------------
    def read_actual_speed(self):
        if not self.instrument: return -1
        with lock:
            try: return self.instrument.read_register(self.REG_READ_SPEED)
            except: return -1

    def grip(self, position, speed, force, speed_threshold=5):
        """机器人停稳 -> 发送指令 -> 0.05s缓冲 -> 监控转速归零"""
        if not self.instrument: return
        
        self.wait_robot_steady() # 互锁：机器人不动才准夹爪动

        with lock:
            try:
                self.instrument.write_register(self.REG_MOTION_TRIGGER, 0)
                pos_h, pos_l = (int(position) >> 16) & 0xFFFF, int(position) & 0xFFFF
                self.instrument.write_registers(self.REG_WRITE_POS, [pos_h, pos_l, int(speed), int(force)])
                self.instrument.write_register(self.REG_MOTION_TRIGGER, 1)
            except Exception as e:
                print(f"❌ 夹爪写入失败: {e}")
                return

        time.sleep(0.05) # 您设定的启动缓冲
        while True:
            actual_speed = self.read_actual_speed()
            if actual_speed != -1 and actual_speed <= speed_threshold:
                break
            time.sleep(0.02)
        return True

    # -------------------------- 相机初始化与数据获取 (保留旧代码) --------------------------
    def init_camera(self):
        """初始化相机并加载标定参数"""
        try:
            self.camera = Camera()
            # 加载内参及位姿参数
            self.cam_intrinsics = np.array([386.471, 0, 321.617, 0, 386.034, 237.2, 0, 0, 1]).reshape(3, 3)
            # 确保路径存在，否则打印提示
            if os.path.exists('real/cam_pose/camera_pose.txt'):
                self.cam_pose = np.loadtxt('real/cam_pose/camera_pose.txt', delimiter=' ')
                self.cam_depth_scale = np.loadtxt('real/cam_pose/camera_depth_scale.txt', delimiter=' ')
                print("✅ 相机初始化及参数加载成功")
            else:
                print("⚠️ 相机标定文件路径不存在，仅初始化硬件")
        except Exception as e:
            print(f"❌ 相机初始化失败：{e}")
            self.camera = None

    def get_camera_data(self):
        if self.camera is None:
            print("相机未初始化")
            return None, None
        return self.camera.get_data()

# -------------------------- 测试脚本 --------------------------
if __name__ == "__main__":
    # 初始化（包含相机和夹爪）
    robot = UR_Robot(robot_ip="192.168.1.35", gripper_port='COM10')

    # 1. 坐标定义
    start_pose = [-0.200, -0.22, 0.25, 3.141, 0.0, 0.0]
    pick_pose = [-0.200, -0.22, 0.15, 3.141, 0.0, 0.0] # 下降 10cm

    print("\n🚀 开始自动化测试流程...")

    # 动作序列
    robot.moveL(start_pose, speed=0.2)
    robot.grip(position=0, speed=100, force=30)  # 张开
    
    robot.moveL(pick_pose, speed=0.1)
    robot.grip(position=11000, speed=80, force=60) # 夹紧 (11000)
    
    robot.moveL(start_pose, speed=0.1)
    robot.grip(position=0, speed=100, force=20)  # 释放

    print("\n✅ 测试流程成功完成")