import numpy as np
import os
import math

class HandInEyeCalibration:
    def __init__(self, cam2end_path, correction_matrix=None, cam_depth_scale=None):
        """
        初始化手眼标定处理器（修复Y轴偏差核心逻辑）
        :param cam2end_path: 相机到末端执行器的标定文件路径
        :param correction_matrix: 坐标系修正矩阵（默认使用固定修正矩阵）
        :param cam_depth_scale: 相机深度缩放因子（必须传入正数，从Realsense获取）
        """
        self.cam2end_path = cam2end_path
        self.correction_matrix = correction_matrix if correction_matrix is not None else self._default_correction_matrix()
        self.cam2end_matrix = self.load_cam2end()  # 相机→末端变换矩阵
        self.cam2base_matrix = None  # 相机→基座变换矩阵
        
        # 深度缩放因子严格校验
        if cam_depth_scale is not None:
            if isinstance(cam_depth_scale, (int, float)):
                if cam_depth_scale > 0:
                    self.cam_depth_scale = float(cam_depth_scale)
                else:
                    raise ValueError("深度缩放比例必须是正数（cam_depth_scale > 0）")
            else:
                raise TypeError("cam_depth_scale必须是整数或浮点数（如：9.887695312491126e-04）")
        else:
            raise ValueError("必须传入cam_depth_scale（从Realsense相机获取）")

    @staticmethod
    def euler_to_rotation_matrix(rx, ry, rz):
        """
        适配UR机器人的欧拉角转旋转矩阵（修复：X→Y→Z固定轴顺序）
        :param rx: UR机器人的rx（绕固定基座X轴旋转，弧度）
        :param ry: UR机器人的ry（绕固定基座Y轴旋转，弧度）
        :param rz: UR机器人的rz（绕固定基座Z轴旋转，弧度）
        :return: 3x3旋转矩阵（满足SO(3)约束）
        """
        # 绕X轴旋转（rx）
        cx, sx = math.cos(rx), math.sin(rx)
        Rx = np.array([
            [1, 0, 0],
            [0, cx, -sx],
            [0, sx, cx]
        ], dtype=np.float32)
        
        # 绕Y轴旋转（ry）
        cy, sy = math.cos(ry), math.sin(ry)
        Ry = np.array([
            [cy, 0, sy],
            [0, 1, 0],
            [-sy, 0, cy]
        ], dtype=np.float32)
        
        # 绕Z轴旋转（rz）
        cz, sz = math.cos(rz), math.sin(rz)
        Rz = np.array([
            [cz, -sz, 0],
            [sz, cz, 0],
            [0, 0, 1]
        ], dtype=np.float32)
        
        # UR官方顺序：固定轴X→Y→Z（核心修复）
        R = Rx @ Ry @ Rz
        
        # 正交化校正（消除浮点误差）
        U, _, Vt = np.linalg.svd(R)
        R_corrected = U @ Vt
        if np.linalg.det(R_corrected) < 0:
            Vt[-1, :] *= -1
            R_corrected = U @ Vt
        
        return R_corrected

    def _default_correction_matrix(self):
        """默认坐标系修正矩阵（核心修复：取消Y轴反转）"""
        return np.array([
            [1,  0,  0, 0],  # X轴不反转
            [0,  1,  0, 0],  # 核心修复：Y轴不反转（取消-1）
            [0,  0, 1, 0],  # Z轴不反转
            [0,  0,  0, 1]
        ], dtype=np.float32)

    def load_cam2end(self):
        """加载相机到末端矩阵（核心修复：修正矩阵乘法顺序）"""
        if not os.path.exists(self.cam2end_path):
            raise FileNotFoundError(f"手眼标定文件不存在：{self.cam2end_path}")

        cam2end = []
        with open(self.cam2end_path, 'r', encoding='utf-8') as f:
            for line_num, line in enumerate(f, 1):
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                try:
                    row = list(map(float, line.split()))
                except ValueError:
                    raise ValueError(f"标定文件第{line_num}行格式错误，需为4个浮点数")
                if len(row) != 4:
                    raise ValueError(f"标定文件第{line_num}行需包含4个数值，实际为{len(row)}个")
                cam2end.append(row)
        
        if len(cam2end) != 4:
            raise ValueError(f"标定矩阵需为4x4，实际为{len(cam2end)}行")
        cam2end = np.array(cam2end, dtype=np.float32)
        
        # 核心修复：修正矩阵乘法顺序（先修正矩阵，后cam2end）
        cam2end_corrected = self.correction_matrix @ cam2end
        print(f"✅ 手眼标定矩阵（相机→末端）加载完成，已应用坐标系修正")
        return cam2end_corrected

    def validate_cam2end(self):
        """验证相机到末端矩阵的有效性"""
        if not hasattr(self, '_det_corrected_printed'):
            self._det_corrected_printed = False
        
        if self.cam2end_matrix.shape != (4, 4):
            return False, f"标定矩阵维度错误，需为4x4，实际为{self.cam2end_matrix.shape}"
        
        homogeneous_row = self.cam2end_matrix[3, :]
        if not np.allclose(homogeneous_row, np.array([0, 0, 0, 1], dtype=np.float32), atol=1e-6):
            return False, f"标定矩阵齐次项错误，最后一行需为[0,0,0,1]，实际为{homogeneous_row}"
        
        R_cam2end = self.cam2end_matrix[:3, :3]
        det_R = np.linalg.det(R_cam2end)
        if not (np.allclose(det_R, 1.0, atol=1e-3) or np.allclose(det_R, -1.0, atol=1e-3)):
            return False, f"标定矩阵旋转部分行列式错误（需≈±1），实际为{det_R:.6f}"
        
        if np.allclose(det_R, -1.0, atol=1e-3) and not self._det_corrected_printed:
            flip_z = np.array([[1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]], dtype=np.float32)
            self.cam2end_matrix = self.cam2end_matrix @ flip_z
            print(f"⚠️  标定矩阵旋转部分行列式=-1，已自动校正为1（翻转Z轴）")
            self._det_corrected_printed = True
        
        return True, "手眼标定矩阵有效（已兼容坐标系修正的镜像变换）"

    def calc_cam2base(self, end2base_matrix):
        """计算相机到基座的变换矩阵"""
        if not hasattr(self, '_cam2base_printed'):
            self._cam2base_printed = False
        
        cam2end_valid, cam2end_msg = self.validate_cam2end()
        if not cam2end_valid:
            raise ValueError(f"相机→末端矩阵无效：{cam2end_msg}")
        
        if end2base_matrix.shape != (4, 4):
            raise ValueError(f"末端→基座矩阵维度错误，需为4x4，实际为{end2base_matrix.shape}")
        
        self.cam2base_matrix = end2base_matrix @ self.cam2end_matrix
        
        if not self._cam2base_printed:
            print(f"✅ 相机→基座变换矩阵计算完成")
            self._cam2base_printed = True
        
        return self.cam2base_matrix

    def cam_to_base_coords(self, cam2base, obj_in_cam):
        """将相机坐标系下的点转换为基座坐标系下的点"""
        if self.cam2base_matrix is None:
            raise RuntimeError("请先调用calc_cam2base计算相机→基座变换矩阵")
        
        if len(obj_in_cam) != 3:
            raise ValueError(f"相机坐标系坐标需为3个数值，实际为{len(obj_in_cam)}个")
        
        # 转换为齐次坐标并计算
        obj_homog = np.append(obj_in_cam, 1.0).astype(np.float32)
        obj_in_base_homog = cam2base @ obj_homog
        obj_in_base = obj_in_base_homog[:3]
        
        return obj_in_base