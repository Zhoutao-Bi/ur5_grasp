#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
眼在手外(D455) / 手内(D435I) 双相机：点击选点 -> 机器人基座坐标 -> 移动到目标正上方 50mm

  HO(D455 215122257404) : HandOutEyeCalibration.pixel_to_robot_coords
  HI(D435I 215222074676): UR_Robot.pixel_to_base  (相机到基座 = end2base @ cam2end)
"""
import numpy as np
import cv2

from bsp.camera_bsp.realsenseD415 import Camera
from bsp.camera_bsp.hand_out_eye_calibration import HandOutEyeCalibration

# -------------------------- 配置 --------------------------
ROBOT_IP = "192.168.1.35"
HO_SERIAL = "215122257404"                  # D455 手外
HI_SERIAL = "215222074676"                  # D435I 手内 (机器人内置相机)
CALIB_PATH = "camera_pose.txt"              # 手外: 相机->机器人基座 4x4 (米)
DEPTH_SCALE_FILE = "camera_depth_scale.txt" # 手外: 原始 z16 计数 -> 米
# 手外 D455 内参（与 camera_pose.txt 标定时所用一致，避免实时 fx/fy 浮动）
HO_FX = 386.471
HO_FY = 386.034
HO_CX = 321.617
HO_CY = 237.200
TOOL_ORIENTATION = [3.141, 0.0, 0.0]        # 固定朝下 (RX, RY, RZ)
LIFT = 0.05                                 # 目标正上方 50mm
HO_Z_OFFSET = 0.026                         # 手外D455 高度基准补偿(标定z整体偏低0.026m)
WORKSPACE_LIMITS = [[-0.5, 0.05], [-0.80, -0.45], [-0.2, 0.6]]


def main():
    # 1. 机器人（手内相机 + 移动）。连接失败则仅手外可用。
    robot = None
    try:
        from bsp.robot_bsp.UR_Robot import UR_Robot
        robot = UR_Robot(robot_ip=ROBOT_IP, is_use_camera=True)  # 内置相机 = 手内 D435I
        print("[OK] 机械臂已连接:", ROBOT_IP)
    except Exception as e:
        print("[WARN] 机械臂连接失败，手内/移动不可用:", e)

    # 2. 手外 D455
    ho_cam = Camera(serial=HO_SERIAL)
    # 用标定所用固定内参（与 camera_pose.txt 一致），而非实时 ho_cam.intrinsics
    K_ho = np.array([[HO_FX, 0, HO_CX], [0, HO_FY, HO_CY], [0, 0, 1]])
    print("[OK] HO(D455) 内参 K (标定值):\n", K_ho)
    depth_scale = float(np.loadtxt(DEPTH_SCALE_FILE))
    print("[OK] HO 深度缩放 =", depth_scale, "(相机scale=", ho_cam.scale, ")")

    class ParamHolder:
        cam_intrinsics = K_ho
        workspace_limits = WORKSPACE_LIMITS

    ho = HandOutEyeCalibration(robot=ParamHolder(), calib_path=CALIB_PATH, cam_depth_scale=depth_scale)
    if not np.allclose(ho.cam_intrinsics, K_ho):
        raise RuntimeError("HO 内参不一致：请确认使用 D455 内参")
    print("[OK] 手外标定加载完成 (camera_pose.txt, cam->base, 米)")

    # 3. 窗口
    win_ho = "HO(D455)_eye_out"
    win_hi = "HI(D435I)_eye_in"
    cv2.namedWindow(win_ho, cv2.WINDOW_NORMAL)
    if robot is not None:
        cv2.namedWindow(win_hi, cv2.WINDOW_NORMAL)

    state = {"ho_depth": None, "hi_depth": None, "target": None}  # target=(x,y,z,label)

    def on_ho(event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        d = state["ho_depth"]
        if d is None:
            return
        pt = ho.pixel_to_robot_coords(x, y, d)
        if pt is None:
            print("[HO] 像素(%d,%d) 深度无效，忽略" % (x, y))
            state["target"] = None
            return
        z = float(pt[2]) + HO_Z_OFFSET  # 校正后的高度
        state["target"] = (float(pt[0]), float(pt[1]), z, "HO")
        print("[HO] 像素(%d,%d) -> 机器人基座坐标 [%.3f, %.3f, %.3f]  (已校正z, 上方50mm: z=%.3f)"
              % (x, y, pt[0], pt[1], z, z + LIFT))

    cv2.setMouseCallback(win_ho, on_ho)

    if robot is not None:
        def on_hi(event, x, y, flags, param):
            if event != cv2.EVENT_LBUTTONDOWN:
                return
            d = state["hi_depth"]
            if d is None:
                return
            z_mm = float(d[y][x])
            if z_mm <= 0:
                print("[HI] 像素(%d,%d) 深度无效，忽略" % (x, y))
                state["target"] = None
                return
            try:
                _, m = robot.pixel_to_base(x, y, z_mm)
            except Exception as e:
                print("[HI] 转换失败:", e)
                return
            state["target"] = (float(m[0]), float(m[1]), float(m[2]), "HI")
            print("[HI] 像素(%d,%d) -> 机器人基座坐标 [%.3f, %.3f, %.3f]  (上方50mm: z=%.3f)"
                  % (x, y, m[0], m[1], m[2], m[2] + LIFT))

        cv2.setMouseCallback(win_hi, on_hi)

    print("\n操作说明:")
    print("  左键点击 HO(D455) 或 HI(D435I) 窗口 -> 打印机器人基座坐标")
    print("  g -> 移到最后一次点击目标的 正上方 50mm")
    print("  q -> 退出")

    while True:
        # HO(D455) 取图
        ho_color, ho_depth = ho_cam.get_data()
        state["ho_depth"] = ho_depth
        ho_disp = ho_color.copy()
        cv2.putText(ho_disp, "HO (D455) eye-out", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow(win_ho, ho_disp)

        # HI(D435I) 取图
        if robot is not None:
            hi_color, hi_depth = robot.get_camera_data()
            state["hi_depth"] = hi_depth
            if hi_color is not None:
                hi_disp = hi_color.copy()
                cv2.putText(hi_disp, "HI (D435I) eye-in", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow(win_hi, hi_disp)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('g'):
            t = state["target"]
            if t is None:
                print("[提示] 请先左键点击一个目标")
            elif robot is None:
                print("[提示] 机械臂未连接，无法移动")
            else:
                x, y, z, lab = t
                target = [x, y, z + LIFT] + list(TOOL_ORIENTATION)
                print("[移动] %s -> moveL %s" % (lab, ["%.3f" % v for v in target]))
                robot.moveL(target, speed=0.05, acceleration=0.05)

    cv2.destroyAllWindows()
    print("退出。")


if __name__ == "__main__":
    main()
