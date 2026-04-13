import cv2
import numpy as np
# 新增：导入RealSense库
import pyrealsense2 as rs

def nothing(x):
    pass

def main():
    # -------------------------- 【修改部分1：替换摄像头开启逻辑】 --------------------------
    # 原代码：cap = cv2.VideoCapture(0)
    # 新代码：初始化RealSense管道和配置
    pipeline = rs.pipeline()
    config = rs.config()
    
    # 【核心：在这里选择你想要的像素/分辨率】（和你原代码逻辑一致，仅改设置方式）
    target_width = 640 
    target_height = 480
    
    # 启用彩色流，设置分辨率和帧率（RealSense默认帧率30）
    config.enable_stream(rs.stream.color, target_width, target_height, rs.format.bgr8, 30)
    
    # 启动管道
    pipeline.start(config)
    
    # 获取实际生效的分辨率（RealSense的分辨率是硬约束，会匹配最接近的支持分辨率）
    # 先获取流的配置信息
    profile = pipeline.get_active_profile()
    color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
    color_intrinsics = color_profile.get_intrinsics()
    actual_w = color_intrinsics.width
    actual_h = color_intrinsics.height
    # -------------------------------------------------------------------------------------

    # 以下代码完全和你原代码一致，无任何修改
    print(f"请求分辨率: {target_width}x{target_height}")
    print(f"实际生效分辨率: {actual_w}x{actual_h}")

    window_name = "ROI_Selector"
    cv2.namedWindow(window_name)

    # 创建滑动条，最大值设为实际像素宽度和高度
    cv2.createTrackbar('Rect_X', window_name, actual_w//4, actual_w, nothing)
    cv2.createTrackbar('Rect_Y', window_name, actual_h//4, actual_h, nothing)
    cv2.createTrackbar('Rect_W', window_name, actual_w//2, actual_w, nothing)
    cv2.createTrackbar('Rect_H', window_name, actual_h//2, actual_h, nothing)
    cv2.createTrackbar('Circle_X', window_name, actual_w//2, actual_w, nothing)
    cv2.createTrackbar('Circle_Y', window_name, actual_h//2, actual_h, nothing)
    cv2.createTrackbar('Radius', window_name, 100, 500, nothing)

    print("\n--- 操作提示 ---")
    print("S 键：保存当前配置 | Q 键：退出程序")

    while True:
        # -------------------------- 【修改部分2：替换帧读取逻辑】 --------------------------
        # 原代码：ret, frame = cap.read()
        # 新代码：从RealSense获取彩色帧
        try:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                print("无法读取摄像头画面")
                continue
            # 将RealSense帧转换为OpenCV格式的numpy数组
            frame = np.asanyarray(color_frame.get_data())
            ret = True  # 保持和原代码的ret变量逻辑一致
        except Exception as e:
            print(f"读取帧出错: {e}")
            break
        # -------------------------------------------------------------------------------------

        if not ret:
            print("无法读取摄像头画面")
            break

        # 获取当前滑动条坐标（原代码无修改）
        rx = cv2.getTrackbarPos('Rect_X', window_name)
        ry = cv2.getTrackbarPos('Rect_Y', window_name)
        rw = cv2.getTrackbarPos('Rect_W', window_name)
        rh = cv2.getTrackbarPos('Rect_H', window_name)
        cx = cv2.getTrackbarPos('Circle_X', window_name)
        cy = cv2.getTrackbarPos('Circle_Y', window_name)
        cr = cv2.getTrackbarPos('Radius', window_name)

        # 可视化：绘制遮罩（原代码无修改）
        overlay = frame.copy()
        cv2.rectangle(overlay, (rx, ry), (rx + rw, ry + rh), (0, 255, 0), -1) # 绿色矩形
        cv2.circle(overlay, (cx, cy), cr, (255, 0, 0), -1) # 蓝色圆形
        cv2.addWeighted(overlay, 0.3, frame, 0.7, 0, frame)

        # 绘制边界线（原代码无修改）
        cv2.rectangle(frame, (rx, ry), (rx + rw, ry + rh), (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), cr, (255, 200, 0), 2)

        # 实时显示分辨率（原代码无修改）
        cv2.putText(frame, f"Resolution: {actual_w}x{actual_h}", (20, 30), 1, 1.2, (0, 255, 255), 2)

        cv2.imshow(window_name, frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            print(f"\n--- 保存参数 (分辨率 {actual_w}x{actual_h}) ---")
            print(f"RECT_ROI = [x:{rx}, y:{ry}, w:{rw}, h:{rh}]")
            print(f"CIRCLE_ROI = [cx:{cx}, cy:{cy}, r:{cr}]")

    # -------------------------- 【修改部分3：替换摄像头释放逻辑】 --------------------------
    # 原代码：cap.release()
    # 新代码：停止RealSense管道
    pipeline.stop()
    # -------------------------------------------------------------------------------------
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()