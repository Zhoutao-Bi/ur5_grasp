import numpy as np
import pyrealsense2 as rs
import cv2

class Camera(object):

    def __init__(self, width=640, height=480, fps=30, serial=None):  # 新增 serial 参数
        self.im_height = height
        self.im_width = width
        self.fps = fps
        self.serial = serial  # 保存序列号
        self.intrinsics = None
        self.scale = None
        self.pipeline = None
        self.connect()  # 连接相机


    def connect(self):
        # 配置相机管道
        self.pipeline = rs.pipeline()
        config = rs.config()

        # 如果指定了序列号，只连接该相机
        if self.serial is not None:
            config.enable_device(self.serial)  # 通过序列号指定设备

        # 配置流（深度和彩色图）
        config.enable_stream(rs.stream.depth, self.im_width, self.im_height, rs.format.z16, self.fps)
        config.enable_stream(rs.stream.color, self.im_width, self.im_height, rs.format.bgr8, self.fps)

        # 启动流
        cfg = self.pipeline.start(config)

        # 获取内参
        rgb_profile = cfg.get_stream(rs.stream.color)
        self.intrinsics = self.get_intrinsics(rgb_profile)

        # 获取深度缩放系数（将深度值转为米）
        self.scale = cfg.get_device().first_depth_sensor().get_depth_scale()

        #print(f"深度缩放系数：{self.scale} 米/单位")
        # 打印连接信息（包含序列号，方便确认）
        print(f"D415 已连接（序列号：{self.serial if self.serial else '未指定'}）")


    def get_data(self):
        # 等待一帧数据（深度和彩色图）
        frames = self.pipeline.wait_for_frames()

        # 对齐深度图到彩色图（确保像素位置对应）
        align = rs.align(align_to=rs.stream.color)
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not aligned_depth_frame or not color_frame:
            return None, None  # 数据无效时返回None

        # 转换为numpy数组
        depth_image = np.asanyarray(aligned_depth_frame.get_data(), dtype=np.float32)
        color_image = np.asanyarray(color_frame.get_data())  # BGR格式（OpenCV默认）

        return color_image, depth_image


    def plot_image(self):
        color_image, depth_image = self.get_data()
        if color_image is None or depth_image is None:
            print("获取图像失败")
            return

        # 深度图上色（便于可视化）
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # 确保深度图和彩色图尺寸一致
        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, 
                                           dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                                           interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))

        # 显示图像
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(5000)
        cv2.destroyAllWindows()


    def get_intrinsics(self, rgb_profile):
        # 获取相机内参（3x3矩阵）
        raw_intrinsics = rgb_profile.as_video_stream_profile().get_intrinsics()
        intrinsics = np.array([
            [raw_intrinsics.fx, 0, raw_intrinsics.ppx],
            [0, raw_intrinsics.fy, raw_intrinsics.ppy],
            [0, 0, 1]
        ])
        return intrinsics


    def close(self):
        # 新增：关闭相机管道（程序退出时调用）
        if self.pipeline is not None:
            self.pipeline.stop()
            print("相机已关闭")


if __name__ == '__main__':
    # 测试：可传入序列号（如 Camera(serial="819612070873")）
    mycamera = Camera()
    mycamera.get_data()
    mycamera.plot_image()
    print("相机内参：\n", mycamera.intrinsics)