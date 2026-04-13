# UR机器人视觉抓取系统 - 代码结构与任务串联详解
## 一、系统架构总览（基于代码实际结构）
### 核心设计理念
采用「模块化+多线程」架构，遵循「配置中心化、职责单一化、线程安全化」原则，核心分为**配置层、初始化层、核心线程层、辅助层**，所有模块通过`VisionGraspSystem`实例实现数据共享与协同，支持语音/手动双控制模式。

### 模块依赖关系
```
main.py（核心入口）
├── init.py（初始化：机器人、相机、标定、模型）
├── 线程模块
│   ├── camera_thread.py（相机采集）
│   ├── detect_thread.py（SAM3检测）
│   ├── coord_thread.py（坐标转换）
│   ├── grasp_place_thread.py（抓取放置核心）
│   └── main_loop.py（主循环：按键+语音队列+画面显示）
├── 配置层：config.py（全局参数集中管理）
├── 语音层：detect_voice.py（语音识别+语义匹配+任务队列）
├── 辅助层
│   ├── constants.py（枚举定义）
│   ├── utils.py（工具函数）
│   ├── hand_in_eye_calibration.py（眼在手标定）
│   └── hand_out_eye_calibration.py（眼在手外标定）
└── 标定文件：eyeinhand_cam2end_12_5_v2.txt等（手眼标定参数）
```

## 二、核心模块详解（职责+关键逻辑）
### 1. 配置层：config.py
- **核心职责**：存储所有可配置参数，避免硬编码，支持快速修改
- **关键配置项**：
  - 机器人基础配置（IP、Home位、夹爪参数）
  - 双相机配置（串口、标定文件路径、深度缩放系数）
  - 高度配置（HEIGHT_CONFIG：按任务区分抓取/放置高度）
  - 语音任务配置（TASK_TARGET_CONFIG：3个核心任务+语音指令+高度关联键）
  - SAM3配置（置信度阈值、掩码参数）
  - 手检测配置（HAND_DETECTION_CONFIG：重试次数、超时、选中序号）
- **维护要点**：所有参数修改仅需改此处，无需动业务逻辑

### 2. 初始化层：init.py
- **核心职责**：统一初始化系统依赖组件，提供标准化接口
- **关键函数**：
  - `init_robot()`：连接UR机器人，移动到Home位
  - `init_cameras()`：初始化眼在手上/手外双相机，返回内参
  - `init_calibration()`：初始化双相机标定实例（复用机器人实例，避免重复创建）
  - `init_sam3_model()`：加载SAM3检测模型和处理器
- **依赖关系**：依赖config.py参数，输出实例给VisionGraspSystem

### 3. 核心入口：main.py
- **核心职责**：整合所有模块，创建全局`VisionGraspSystem`实例，启动线程
- **关键组件**：
  - `VisionGraspSystem`类：存储所有组件实例、共享数据、线程锁
  - 线程启动：相机采集、检测、坐标转换、语音识别线程
  - 语音任务处理：`_handle_voice_task()`（解析任务配置、设置检测目标、触发抓取线程）
  - 任务状态复位：`reset_task_status()`（任务完成/失败后重置状态）
- **维护要点**：系统启动流程、线程生命周期、语音任务入口均在此处

### 4. 线程层（核心执行单元）
#### （1）相机采集线程：camera_thread.py
- **职责**：持续采集双相机彩色图+深度图，存入共享缓存
- **关键逻辑**：
  - 循环调用相机`get_data()`，加锁更新`hi_img/hi_depth`（眼在手上）、`ho_img/ho_depth`（眼在手外）
  - 10ms休眠控制采集频率，异常捕获并打印
- **数据流向**：相机硬件 → 共享缓存 → detect_thread.py

#### （2）检测线程：detect_thread.py
- **职责**：响应检测触发，调用SAM3模型检测目标，输出检测结果（坐标、角度、掩码）
- **关键逻辑**：
  - 按任务阶段（GRASPING/PLACING/IDLE）动态切换检测目标（抓取/放置目标）
  - 眼在手上检测：计算目标角度（调用utils.calculate_angle）
  - 眼在手外检测：输出粗定位坐标
  - 检测结果加锁存入`hi_dets/ho_dets`，设置`has_detection_result=True`
- **触发条件**：`trigger_detect`标志置位（由主循环/抓取线程触发）
- **数据流向**：共享缓存图像 → SAM3检测 → 共享检测结果 → coord_thread.py

#### （3）坐标转换线程：coord_thread.py
- **职责**：将检测到的像素坐标转换为机器人基座坐标系三维坐标
- **关键逻辑**：
  - 按任务阶段确定目标类，匹配检测结果
  - 眼在手上：像素坐标→相机坐标→机器人坐标（依赖手眼标定）
  - 眼在手外：直接调用标定实例`pixel_to_robot_coords()`
  - 加锁更新`hi_xyz/ho_xyz`（目标三维坐标）
- **依赖关系**：依赖detect_thread检测结果、init.py标定实例
- **数据流向**：共享检测结果 → 坐标转换 → 共享三维坐标 → grasp_place_thread.py

#### （4）抓取放置线程：grasp_place_thread.py
- **职责**：完整执行抓取-放置流程，支持语音任务/手动任务、装配/手放置分支
- **核心流程**：
  1. 初始化：读取任务配置，按高度关联键获取抓取/放置高度（支持双层兜底）
  2. 抓取阶段：眼在手外粗定位 → 眼在手上精定位 → 角度调整 → 执行抓取
  3. 放置阶段：
     - 手放置（语音任务）：眼在手外检测手 → 移动到手上方 → 释放物体
     - 装配任务：检测装配基座 → 精定位 → 叠加装配偏移 → 放置
     - 普通放置：眼在手外+手内定位 → 放置
  4. 收尾：复位关节、回Home位、重置状态
- **线程安全**：使用`phase_lock/grasp_lock`避免并发，异常时紧急复位机器人
- **维护要点**：所有运动逻辑、高度配置、任务分支均在此处

#### （5）主循环线程：main_loop.py
- **职责**：处理用户交互（按键+语音队列）、实时显示画面
- **关键逻辑**：
  - 非阻塞读取语音任务队列，触发`_handle_voice_task()`
  - 按键控制：切换相机模式、切换目标、触发检测/抓取、退出
  - 画面显示：叠加检测结果、坐标、任务阶段、目标信息
- **数据流向**：用户输入 → 任务触发 → 共享标志位 → 对应线程响应

### 5. 语音识别层：detect_voice.py
- **职责**：语音指令采集、识别、语义匹配，输出任务队列
- **关键逻辑**：
  - VAD降噪→Vosk语音识别→SBERT语义匹配（匹配TASK_TARGET_CONFIG）
  - 用户确认机制（空格确认/M键放弃）
  - 匹配成功后将任务名放入队列（供main_loop读取）
- **依赖关系**：依赖config.py的语音模型路径、任务配置、匹配阈值
- **维护要点**：新增语音指令仅需扩展TASK_TARGET_CONFIG的`voice_instructions`

### 6. 辅助层
- **constants.py**：定义枚举（Mode：相机模式；TaskPhase：任务阶段；TaskType：任务类型）
- **utils.py**：工具函数（掩码绘制、检测图片保存、目标角度计算）
- **标定类**：`hand_in_eye_calibration.py`/`hand_out_eye_calibration.py`：实现相机→机器人坐标转换的标定逻辑

## 三、数据流转与线程同步
### 1. 共享数据（VisionGraspSystem实例存储）
| 数据类型       | 变量名                | 写入线程          | 读取线程          | 锁保护       |
|----------------|-----------------------|-------------------|-------------------|--------------|
| 图像数据       | hi_img/ho_img等       | camera_thread     | detect_thread     | data_lock    |
| 检测结果       | hi_dets/ho_dets等     | detect_thread     | coord_thread      | data_lock    |
| 三维坐标       | hi_xyz/ho_xyz         | coord_thread      | grasp_place_thread| data_lock    |
| 任务阶段       | current_task_phase    | grasp_place_thread| detect_thread     | phase_lock   |
| 检测触发标志   | trigger_detect        | main_loop/grasp_thread | detect_thread | data_lock    |
| 语音任务队列   | task_queue            | detect_voice      | main_loop         | 队列内置锁   |

### 2. 同步机制
- **锁机制**：`data_lock`（共享数据读写）、`phase_lock`（任务阶段切换）、`grasp_lock`（抓取状态控制），避免数据竞态
- **队列机制**：语音任务队列（`task_queue`）实现生产者-消费者模式（detect_voice生产，main_loop消费），解耦语音识别与主流程
- **标志位**：`detecting`（检测中）、`grasping`（抓取中）、`has_detection_result`（检测完成），实现线程间状态同步

## 四、核心任务串联流程
### 1. 语音任务流程（以「拿十字螺丝刀」为例）
```
1. 语音输入 → detect_voice.py：VAD降噪→Vosk识别→SBERT匹配→用户确认→放入task_queue
2. main_loop.py：非阻塞读取队列→调用_handle_voice_task()
3. 任务解析：从TASK_TARGET_CONFIG读取配置（grasp_prompt=Screwdriver、use_hand_placement=True、高度关联键）
4. 检测目标设置：更新current_grasp_class=Screwdriver
5. 触发抓取线程：启动grasp_place_thread()
6. 抓取流程：眼在手外粗定位→眼在手上精定位→抓取螺丝刀
7. 放置流程：detect_thread检测手→移动到手上方→释放→回Home位
8. 状态复位：调用reset_task_status()，等待新指令
```

### 2. 装配任务流程（「装配白色塑料件」）
```
1. 语音/按键触发→解析任务配置（task_type=assembly、grasp_prompt=white plastic object、base_prompt=red plastic object）
2. 抓取阶段：检测白色塑料件→精定位→抓取
3. 放置阶段：
   - detect_thread检测红色基座→coord_thread转换坐标
   - grasp_place_thread：精定位基座→叠加ASSEMBLY_OFFSET→下降放置
4. 收尾：复位关节→回Home位
```

### 3. 手动任务流程（按键触发）
```
1. 按键操作：a/s/d/f切换抓取目标→空格触发检测→g触发抓取
2. 检测流程：detect_thread输出检测结果→coord_thread转换坐标
3. 抓取放置：grasp_place_thread执行普通抓取-放置逻辑
4. 画面显示：实时更新检测结果、坐标、任务阶段
```

## 五、维护核心要点
### 1. 常见修改场景（无需改核心逻辑）
- 新增抓取目标：扩展config.py的GRASP_PROMPTS+TASK_TARGET_CONFIG，新增高度配置
- 调整检测精度：修改config.py的SAM3_CONF_THRESH（置信度阈值）
- 调整运动参数：修改config.py的GRIP_SPEED/GRIP_FORCE/SAFE_HEIGHT
- 新增语音指令：扩展TASK_TARGET_CONFIG中对应任务的`voice_instructions`
- 调整放置高度：修改HEIGHT_CONFIG中对应任务的`safe/final`值

### 2. 关键风险点
- 线程安全：所有共享数据修改必须加锁，新增共享变量需补充锁保护
- 机器人复位：异常处理中必须保留紧急回Home位+打开夹爪逻辑，避免设备损坏
- 标定文件：修改相机/机器人位置后，需重新标定并更新标定文件

### 3. 调试技巧
- 日志打印：各线程均有明确打印（检测结果、坐标、任务阶段），可通过终端定位问题
- 图片保存：检测结果自动保存到detection_results文件夹，便于验证检测效果
- 状态查看：画面实时显示当前目标、坐标、任务阶段，快速判断系统状态

## 六、总结
系统核心是「多线程协同+配置中心化」，所有业务逻辑围绕`VisionGraspSystem`实例展开，数据通过共享变量流转，线程通过锁+队列同步。维护时需重点关注：
1. 任务流程修改→grasp_place_thread.py
2. 配置参数修改→config.py
3. 交互逻辑修改→main_loop.py/detect_voice.py
4. 检测/坐标逻辑修改→detect_thread.py/coord_thread.py
5. 初始化逻辑修改→init.py

所有模块职责单一，修改时只需聚焦对应文件，无需改动其他模块，符合「高内聚低耦合」设计，便于长期维护。