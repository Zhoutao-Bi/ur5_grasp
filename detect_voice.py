import json
import os
import warnings
import pyaudio
import webrtcvad
import torch
from vosk import Model, KaldiRecognizer, SetLogLevel
from sentence_transformers import SentenceTransformer, util
# 导入线程和队列模块
import queue
import threading
# 导入按键输入相关模块（跨平台兼容）
import sys
# 关键：导入统一配置模块
import config

# 跨平台按键输入函数（无需回车，支持空格、m键）
def get_key():
    """
    读取单个按键输入（跨平台）：
    - Windows：使用msvcrt
    - Linux/macOS：使用termios/tty
    返回：按下的字符（小写）
    """
    if sys.platform == 'win32':
        import msvcrt
        while True:
            if msvcrt.kbhit():
                key = msvcrt.getch().decode('utf-8', errors='ignore').lower()
                return key
    else:
        import termios
        import tty
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1).lower()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

# ---------------------- 1. SBERT语义匹配类（核心修改：从config读取配置） ----------------------
class ChineseTaskMatcher:
    # 【修改1：简化初始化参数，从config读取模型路径和阈值】
    def __init__(self):
        warnings.filterwarnings("ignore", category=UserWarning, module="transformers.models.bert.modeling_bert")
        warnings.filterwarnings("ignore", category=UserWarning, module="torch.nn.functional")

        # 【修改2：从config读取SBERT模型路径】
        local_model_path = config.SBERT_MODEL_PATH
        # 加载模型
        print(f"\n正在加载SBERT语义匹配模型：{local_model_path}")
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = SentenceTransformer(local_model_path, device=self.device)
        print(f"SBERT使用设备：{self.device}")

        # 【修改3：从config读取语音匹配阈值】
        self.threshold = config.VOICE_MATCH_THRESHOLD

        # 【修改4：从config读取任务配置，移除硬编码的tasks】
        self.task_config = config.TASK_TARGET_CONFIG  # 整合后的任务配置（含语音指令）

        # 构建「指令-任务」映射（逻辑不变，数据源改为config）
        self.all_instructions = []  # 所有单独的指令
        self.instruction_to_task = {}  # 指令→任务名称的映射
        for task_name, task_info in self.task_config.items():
            # 读取该任务的语音指令列表（兜底：空列表）
            instructions = task_info.get("voice_instructions", [])
            for instr in instructions:
                self.all_instructions.append(instr)
                self.instruction_to_task[instr] = task_name

        # 对每个单独指令编码（添加容错：避免空列表）
        if self.all_instructions:
            self.instruction_embeddings = self.model.encode(
                self.all_instructions,
                convert_to_tensor=True,
                device=self.device
            )
        else:
            self.instruction_embeddings = None
            print("⚠️ 从config中未读取到任何语音指令，语义匹配功能将失效")

    # ---------------------- 匹配任务方法（逻辑不变） ----------------------
    def match_task(self, input_text):
        """核心：匹配输入文本与所有单独指令的相似度，取最高，返回任务名（字符串）或None"""
        if not input_text.strip() or self.instruction_embeddings is None:
            return None  # 空文本或无指令时返回None

        # 计算输入文本向量
        input_embedding = self.model.encode(
            input_text,
            convert_to_tensor=True,
            device=self.device
        )

        # 计算输入与所有指令的相似度
        similarities = util.cos_sim(input_embedding, self.instruction_embeddings)[0]
        max_idx = similarities.argmax().item()
        max_sim = similarities[max_idx].item()
        best_instruction = self.all_instructions[max_idx]  # 最匹配的单个指令
        best_task = self.instruction_to_task[best_instruction]  # 对应的任务名称

        if max_sim >= self.threshold:
            return best_task  # 仅返回任务名
        else:
            return None  # 未匹配到返回None

    # ---------------------- 保留打印版匹配方法（用于测试） ----------------------
    def match_task_with_print(self, input_text):
        """原有版本，返回描述性字符串，用于测试/打印"""
        if not input_text.strip():
            return "⚠️ 识别到空文本，无法匹配任务"
        if self.instruction_embeddings is None:
            return "⚠️ 无可用的语音指令，无法匹配任务"

        input_embedding = self.model.encode(
            input_text,
            convert_to_tensor=True,
            device=self.device
        )

        similarities = util.cos_sim(input_embedding, self.instruction_embeddings)[0]
        max_idx = similarities.argmax().item()
        max_sim = similarities[max_idx].item()
        best_instruction = self.all_instructions[max_idx]
        best_task = self.instruction_to_task[best_instruction]

        if max_sim >= self.threshold:
            return (
                "✅ 语义匹配结果\n"
                f"输入文本：{input_text}\n"
                f"最匹配指令：{best_instruction}\n"
                f"对应任务：{best_task}\n"
                f"相似度：{max_sim:.2f}（≥阈值{self.threshold}）"
            )
        else:
            return (
                "❌ 未匹配到任务\n"
                f"输入文本：{input_text}\n"
                f"最接近指令：{best_instruction}（对应任务：{best_task}）\n"
                f"相似度：{max_sim:.2f} < 阈值{self.threshold}"
            )

# ---------------------- 2. VAD+Vosk语音识别类（逻辑不变，参数从config读取） ----------------------
class VADVoskASR:
    def __init__(self, vosk_model_path, sbert_matcher, task_queue, vad_mode=2, mic_index=None):
        self.sbert_matcher = sbert_matcher
        self.task_queue = task_queue  # 线程安全的任务队列
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(vad_mode)
        self.vosk_model = self._load_vosk_model(vosk_model_path)
        self.sample_rate = 16000
        self.frame_duration = 20
        self.frame_size = int(self.sample_rate * self.frame_duration / 1000)
        self.chunk_size = self.frame_size * 5
        self.voice_buffer = []
        self.mic_index = mic_index
        self.is_running = True  # 控制识别线程的运行状态

    def _load_vosk_model(self, model_path):
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Vosk模型路径不存在：{model_path}")
        SetLogLevel(-1)
        try:
            print(f"正在加载Vosk语音识别模型：{model_path}")
            return Model(model_path)
        except Exception as e:
            raise RuntimeError(f"Vosk模型加载失败：{str(e)}")

    def _is_voice_frame(self, frame):
        return self.vad.is_speech(frame, self.sample_rate)

    def start_recognition(self):
        p = pyaudio.PyAudio()
        stream = p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.sample_rate,
            input=True,
            input_device_index=self.mic_index,
            frames_per_buffer=self.chunk_size
        )

        rec = KaldiRecognizer(self.vosk_model, self.sample_rate)
        print(
            f"\n✅ 所有模块加载完成！\n"
            f"当前使用麦克风：{'默认设备' if self.mic_index is None else f'编号{self.mic_index}'}\n"
            f"提示：开始说话（按Ctrl+C停止）\n"
            "支持的任务：拿十字螺丝刀、拿胶布、装配白色塑料件\n"  # 【修改：修正任务描述】
            "----------------------------------------"
        )

        try:
            while self.is_running:  # 用is_running控制循环
                # 处理音频流溢出异常
                data = stream.read(self.chunk_size, exception_on_overflow=False)
                frames = [data[i:i + self.frame_size * 2] for i in range(0, len(data), self.frame_size * 2)]

                for frame in frames:
                    if self._is_voice_frame(frame):
                        self.voice_buffer.append(frame)
                        rec.AcceptWaveform(frame)
                    else:
                        if self.voice_buffer:
                            res = json.loads(rec.FinalResult())

                                                 
                            if recognized_text := res.get("text"):
                                # 1. 识别并打印匹配信息
                                print(f"\n🗣️ Vosk识别结果：{recognized_text}")
                                match_info = self.sbert_matcher.match_task_with_print(recognized_text)
                                print(match_info)

                                # 2. 获取匹配的任务名
                                matched_task = self.sbert_matcher.match_task(recognized_text)

                                # 3. 自动判断：只要匹配成功（即过了阈值），直接入队
                                if matched_task:
                                    print(f"✅ 匹配达标，自动放入队列：{matched_task}")
                                    self.task_queue.put(matched_task)
                                else:
                                    print("❌ 相似度未达标，已忽略该指令")

                                # 4. 清空缓冲区，准备下一次识别
                                self.voice_buffer = []
                                print("----------------------------------------")
                            else:
                                # 无识别文本，直接清空缓冲区
                                self.voice_buffer = []
        except KeyboardInterrupt:
            print("\n\n🛑 正在停止识别...")
            if self.voice_buffer:
                res = json.loads(rec.FinalResult())
                if recognized_text := res.get("text"):
                    print(f"\n🗣️ Vosk最终识别结果：{recognized_text}")
                    print(self.sbert_matcher.match_task_with_print(recognized_text))
        except Exception as e:
            print(f"\n❌ 语音识别过程中出错：{str(e)}")
        finally:
            stream.stop_stream()
            stream.close()
            p.terminate()
            print("\n✅ 所有资源已释放，程序退出")

    def stop_recognition(self):
        """停止语音识别线程"""
        self.is_running = False

# ---------------------- 3. 主程序（核心修改：所有参数从config读取） ----------------------
if __name__ == "__main__":
    try:
        # 【修改：从config读取所有参数】
        VOSK_MODEL_PATH = config.VOSK_MODEL_PATH
        SBERT_MODEL_PATH = config.SBERT_MODEL_PATH
        MIC_INDEX = config.MIC_INDEX
        VAD_MODE = config.VAD_MODE

        # 创建任务队列（最大存储10个任务，避免堆积）
        task_queue = queue.Queue(maxsize=10)

        # 初始化SBERT匹配器（无需传参数，内部从config读取）
        sbert_matcher = ChineseTaskMatcher()
        # 初始化VAD+Vosk语音识别系统
        asr_system = VADVoskASR(
            vosk_model_path=VOSK_MODEL_PATH,
            sbert_matcher=sbert_matcher,
            task_queue=task_queue,
            vad_mode=VAD_MODE,
            mic_index=MIC_INDEX
        )

        # 启动识别
        asr_system.start_recognition()

        # （可选）测试队列读取：取消注释即可
        # while True:
        #     try:
        #         task = task_queue.get(block=True, timeout=1)
        #         print(f"\n📥 从队列读取到任务：{task}")
        #     except queue.Empty:
        #         continue

    except Exception as e:
        print(f"\n❌ 程序运行失败：{str(e)}")