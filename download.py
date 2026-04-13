import torch

# 检查是否能识别GPU
print("GPU是否可用：", torch.cuda.is_available())  # 必须返回True
print("当前使用设备：", torch.device("cuda" if torch.cuda.is_available() else "cpu"))  # 必须显示cuda
print("CUDA版本：", torch.version.cuda)  # 需≥11.7（Flash Attention要求）
print("PyTorch版本：", torch.__version__)