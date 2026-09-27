"""验证环境配置脚本 - 检查 .env 读取和模块导入"""
import os
import sys

print("=" * 50)
print("  环境验证脚本")
print("=" * 50)

# 1. Python 版本
print(f"\n[1] Python 版本: {sys.version}")
print(f"    Python 路径: {sys.executable}")

# 2. 加载 .env
from dotenv import load_dotenv
env_path = os.path.join(os.path.dirname(__file__), ".env")
print(f"\n[2] .env 文件路径: {env_path}")
print(f"    .env 文件存在: {os.path.exists(env_path)}")

result = load_dotenv(env_path)
print(f"    load_dotenv() 返回: {result}")

# 3. 检查配置项
API_KEY = os.getenv("DASHSCOPE_API_KEY", "")
WORKSPACE_ID = os.getenv("WORKSPACE_ID", "")
APP_ID = os.getenv("APP_ID", "")
VOICE = os.getenv("VOICE", "longxiaochun_v2")
DOWNSTREAM_SAMPLE_RATE = os.getenv("DOWNSTREAM_SAMPLE_RATE", "24000")

print(f"\n[3] 配置读取结果:")
print(f"    DASHSCOPE_API_KEY: {API_KEY[:12]}...{API_KEY[-8:]}" if len(API_KEY) > 20 else f"    DASHSCOPE_API_KEY: '{API_KEY}' (太短!)")
print(f"    WORKSPACE_ID:      {WORKSPACE_ID}")
print(f"    APP_ID:            {APP_ID}")
print(f"    VOICE:             {VOICE}")
print(f"    DOWNSTREAM_SAMPLE_RATE: {DOWNSTREAM_SAMPLE_RATE}")

errors = []
if not API_KEY:
    errors.append("DASHSCOPE_API_KEY 为空!")
if not WORKSPACE_ID:
    errors.append("WORKSPACE_ID 为空!")
if not APP_ID:
    errors.append("APP_ID 为空!")

# 4. 检查依赖导入
print(f"\n[4] 依赖导入检查:")
modules = {
    "dashscope": "dashscope",
    "pyaudio": "pyaudio",
    "numpy": "numpy",
    "sounddevice": "sounddevice",
    "cv2": "cv2",
    "dotenv": "python-dotenv",
}

for mod_name, pkg_name in modules.items():
    try:
        mod = __import__(mod_name)
        ver = getattr(mod, "__version__", getattr(mod, "VERSION", "N/A"))
        print(f"    {pkg_name:20s} : OK (版本 {ver})")
    except ImportError as e:
        print(f"    {pkg_name:20s} : 失败! {e}")
        errors.append(f"{pkg_name} 导入失败: {e}")

# 5. 检查 dashscope.multimodal 子模块
print(f"\n[5] dashscope.multimodal 导入检查:")
try:
    from dashscope.multimodal import (
        MultiModalDialog,
        MultiModalCallback,
        RequestParameters,
        Upstream,
        Downstream,
        ClientInfo,
        Device,
        BizParams,
        RequestToRespondParameters,
        DialogState,
    )
    print(f"    MultiModalDialog          : OK")
    print(f"    MultiModalCallback        : OK")
    print(f"    RequestParameters         : OK")
    print(f"    Upstream                  : OK")
    print(f"    Downstream                : OK")
    print(f"    ClientInfo                : OK")
    print(f"    Device                    : OK")
    print(f"    BizParams                 : OK")
    print(f"    RequestToRespondParameters: OK")
    print(f"    DialogState               : OK")
except ImportError as e:
    print(f"    导入失败: {e}")
    errors.append(f"dashscope.multimodal 导入失败: {e}")

# 6. 日志文件路径
log_file = os.path.join(os.path.dirname(__file__), "mmdialog.log")
print(f"\n[6] 日志文件路径: {log_file}")
try:
    with open(log_file, "a", encoding="utf-8") as f:
        f.write("")
    print(f"    日志文件可写: OK")
except Exception as e:
    print(f"    日志文件不可写: {e}")
    errors.append(f"日志文件不可写: {e}")

# 总结
print(f"\n{'=' * 50}")
if errors:
    print(f"  验证失败! 发现 {len(errors)} 个问题:")
    for e in errors:
        print(f"    - {e}")
    sys.exit(1)
else:
    print(f"  全部通过! 环境配置正确, 可以运行 demo_multimodal_dialog.py")
    print(f"  运行命令:")
    print(f"    conda activate mmdialog")
    print(f"    cd D:\\pycode\\qwen")
    print(f"    python demo_multimodal_dialog.py")
print("=" * 50)
