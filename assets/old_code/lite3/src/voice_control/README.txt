ROS 2 Foxy 集成 USB 音响使用说明
=====================================

项目里已经准备好了两套扬声器使用方案：底层是 `src/voice_setting/play_audio.py`

---
## 一、纯 Python 方式

核心文件：`src/voice_setting/play_audio.py`

### 1. 依赖
```bash
sudo apt install mpg123 alsa-utils espeak
```
### 2. 命令行用法

播放本地音频文件：
```bash
python3 src/voice_setting/play_audio.py --file /home/pi/love.mp3 --card 3 --device 0
```

文字转语音：
```bash
python3 src/voice_setting/play_audio.py --text "你好，世界" --card 3 --lang zh --speed 160
```

使用配置文件（支持 YAML/JSON）：
```bash
python3 src/voice_setting/play_audio.py --config src/voice_setting/audio_config.yaml
```

### 3. 作为 Python 模块调用
也可以直接 import 函数使用：

```python
from play_audio import play_file, play_text

# 播放文件
play_file('/home/pi/love.mp3', card=3, device=0)

# TTS 播放文字
play_text('目标已到达', card=3, device=0, lang='zh', speed=160)
```




注意：实际使用前先用 `aplay -l` 或 `cat /proc/asound/cards` 确认 USB 音响的声卡号，把 `card` 改成正确的值。
