#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
USB 音响动态播放脚本
支持：
  1. 播放本地 MP3/WAV 文件
  2. 文字转语音（TTS，依赖 espeak）
  3. 通过命令行参数或 YAML/JSON 配置文件指定声卡和音频

用法示例：
  python3 play_audio.py --file /home/pi/love.mp3 --card 3
  python3 play_audio.py --text "你好，世界" --card 3 --lang zh
  python3 play_audio.py --config audio_config.yaml
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
import yaml
from pathlib import Path


def run_cmd(cmd, check=True, dry_run=False):
    """执行 shell 命令并返回结果。"""
    print(f"[RUN] {' '.join(cmd)}")
    if dry_run:
        return None
    try:
        result = subprocess.run(cmd, check=check, capture_output=True, text=True)
        if result.stdout:
            print(result.stdout.strip())
        return result
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] 命令执行失败: {' '.join(cmd)}")
        if e.stderr:
            print(e.stderr.strip())
        raise


def require_command(name):
    """检查系统命令是否存在。"""
    if shutil.which(name) is None:
        print(f"[ERROR] 缺少命令: {name}，请先安装: sudo apt install {name}")
        sys.exit(1)


def get_ext(path):
    return Path(path).suffix.lower()


def play_file(filepath, card, device=0, dry_run=False):
    """根据扩展名选择 mpg123 或 aplay 播放本地音频。"""
    if not dry_run and not os.path.isfile(filepath):
        print(f"[ERROR] 文件不存在: {filepath}")
        sys.exit(1)

    ext = get_ext(filepath)
    if ext == ".mp3":
        if not dry_run:
            require_command("mpg123")
        run_cmd(["mpg123", "-a", f"hw:CARD={card}", filepath], dry_run=dry_run)
    elif ext == ".wav":
        if not dry_run:
            require_command("aplay")
        run_cmd(["aplay", "-D", f"plughw:{card},{device}", filepath], dry_run=dry_run)
    else:
        print(f"[ERROR] 不支持的音频格式: {ext}，目前仅支持 .mp3 和 .wav")
        sys.exit(1)


def play_text(text, card, device=0, lang="zh", speed=160, dry_run=False):
    """使用 espeak 合成语音并播放。"""
    if not dry_run:
        require_command("espeak")
        require_command("aplay")

    # 生成临时 wav 文件
    if dry_run:
        tmp_path = "/tmp/dry_run_tts.wav"
    else:
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            tmp_path = tmp.name

    try:
        # espeak: -v 语言, -s 语速, -w 输出 wav
        run_cmd([
            "espeak",
            text,
            "-v", lang,
            "-s", str(speed),
            "-w", tmp_path
        ], dry_run=dry_run)
        run_cmd(["aplay", "-D", f"plughw:{card},{device}", tmp_path], dry_run=dry_run)
    finally:
        if not dry_run and os.path.exists(tmp_path):
            os.remove(tmp_path)


def load_config(path):
    """加载 YAML 或 JSON 配置文件。"""
    if not os.path.isfile(path):
        print(f"[ERROR] 配置文件不存在: {path}")
        sys.exit(1)

    ext = get_ext(path)
    with open(path, "r", encoding="utf-8") as f:
        if ext in (".yaml", ".yml"):
            return yaml.safe_load(f)
        elif ext == ".json":
            return json.load(f)
        else:
            print(f"[ERROR] 配置文件格式不支持: {ext}，请使用 .yaml/.yml/.json")
            sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="USB 音响动态播放工具")
    parser.add_argument("--file", "-f", help="要播放的本地音频文件路径（.mp3 或 .wav）")
    parser.add_argument("--text", "-t", help="要转换为语音并播放的文字")
    parser.add_argument("--card", "-c", type=int, default=3, help="USB 音响声卡号，默认 3")
    parser.add_argument("--device", "-d", type=int, default=0, help="设备号，默认 0")
    parser.add_argument("--lang", "-l", default="zh", help="TTS 语言，默认 zh（中文）")
    parser.add_argument("--speed", "-s", type=int, default=160, help="TTS 语速，默认 160")
    parser.add_argument("--config", "-C", help="YAML/JSON 配置文件路径")
    parser.add_argument("--dry-run", action="store_true", help="只打印要执行的命令，不真正播放")

    args = parser.parse_args()

    # 如果指定了配置文件，用配置文件覆盖默认值
    if args.config:
        cfg = load_config(args.config)
        if not cfg:
            cfg = {}
        args.card = cfg.get("card", args.card)
        args.device = cfg.get("device", args.device)
        args.lang = cfg.get("lang", args.lang)
        args.speed = cfg.get("speed", args.speed)
        args.file = cfg.get("file", args.file)
        args.text = cfg.get("text", args.text)

    # 校验输入
    if not args.file and not args.text:
        parser.print_help()
        print("\n[ERROR] 必须提供 --file 或 --text 之一，或使用 --config 指定配置文件")
        sys.exit(1)

    if args.file and args.text:
        print("[WARN] 同时提供了 --file 和 --text，优先播放文件")
        args.text = None

    # 执行播放
    if args.file:
        play_file(args.file, args.card, args.device, dry_run=args.dry_run)
    else:
        play_text(args.text, args.card, args.device, args.lang, args.speed, dry_run=args.dry_run)

    if args.dry_run:
        print("[DRY-RUN] 仅打印命令，未实际播放")
    else:
        print("[OK] 播放完成")


if __name__ == "__main__":
    main()
