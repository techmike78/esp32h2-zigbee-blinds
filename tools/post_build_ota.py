# PlatformIO post-build хук (extra_scripts в platformio.ini): после каждой сборки пересобирает
# OTA-образ для Z2M из СВЕЖЕГО firmware.bin, чтобы не запускать tools/make_ota.py руками.
# Кладёт файлы (см. make_ota.py) — НЕ копирует их на сервер Z2M, это отдельный шаг (см. main.c,
# комментарий в начале файла, и roadmap.md, Этап 21).
Import("env")

import subprocess
import sys
from pathlib import Path


def make_ota(source, target, env):
    # SCons exec's этот файл, а не импортирует как модуль — __file__ здесь не определён,
    # поэтому путь берём из окружения PlatformIO.
    tools_dir = Path(env["PROJECT_DIR"]) / "tools"
    firmware_bin = Path(target[0].get_abspath())
    result = subprocess.run(
        [sys.executable, str(tools_dir / "make_ota.py"), "--bin", str(firmware_bin)],
        cwd=str(tools_dir.parent),
    )
    if result.returncode != 0:
        print("!!! make_ota.py failed — OTA image was NOT (re)generated, firmware.bin itself is fine")


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", make_ota)
