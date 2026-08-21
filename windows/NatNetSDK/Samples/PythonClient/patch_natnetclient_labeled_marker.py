#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
patch_natnetclient_labeled_marker.py

修補 NatNet SDK 4.1.1 Samples/PythonClient/NatNetClient.py，
讓它支援：

    client.labeled_marker_listener = callback

callback 會在每一顆 labeled marker 被解析時被呼叫：

    callback(tmp_id, pos, size, param, residual)

其中：
    tmp_id    : NatNet marker id，可用 tmp_id >> 16 得到 model_id，
                tmp_id & 0xFFFF 得到 marker_id
    pos       : (x, y, z)，通常單位 meter
    size      : marker size
    param     : NatNet marker parameter
    residual  : residual/error
"""

from pathlib import Path
import shutil
import sys


def patch_natnetclient(path: Path):
    text = path.read_text(encoding="utf-8")

    changed = False

    anchor = "        self.rigid_body_listener = None\n        self.new_frame_listener  = None\n"
    replacement = (
        "        self.rigid_body_listener = None\n"
        "        self.new_frame_listener  = None\n"
        "        # Added by patch: callback for individual labeled markers.\n"
        "        # Signature: callback(tmp_id, pos, size, param, residual)\n"
        "        self.labeled_marker_listener = None\n"
    )

    if "self.labeled_marker_listener" not in text:
        if anchor not in text:
            raise RuntimeError("找不到 __init__ 中 rigid_body_listener/new_frame_listener 區塊，無法自動插入。")
        text = text.replace(anchor, replacement, 1)
        changed = True
        print("[PATCH] 已加入 self.labeled_marker_listener = None")
    else:
        print("[SKIP] self.labeled_marker_listener 已存在")

    anchor2 = (
        "                labeled_marker = MoCapData.LabeledMarker(tmp_id,pos,size,param, residual)\n"
        "                labeled_marker_data.add_labeled_marker(labeled_marker)\n"
    )

    callback_block = (
        "                labeled_marker = MoCapData.LabeledMarker(tmp_id,pos,size,param, residual)\n"
        "                labeled_marker_data.add_labeled_marker(labeled_marker)\n"
        "\n"
        "                # Added by patch: notify external listener for each labeled marker.\n"
        "                if self.labeled_marker_listener is not None:\n"
        "                    try:\n"
        "                        self.labeled_marker_listener(tmp_id, pos, size, param, residual)\n"
        "                    except Exception as e:\n"
        "                        print(f\"ERROR: labeled_marker_listener callback failed: {e}\")\n"
    )

    if "labeled_marker_listener(tmp_id, pos, size, param, residual)" not in text:
        if anchor2 not in text:
            raise RuntimeError("找不到 labeled_marker 建立區塊，無法自動插入 callback。")
        text = text.replace(anchor2, callback_block, 1)
        changed = True
        print("[PATCH] 已加入 labeled_marker_listener callback 呼叫")
    else:
        print("[SKIP] labeled_marker_listener callback 已存在")

    if not changed:
        print("NatNetClient.py 看起來已經修補過，不需要改。")
        return

    backup = path.with_suffix(path.suffix + ".bak")
    if not backup.exists():
        shutil.copy2(path, backup)
        print(f"[BACKUP] 已備份：{backup}")
    else:
        print(f"[BACKUP] 備份已存在：{backup}")

    path.write_text(text, encoding="utf-8")
    print(f"[DONE] 已修補：{path}")


def main():
    if len(sys.argv) >= 2:
        path = Path(sys.argv[1])
    else:
        path = Path("NatNetClient.py")

    if not path.exists():
        raise FileNotFoundError(f"找不到檔案：{path}")

    patch_natnetclient(path)


if __name__ == "__main__":
    main()
