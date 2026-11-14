#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""扫描 public/STL 下所有 .STL，重新生成 manifest.json

新增/删除 STL 后运行一次：
    python tools/make_stl_manifest.py
"""

import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
STL_DIR = os.path.join(HERE, "..", "public", "STL")
OUT = os.path.join(STL_DIR, "manifest.json")

names = sorted(
    (f for f in os.listdir(STL_DIR) if f.lower().endswith(".stl")),
    key=lambda s: (len(s), s),
)
with open(OUT, "w", encoding="utf-8") as f:
    json.dump(names, f, ensure_ascii=False, indent=2)

print("已写入 %s" % os.path.normpath(OUT))
for n in names:
    print("  -", n)
