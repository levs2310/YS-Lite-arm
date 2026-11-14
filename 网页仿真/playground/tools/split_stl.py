#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把「整机 STL」按连通壳体拆成多个零件 STL（坐标完全保持不变）

用法：
    python tools/split_stl.py public/STL/整机.STL
    python tools/split_stl.py public/STL/整机.STL --min-tris 200 --out public/STL/parts

原理：
    读入全部三角面 → 按「共享顶点」做并查集 → 每个连通分量就是一个实体零件
    → 各零件顶点坐标原样写出（不缩放、不平移、不旋转）

输出：
    out/part_01.STL, part_02.STL ...
    以及 out/split_report.txt（每个零件的三角数、包围盒、体积）
"""

import argparse
import os
import struct

import numpy as np


def read_stl(path):
    with open(path, "rb") as f:
        head = f.read(84)
        if len(head) < 84:
            raise ValueError("文件太小，不是 STL")
        n = struct.unpack("<I", head[80:84])[0]
        expect = 84 + n * 50
        actual = os.path.getsize(path)
        if actual != expect:
            raise ValueError(
                "不是二进制 STL（大小 %d，按头部应为 %d）。ASCII STL 请先在 SW/Shapr3D 里转成二进制。"
                % (actual, expect)
            )
        raw = np.frombuffer(f.read(n * 50), dtype=np.uint8)
    tri = raw.reshape(n, 50)[:, :48].copy().view("<f4").reshape(n, 12)[:, 3:].reshape(n, 3, 3)
    return tri.astype(np.float64)


def write_stl(path, tris):
    n = len(tris)
    with open(path, "wb") as f:
        f.write(b"\0" * 80)
        f.write(struct.pack("<I", n))
        for t in tris:
            f.write(struct.pack("<3f", 0.0, 0.0, 0.0))
            for v in t:
                f.write(struct.pack("<3f", float(v[0]), float(v[1]), float(v[2])))
            f.write(struct.pack("<H", 0))


def split(tris, tol=1e-3):
    """按共享顶点做并查集"""
    flat = tris.reshape(-1, 3)
    key = np.round(flat / tol).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    vid = inv.reshape(-1, 3)                 # 每个三角面的 3 个顶点索引

    parent = np.arange(vid.max() + 1)

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    for a, b, c in vid:
        union(a, b)
        union(a, c)

    roots = np.array([find(v) for v in vid[:, 0]])
    groups = {}
    for i, r in enumerate(roots):
        groups.setdefault(int(r), []).append(i)
    return [tris[idx] for idx in groups.values()]


def volume(tris):
    v0 = tris[:, 0]
    v1 = tris[:, 1]
    v2 = tris[:, 2]
    return abs(np.einsum("ij,ij->i", v0, np.cross(v1, v2)).sum() / 6.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stl")
    ap.add_argument("--out", default=None)
    ap.add_argument("--min-tris", type=int, default=50, help="少于这个三角数的碎面丢弃")
    args = ap.parse_args()

    path = args.stl
    out_dir = args.out or os.path.join(os.path.dirname(path), "parts")
    os.makedirs(out_dir, exist_ok=True)

    tris = read_stl(path)
    print("读入 %s：%d 个三角面" % (path, len(tris)))

    parts = split(tris)
    parts = [p for p in parts if len(p) >= args.min_tris]
    parts.sort(key=lambda p: -len(p))
    print("拆出 %d 个连通体（已丢弃 < %d 面的碎片）" % (len(parts), args.min_tris))

    lines = []
    for i, p in enumerate(parts, 1):
        name = "part_%02d.STL" % i
        write_stl(os.path.join(out_dir, name), p)
        flat = p.reshape(-1, 3)
        mn, mx = flat.min(axis=0), flat.max(axis=0)
        vol = volume(p)
        line = ("%-13s 三角 %6d  体积 %10.0f mm³  "
                "X %7.1f…%-7.1f Y %7.1f…%-7.1f Z %7.1f…%-7.1f"
                % (name, len(p), vol, mn[0], mx[0], mn[1], mx[1], mn[2], mx[2]))
        print("  " + line)
        lines.append(line)

    with open(os.path.join(out_dir, "split_report.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("\n输出目录：%s" % os.path.abspath(out_dir))


if __name__ == "__main__":
    main()
