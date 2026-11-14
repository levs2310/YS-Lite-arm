#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从整机 STL + 关节表生成 URDF 与各 link 网格

关节表按 (x, z, y) 给出（Y 向上），内部换算成 three/URDF 的 (x, y, z)。
每个 link 的网格以「该 link 的关节位置」为原点重写（纯平移，不旋转），
因此全部关节为 0 时，整机与原始 STL 完全重合。

用法：
    python tools/build_urdf.py
    set ASM_STL=其他整机.stl && python tools/build_urdf.py
"""

import os
import struct

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PLAY = os.path.dirname(HERE)
OUT_DIR = os.path.join(PLAY, "public", "URDF")
MESH_DIR = os.path.join(OUT_DIR, "meshes")

SRC = os.environ.get("ASM_STL", r"C:\Users\23107\Desktop\装配 (2).stl")

# ---- 关节表：(名字, x, z, y, 轴, 下限°, 上限°) ----
JOINTS = [
    ("joint1_yaw",    0.0,    0.0,   52.8, "Y", -180, 180),
    ("joint2_arm",   20.0,   26.6,   69.0, "-Z",   -3, 150),   # 下限 -3°，不能更低
    ("joint3_fore", -70.0,   26.6,   69.0, "Z",   -3, 150),   # 下限 -3°，不能更低
    ("joint4_roll", -34.6,    0.0,  110.6, "-X", -180, 180),
    ("joint5_wrist",  1.6,   25.5,  110.6, "Z", -120, 120),
    ("joint6_roll",  74.8,    0.0,  110.6, "X", -180, 180),
]
AXIS = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0),
        "-X": (-1.0, 0.0, 0.0), "-Y": (0.0, -1.0, 0.0), "-Z": (0.0, 0.0, -1.0)}

LINK_NAMES = ["link0_base", "link1_yaw", "link2_arm", "link3_fore",
              "link4_roll", "link5_wrist", "link6_roll"]

# ---- 连通体 → link 的人工指定（按体积从大到小排序后的序号，从 1 开始） ----
#  序号: 质心大致位置                        归属
LINK_OF = {
    1: 0,   # (-5.8, 35.9, 0)   底座立柱          -> 底座
    2: 3,   # (-52.3,115.0, 0)  上层连杆(靠左)     -> link3
    3: 5,   # ( 57.2,116.4, 0)  上层连杆(靠右)     -> link5
    4: 1,   # ( 15.1, 69.1,-9.2) 关节2舵机（固定在座子上，不随 H 连杆转）
    5: 3,   # (-69.9, 73.9,-9.2) 关节3立柱         -> link3
    6: 4,   # ( -3.3,110.7,-9.2) 关节4附近横件     -> link4
    7: 2,   # (-24.8, 69.1, 0.3) 关节2↔关节3 横杆   -> link2
    8: 1,   # ( -0.1, 63.0, 0.3) 关节1上方立柱      -> link1
    9: 4,   # (-24.3,110.7, 0.2) 关节4横件          -> link4
    10: 5,  # (  9.5,110.6, 0.4) 关节5附近          -> link5
    11: 0,  # ( -9.7,  7.7,-0.4) 底板               -> 底座
    12: 3,  # (-65.9, 98.8, 0.3) 关节3→4 立柱       -> link3
    13: 5,  # ( 27.8,118.2, 0.1) 右侧连杆           -> link5
    14: 4,  # (-36.0,110.8, 0.1) 关节4转子          -> link4
    15: 0,  # ( -0.2, 51.4, 0.1) 关节1转子/法兰     -> 底座
    16: 6,  # ( 73.5,110.8, 0.0) 关节6转子          -> link6
    17: 6,  # ( 77.4,111.4, 0.2) 夹爪指             -> link6
    18: 1,  # (  2.0, 69.1,-18.0) 关节2舵机内薄片（跟随舵机，不随 H 连杆）
    19: 3,  # (-69.9, 87.0,-18.0) 关节3前侧件        -> link3
    20: 4,  # (-16.4,110.7,-18.0) 关节4前侧件        -> link4
}


def read_stl(path):
    with open(path, "rb") as f:
        head = f.read(84)
        n = struct.unpack("<I", head[80:84])[0]
        if os.path.getsize(path) != 84 + n * 50:
            raise ValueError("不是二进制 STL：%s" % path)
        raw = np.frombuffer(f.read(n * 50), dtype=np.uint8)
    return raw.reshape(n, 50)[:, :48].copy().view("<f4").reshape(n, 12)[:, 3:].reshape(n, 3, 3).astype(np.float64)


def write_stl(path, tris):
    n = len(tris)
    with open(path, "wb") as f:
        f.write(b"\0" * 80)
        f.write(struct.pack("<I", n))
        buf = np.zeros((n, 12), dtype="<f4")
        buf[:, 3:] = tris.reshape(n, 9)
        attr = np.zeros((n, 1), dtype="<u2")
        rec = np.hstack([buf.view(np.uint8).reshape(n, 48), attr.view(np.uint8).reshape(n, 2)])
        f.write(rec.tobytes())


def components(tri, tol=1e-3):
    flat = tri.reshape(-1, 3)
    key = np.round(flat / tol).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    vid = inv.reshape(-1, 3)
    parent = np.arange(vid.max() + 1)

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for a, b, c in vid:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra
        ra, rc = find(a), find(c)
        if ra != rc:
            parent[rc] = ra

    roots = np.array([find(v) for v in vid[:, 0]])
    groups = {}
    for i, r in enumerate(roots):
        groups.setdefault(int(r), []).append(i)
    return [tri[idx] for idx in groups.values()]


def main():
    os.makedirs(MESH_DIR, exist_ok=True)
    tri = read_stl(SRC)
    pos = [np.array([x, y, z]) for (_n, x, z, y, _a, _lo, _hi) in JOINTS]
    origins = [np.zeros(3)] + pos

    parts = [p for p in components(tri) if len(p) >= 100]
    parts.sort(key=lambda p: -len(p))
    print("整机 %d 面，拆出 %d 个连通体" % (len(tri), len(parts)))

    buckets = {i: [] for i in range(7)}
    for idx, p in enumerate(parts, 1):
        link = LINK_OF.get(idx)
        if link is None:
            c = p.reshape(-1, 3).mean(0)
            link = 0
            print("  !! #%02d 未指定，临时归 link0（质心 %.1f,%.1f,%.1f）" % (idx, *c))
        buckets[link].append(p)

    for i in range(7):
        if not buckets[i]:
            print("  !! %s 没有零件" % LINK_NAMES[i])
            continue
        tris = np.vstack(buckets[i])
        local = tris - origins[i]
        write_stl(os.path.join(MESH_DIR, LINK_NAMES[i] + ".STL"), local)
        mn, mx = local.reshape(-1, 3).min(0), local.reshape(-1, 3).max(0)
        print("  %-14s 零件%2d 面%6d  局部 X %.1f…%.1f  Y %.1f…%.1f  Z %.1f…%.1f"
              % (LINK_NAMES[i], len(buckets[i]), len(tris),
                 mn[0], mx[0], mn[1], mx[1], mn[2], mx[2]))

    # ---- URDF ----
    L = ['<?xml version="1.0"?>', '<robot name="my_arm">', '']
    for i in range(7):
        L += ['  <link name="%s">' % LINK_NAMES[i],
              '    <visual>',
              '      <origin xyz="0 0 0" rpy="0 0 0"/>',
              '      <geometry><mesh filename="meshes/%s.STL"/></geometry>' % LINK_NAMES[i],
              '    </visual>',
              '  </link>']

    for i, (name, x, z, y, axis, lo, hi) in enumerate(JOINTS):
        origin = pos[i] - origins[i]
        L += ['  <joint name="%s" type="revolute">' % name,
              '    <parent link="%s"/>' % LINK_NAMES[i],
              '    <child link="%s"/>' % LINK_NAMES[i + 1],
              '    <origin xyz="%.4f %.4f %.4f" rpy="0 0 0"/>' % tuple(origin),
              '    <axis xyz="%g %g %g"/>' % AXIS[axis],
              '    <limit lower="%.4f" upper="%.4f" effort="10" velocity="3"/>'
              % (np.deg2rad(lo), np.deg2rad(hi)),
              '  </joint>']
    L.append("</robot>")

    urdf = os.path.join(OUT_DIR, "robot.urdf")
    with open(urdf, "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")
    print("\nURDF: %s" % urdf)
    print("网格: %s" % MESH_DIR)


if __name__ == "__main__":
    main()
