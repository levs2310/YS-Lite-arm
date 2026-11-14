# 网页仿真（精简版）

保留 **STL / URDF 加载 + 正向运动学 + 逆运动学**，以及与上位机（`servo_control.py`）的 **WebSocket 双向联动**。
旧版全部代码备份在 `_legacy/`（不入库）。

## 启动

```bat
cd 网页仿真\playground
npm install        :: 首次
npm run dev
```

浏览器会打开 http://127.0.0.1:5173

也可以由上位机界面的「打开网页仿真」按钮拉起：程序会先 `npm run build` 的产物（`dist/`）用内建静态服务挂在 http://127.0.0.1:8766 并自动开浏览器，同时把 WebSocket 桥挂在 8765。

## 界面

| 区域 | 作用 |
|---|---|
| 六轴关节 | 按 URDF 自动生成滑条（旋转关节用 °，移动关节用 mm），限位取自 `<limit>` |
| 末端坐标 | 实时显示末端点世界坐标（mm） |
| 逆解 | 「当前末端逆解姿态」按当前末端位置求解一次，数值阻尼最小二乘，失败会显示残差 |
| 上位机 | 显示与上位机的连接状态与当前联动方向 |

末端点 = URDF 树里最深的那条 link，不用改代码。

## 与上位机联动

上位机 `host-app/servo_control.py` 内置了 WebSocket 桥（8765）和静态服务（8766），页面向 `ws://<host>:8765` 连接。

| 消息 | 方向 | 内容 |
|---|---|---|
| `motor_angles` | 上位机 → 页面 | `{"angles":[6 个值]}`，前 5 个是关节角，第 6 个是夹爪开合 0–400 |
| `sim_angles` | 页面 → 上位机 | 同上，附 `t` 时间戳；只在用户刚拖动过时发送 |
| `sim_mode` | 上位机 → 页面 | 当前联动方向：`off` / `to_sim` / `from_sim` |

联动方向在上位机侧选择（默认关闭）。**联动开启时，页面的「当前末端逆解姿态」按钮会被禁用**——一次逆解会把六轴同时推到新姿态，联动状态下等于直接驱动真机，电流冲击很大。

上位机在「仿真 → 真机」方向做了限流：最多 2 路轴同时运动、逐路下发、45°/s、单次不超过 700 ms，避免电源过流导致总线超时。

## 先看 STL 装配（不写 URDF）

把 STL 放进 public/STL/，然后：

`at
python tools/make_stl_manifest.py     :: 生成/更新 manifest.json
npm run dev
`

页面点 **「加载 STL 目录」**，所有零件按各自建模坐标叠在一起，就是装配位置。
每个零件有显示开关和独立颜色，方便核对。

> **模型不做任何缩放 / 旋转 / 平移**：STL 按你导出的原始坐标摆好，就是装配位置。
> 只有相机、灯光、网格会按模型尺寸自适应（Z 轴朝上、XY 为地面）。

## 已生成的 URDF

public/URDF/robot.urdf + meshes/link*_*.STL

- 7 个 link、6 个 revolute 关节，关节位置/轴按你给的表
- 每个 link 网格以该关节位置为原点（纯平移），**全零位时整机与 assembly.STL 完全重合**（实测偏差 0.0000 mm）
- 打开页面会自动加载它

重新生成（改了关节表或换了整机 STL 之后）：

`at
python tools\build_urdf.py
`

关节表在脚本顶部 JOINTS，连通体归属在 LINK_OF。

## 换新模型（关键）

### 方式 A：放进 public 目录（推荐）

```
public/URDF/你的模型.urdf
public/URDF/meshes/1.STL ...
`

> `public/STL/assembly.STL` 与仓库里的 `model/装配.stl` 是同一个文件，为避免重复占 3 MB **没有入库**；
> 需要看整机装配预览时，把它复制回 `public/STL/` 即可。

URDF 里引用网格时用相对路径：

```xml
<visual>
  <geometry>
    <mesh filename="meshes/1.STL" scale="0.001 0.001 0.001"/>
  </geometry>
</visual>
```

> `scale="0.001"` 用于 **毫米建模的 STL**（绝大多数 CAD 导出）。若 STL 本身以米为单位就删掉 scale。

然后在页面「模型」里把路径改成 `URDF/你的模型.urdf`，点加载。

### 方式 B：本地临时看

「模型」区域的文件选择框里，**一次选中** `.urdf` + 它引用的所有 `.STL`，
页面会用文件名匹配，不需要起服务器也能看。

## 坐标系（重要）

场景默认 **Y 轴朝上**（three.js / SolidWorks 惯例）。
如果你的模型是 Z-up（Shapr3D / Fusion / ROS），在「坐标系」下拉里切成 Z 向上 即可。

说明两点：

- **正逆解显示的数字永远是 URDF 原始坐标**（模型自身坐标系），所以你在 Shapr3D 里量的是多少，这里就是多少
- 切换「坐标系」只改变**模型在屏幕上的摆法**，不改数字

URDF/three.js 轴向对照（默认 Y-up 模式）：

| 轴 | 含义 |
|---|---|
| X | 右 |
| Y | **上** |
| Z | 朝向观察者 |

## 正逆解说明

- **正解**：直接读 three.js 的关节世界矩阵，所以和 URDF 完全一致，不写死任何机型
- **逆解**：3 自由度位置逆解（不控姿态）
  - 数值雅可比 + 阻尼最小二乘（Levenberg–Marquardt 风格）
  - 每步限幅 0.3 rad，超限自动裁剪到关节限位
  - 收敛判据 0.5 mm，最多 120 次迭代
  - 只优化末端**位置**，姿态不做约束

## 目录

```
playground/
  index.html          界面
  src/main.js         场景 / 加载 / 正逆解 / 与上位机联动
  vite.config.js      仅静态服务
  public/URDF/        robot.urdf + meshes/link*_*.STL（当前模型）
  public/STL/         整机装配预览用的 STL 目录（assembly.STL 未入库，见上）
  tools/              从整机 STL 拆件、生成 URDF、生成 manifest 的脚本
  _legacy/            旧版全部代码（备份，不入库）
```
