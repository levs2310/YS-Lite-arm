# YS-Lite-arm

远山 Lite 六轴机械臂开源资料：上位机控制程序与结构件 3D 打印模型。

```text
host-app/    上位机（Python + tkinter，Windows）
model/       结构件模型（STL）
```

## 上位机

```bash
python host-app/servo_control.py
```

- 连接：USB-TTL 单线半双工，默认 `115200-8-N-1`，串口号在界面里选
- `servo_control.py`：主程序。六轴角度 / PWM 双向换算下发、状态监控、参数设置、急停释放力矩与恢复
- `sim_bridge.py`：最简 WebSocket 广播（`ws://localhost:8765`），向网页仿真推送关节角度
- `arm_config.json`：关节参数（行程、PWM 限位、方向、速度、补偿、归零位），需与主程序同目录

无第三方依赖，串口经 Win32 API 实现，Windows + Python 3 即可运行。

## 模型

`model/3D打印.stl` — 结构件打印模型，二进制 STL，可直接导入切片软件打印。

## 后续补充

- 仿真
