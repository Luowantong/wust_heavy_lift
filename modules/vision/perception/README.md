# Perception (vendored)

从 HKUST ENTERPRIZE 的开源仓库借用感知部分：

- 仓库：<https://github.com/hkustenterprize/RM2026-Engineer-Assembly-Algorithm>
- 提交：`7fb7609`
- 源路径：`runtime/core/arm_exchange_core/`、`runtime/host/arm_exchange_host/`
- 协议：MIT，见本目录 `LICENSE`（版权归原作者）

这一步只把**感知代码**原样拷进来，不做任何适配、不改话题/坐标系、不加资产和权重。

## 目录内容

```
perception/
├── LICENSE
├── arm_exchange_core/
│   ├── __init__.py                    # load_config()
│   ├── system_config.example.yaml     # perception + keypoint_schemas 等配置模板
│   ├── transform.py                   # 四元数/变换工具
│   └── perception/
│       ├── __init__.py
│       ├── detector.py                # OpenVINO YOLO + LiteHRNet
│       └── pose.py                    # RANSAC + SQPnP + LM PnP
└── arm_exchange_host/
    ├── __init__.py
    ├── perception_node.py             # ROS 节点模板
    └── ros_utils.py                   # Image 解码 / PoseStamped 转换
```

## 尚未包含（后续步骤）

- 权重（HuggingFace 的 OpenVINO 导出）和 `weights/` 目录
- MuJoCo 的兑换站资产（`runtime/sim/arm_exchange_sim/model/model/exchange_station/`）
- `arm_exchange_interfaces` 消息包、`arm_exchange_core/trajectory.py`
  （`perception_node.py` / `ros_utils.py` 目前引用了它们，直接运行会 import 失败）
- wust 侧的话题/坐标系映射、相机内参与手眼标定、`station → workcell` 偏移

## 依赖

`openvino`、`opencv`、`numpy`、`rclpy`、`tf2_ros`。不需要 torch / MMPose / Ultralytics。
