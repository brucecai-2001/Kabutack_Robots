# Unitree ROS2 Scripts 使用指南

本包包含用于控制Unitree机器人的ROS2 Python节点。

## 包结构

```
go2_scripts/
├── package.xml              # ROS2包配置文件
├── setup.py                 # Python包安装配置
├── setup.cfg                # 构建配置
├── resource/                # 资源文件夹
│   └── go2_scripts
└── src/                     # 源码目录
    └── go2_scripts/         # Python包源码
        ├── __init__.py
        ├── api/             # API模块
        │   ├── __init__.py
        │   └── sport_api.py # 运动API封装
        ├── examples/        # 示例代码
        │   ├── __init__.py
        │   └── simple_move_forward.py # 简单前进示例
        └── nodes/           # ROS2节点
            ├── __init__.py
            └── go2_node.py  # 主控制节点
```

## 环境要求

- ROS2 (Humble/Foxy)
- Python 3.8+
- 已安装的Unitree相关包：
  - `unitree_go`
  - `unitree_api`
  - `sensor_msgs`
  - `cv_bridge`



## 构建包

1. 进入工作空间根目录：
```bash
cd /path/to/your/ros2_workspace
```

```bash
conda activate unitree_foxy
pip install -U colcon-common-extensions
```

2. 构建包：
```bash
python -m colcon build --packages-select go2_scripts
```

3. 源化环境：
```bash
source install/setup.bash
```

## 运行节点

构建包后，可以直接使用ROS2命令运行：

```bash
# 确保已经源化环境
source install/setup.bash

# 运行simple_move_forward节点
ros2 run go2_scripts simple_move_forward

# 运行主控制节点
ros2 run go2_scripts go2_node
```


### 节点功能说明

`simple_move_forward` 节点的功能：

- **目标**：控制机器人向前移动1米
- **移动速度**：0.3 m/s
- **移动时间**：约3.3秒（1米 ÷ 0.3 m/s）
- **控制频率**：10Hz（每100ms发送一次命令）
- **自动停止**：完成移动后自动发送停止命令并关闭节点

### 运行日志示例

正常运行时，你会看到类似以下的日志输出：

```
[INFO] [1234567890.123456789] [simple_move_forward]: 简单前进控制节点已启动
[INFO] [1234567890.223456789] [simple_move_forward]: 开始前进，速度: 0.3 m/s，持续时间: 3.3 秒
[INFO] [1234567893.523456789] [simple_move_forward]: 停止移动
[INFO] [1234567893.523456789] [simple_move_forward]: 前进1米动作完成！
[INFO] [1234567895.523456789] [simple_move_forward]: 任务完成，关闭节点
```

## 注意事项

1. **安全第一**：运行前确保机器人周围有足够的空间，避免碰撞
2. **机器人状态**：确保机器人处于站立状态且可以接收运动命令
3. **话题连接**：节点会发布到 `/api/sport/request` 话题，确保相关的控制系统正在运行
4. **紧急停止**：如需紧急停止，可以使用 `Ctrl+C` 中断程序

## 故障排除

### 常见问题

1. **导入错误**：
   ```
   ModuleNotFoundError: No module named 'unitree_ros2.scripts.sport_api'
   ```
   **解决方案**：确保已正确构建包并源化环境

2. **话题连接失败**：
   ```
   [WARN] [simple_move_forward]: 发布者尚未连接
   ```
   **解决方案**：检查机器人控制系统是否正在运行

3. **权限错误**：
   ```
   Permission denied
   ```
   **解决方案**：确保Python脚本有执行权限：`chmod +x simple_move_forward.py`

## 其他节点

- **go2_node.py**：主控制节点，提供完整的机器人控制功能和ZMQ通信接口
- **sport_api.py**：运动API封装，提供各种运动控制方法

## 开发和自定义

如需修改移动参数，可以编辑 `simple_move_forward.py` 中的以下变量：

```python
move_speed = 0.3      # 移动速度 (m/s)
move_duration = 1.0 / move_speed  # 移动时间根据目标距离计算
```

要改变移动距离，修改计算公式：
```python
target_distance = 2.0  # 目标距离 (米)
move_duration = target_distance / move_speed
```