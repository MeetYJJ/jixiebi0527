# 果园自动采摘 ROS2 Foxy 第一版程序

这是 Ubuntu 20.04 + ROS2 Foxy + Python3 下的第一版可运行骨架，面向“视觉给苹果三维坐标，睿尔曼 6 自由度机械臂 + EG2-4C2 夹爪执行采摘”的流程。

## 程序结构

- `rm_json_bridge.py`：唯一 TCP Socket 驱动节点，负责 JSON 编码、`\r\n` 帧尾、回包等待、单位转换。
- `pick_task_manager.py`：采摘状态机，流程为开爪、预抓取、直线靠近、夹紧、第六关节扭转、回撤、回 home、松爪。
- `fake_apple_target.py`：相机未接入时发布一个安全姿态附近的测试苹果坐标。
- `config/picker_params.yaml`：机械臂 IP、home 点、TCP 姿态、夹爪参数、工作空间边界。

## 第一版实现功能

第一版目标是先打通“视觉目标 -> 机械臂运动 -> 夹爪动作 -> 扭果 -> 回安全点”的最小闭环，避免过早引入复杂避障和识别策略。

已实现：

- 睿尔曼机械臂 JSON TCP 通信，所有指令自动追加 `\r\n`。
- ROS2 Foxy 服务接口封装：查询状态、关节运动、位姿运动、夹爪张开、夹爪夹紧、夹爪位置、工具端电压。
- 相机接口预留：订阅 `/apple/target_pose` 和 `/apple/target_point`。
- 相机未调好时的假目标节点：发布安全姿态附近的苹果坐标用于联调。
- 固定 TCP 姿态采摘：第一版先使用参数 `tcp_rpy_rad`，不做手眼误差补偿。
- 单苹果采摘状态机：开爪、到预抓取点、直线靠近、夹紧、第六关节扭转、回撤、回 home、松爪。
- 工作空间范围检查，目标超出 `workspace_min/max` 会拒绝执行。
- 失败恢复：执行失败时默认回安全 home。

第一版暂不实现：

- 复杂障碍物避障；
- 苹果大小估计和自适应抓取姿态；
- 手眼标定误差补偿；
- 多苹果排序和连续采摘；
- MoveIt 或点云路径规划。

第一版的核心边界是：相机只给目标点，状态机只管流程和失败恢复，驱动节点只管可靠下发 JSON 指令。

## 已写入的安全姿态

来自当前机械臂状态：

```json
{"arm_state":{"err":[0],"joint":[107149,-84991,112484,-130772,64784,85805],"pose":[150269,-219514,597973,782,409,1043]},"state":"current_arm_state"}
```

程序参数中使用：

- `home_joint_deg`: `[107.149, -84.991, 112.484, -130.772, 64.784, 85.805]`
- `safe_tcp_pose_m_rpy_rad`: `[0.150269, -0.219514, 0.597973, 0.782, 0.409, 1.043]`
- 假相机目标默认值：`[0.200269, -0.219514, 0.597973]`

假目标配合 `approach_distance: 0.05`，使预抓取点接近安全 TCP 位置，方便低风险联调。

## 编译

把整个 `orchard_picker` 文件夹复制到 ROS2 工作区：

```bash
mkdir -p ~/ros2_ws/src
cp -r orchard_picker ~/ros2_ws/src/
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select orchard_picker
source install/setup.bash
```

如果缺少依赖，通常补这些：

```bash
sudo apt install ros-foxy-tf2-ros ros-foxy-tf2-geometry-msgs
```

## 上机前必须修改

编辑：

```bash
nano ~/ros2_ws/src/orchard_picker/config/picker_params.yaml
```

至少确认：

- `rm_json_bridge.host`：机械臂 IP，常见为 `192.168.1.18`。
- `rm_json_bridge.port`：JSON TCP 端口，常见为 `8080`。
- `pick_task_manager.home_joint_deg`：已按你给的安全姿态写入，上机前仍建议用示教器确认。
- `pick_task_manager.tcp_rpy_rad`：当前用安全姿态中的 TCP RPY。
- `workspace_min/workspace_max`：允许采摘的安全空间范围。
- `gripper_pick_force`：夹持力，先从小值测试。

如果夹爪需要工具端供电，把：

```yaml
set_tool_voltage_on_start: true
tool_voltage_type: 3
```

其中 `3` 表示 24V。

## 因时 EG2-4C2 夹爪设置

当前第一版默认使用因时手册的 Modbus RTU 寄存器方式控制夹爪：

```yaml
gripper_driver: "inspire_modbus"
gripper_modbus_device: 1
gripper_modbus_type: 1
gripper_modbus_baudrate: 115200
gripper_modbus_control_address: 10
gripper_modbus_status_address: 65
```

驱动启动时会先发送睿尔曼 JSON 指令，把末端 RS485 设置为 Modbus RTU 主站：

```json
{"command":"set_tool_rs485_mode","mode":0,"baudrate":115200}
```

夹爪动作使用手册里的连续保持寄存器：

```text
地址 10: 目标开口，0-1000，0 为闭合，1000 为张开
地址 11: 速度，10-1000
地址 12: 力，100-1000
地址 65: 夹爪状态
```

服务调用时会通过睿尔曼 JSON 发送：

```json
{"command":"write_modbus_rtu_registers","address":10,"data":[开口,速度,力],"device":1,"type":1}
```

常用测试：

```bash
# 张开：开口 1000，速度 300，默认力 200
ros2 service call /rm_json_bridge/gripper_release orchard_picker/srv/GripperRelease "{speed: 300, wait: true}"

# 夹紧：开口 0，速度 200，力 150
ros2 service call /rm_json_bridge/gripper_pick orchard_picker/srv/GripperPick "{speed: 200, force: 150, wait: true}"

# 指定开口位置，例如半开 500
ros2 service call /rm_json_bridge/gripper_position orchard_picker/srv/GripperPosition "{position: 500, wait: true}"
```

## T930 网口检查

如果是 T930 直接网线连接机械臂，先用项目内置脚本配置网口并确认机械臂 JSON TCP 端口可用：

```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run orchard_picker t930_arm_net_check.sh
```

脚本默认参数和你之前成功连接时一致：

```text
IF=enP5p1s0f0
HOST_IP=192.168.1.100
ARM_IP=192.168.1.18
PORT=8080
```

如果网口名不同，可以临时覆盖：

```bash
IF=你的网口名 ros2 run orchard_picker t930_arm_net_check.sh
```

第 6 步能返回 `current_arm_state`，说明 ROS 之外的网线、IP、路由和 JSON TCP 都是通的。

## 启动和调试

建议按下面顺序调试，不要一开始就跑完整采摘。

如果刚更新了代码，先确认脚本有执行权限并干净重编译：

```bash
cd ~/ros2_ws
chmod +x src/orchard_picker/scripts/*.py src/orchard_picker/scripts/*.sh
rm -rf build/orchard_picker install/orchard_picker
colcon build --packages-select orchard_picker
source install/setup.bash
```

如果 `ros2 run orchard_picker t930_arm_net_check.sh` 提示 `No executable found`，说明安装空间里还没有这个脚本。重新执行上面的 `chmod +x` 和干净重编译即可；也可以临时直接运行：

```bash
bash src/orchard_picker/scripts/t930_arm_net_check.sh
```

### 1. 网络直连验证

先确认 T930 到机械臂的网线、IP、端口都通：

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run orchard_picker t930_arm_net_check.sh
```

成功标志：

- `ping 192.168.1.18` 有回复；
- `nc` 显示 `8080` 端口连接成功；
- 第 6 步返回 `current_arm_state`。

### 2. 启动机械臂 JSON 驱动

终端 A：

```bash
ros2 launch orchard_picker driver_only.launch.py
```

成功标志：

```text
Connected to RealMan controller at 192.168.1.18:8080
```

可选观察 TCP 连接状态和原始收发：

```bash
ros2 topic echo /rm_json_bridge/connected
ros2 topic echo /rm_json_bridge/raw_tx
ros2 topic echo /rm_json_bridge/raw_rx
```

### 3. 查询机械臂状态

终端 B：

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run orchard_picker raw_json_cli.py '{"command":"get_current_arm_state"}'
```

成功标志：

```text
success: True
message: ok
response_json: {"ack":{"arm_state":...,"state":"current_arm_state"}}
```

注意这条命令要在另一个终端运行，并且 `driver_only.launch.py` 终端必须保持运行。如果提示一直等待 `/rm_json_bridge/raw_json`，说明 `rm_json_bridge` 没有启动、启动后退出了，或者当前终端没有 `source ~/ros2_ws/install/setup.bash`。

### 4. 单独测试夹爪

```bash
ros2 service call /rm_json_bridge/gripper_release orchard_picker/srv/GripperRelease "{speed: 300, wait: true}"
ros2 service call /rm_json_bridge/gripper_pick orchard_picker/srv/GripperPick "{speed: 200, force: 150, wait: true}"
```

### 5. 回安全 home

终端 A 停掉 `driver_only.launch.py`，然后启动完整程序：

```bash
ros2 launch orchard_picker orchard_picker.launch.py
```

另开终端 B，调用回安全 home。这一步会让机械臂运动：

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 service call /pick_task_manager/home std_srvs/srv/Trigger "{}"
```

### 6. 相机未接入时跑假目标

打开终端A观察状态：

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch orchard_picker fake_target_demo.launch.py
```

另开终端B观察状态：

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /pick_task_manager/state
```

另开终端C再执行一次完整采摘流程：

```bash
ros2 service call /pick_task_manager/run_once std_srvs/srv/Trigger "{}"
```

状态顺序应接近：

```text
IDLE
OPEN_GRIPPER
MOVE_PRE_GRASP
MOVE_GRASP
CLOSE_GRIPPER
TWIST_J6
RETREAT
HOME
RELEASE_GRIPPER
DONE
```

## 接入真实视觉

视觉节点发布下面任意一个 topic：

```text
/apple/target_pose   geometry_msgs/msg/PoseStamped
/apple/target_point  geometry_msgs/msg/PointStamped
```

如果相机坐标不是 `base_link`，需要提供 TF，例如 `camera_link -> base_link`。程序会自动尝试把苹果坐标转换到 `base_link`。

## 协议注意事项

- 睿尔曼 JSON TCP 指令必须以 `\r\n` 结尾，本驱动已经统一处理。
- 调试 JSON 时不要同时连接 WEB 示教器或其他上位机，避免命令冲突。
- `movej` 关节角单位：协议中是 `0.001°`，程序接口中使用“度”。
- `movej_p/movel` 位姿单位：协议中位置是 `0.001 mm`、姿态是 `0.001 rad`，程序接口中使用“米 + 四元数”。
- EG2-4C2 夹爪命令使用 `set_gripper_release`、`set_gripper_pick`、`set_gripper_position`。
