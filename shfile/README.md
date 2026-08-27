# FAST-LIO2 一键部署脚本

本目录同时保留 Livox 与 RoboSense Airy 两套部署脚本。Airy 主线已在 NVIDIA
Jetson Orin NX（Ubuntu 20.04 + ROS Noetic，`arm64`）实测，也支持 `amd64`、
`i386` 和 `armhf` 目标机原生编译。

Airy 从接线、网络配置到编译、运行、录包、调参和故障排查的完整说明见
项目根目录的 `AIRY_FASTLIO_MANUAL.md`。

## RoboSense Airy（当前方案）

源码及配置位于：

```text
environment/rslidar_sdk/              # 官方 SDK/ROS1 驱动，固定 v1.5.19
environment/rslidar_sdk/src/rs_driver # 官方驱动子模块
environment/rslidar_sdk/config/config_airy.yaml
config/airy.yaml
launch/mapping_airy.launch
```

在 FAST_LIO 根目录执行：

```bash
bash shfile/install_fastlio2_Airy.sh
```

默认模式会获取 sudo 权限并安装系统依赖，然后把官方驱动编译为 `XYZIRT` 点型、
开启 Airy IMU 解析及 PCAP 支持，最后编译不依赖 Livox 消息包的 FAST-LIO2。
每种 CPU 架构使用独立构建目录，不会混用 x86 与 ARM 产物。运行前要求 Ubuntu
20.04 的 APT/ROS Noetic 软件源可用，脚本不会自行添加软件源或下载缺失源码。

Orin 或内存较紧的平台可限制并行度：

```bash
bash shfile/install_fastlio2_Airy.sh --jobs 2
```

安装完成后启动驱动与建图：

```bash
source shfile/setup_fastlio2_Airy.bash
roslaunch fast_lio mapping_airy.launch
```

也可以先只启动驱动，再用自检脚本确认点云和 IMU：

```bash
roslaunch rslidar_sdk start_airy.launch
# 在另一个终端执行
source shfile/setup_fastlio2_Airy.bash
bash shfile/check_airy_topics.sh
```

无人机板端默认不启动 RViz。在有桌面环境的电脑上可使用：

```bash
roslaunch fast_lio mapping_airy.launch rviz:=true
```

## 飞机端综合一键启动（Airy + FAST-LIO + MAVROS/PX4）

[`start_airy_px4.sh`](start_airy_px4.sh) 面向当前 Orin + 飞控装机方案，一次启动：

```text
Airy 点云/IMU -> FAST-LIO /Odometry
                           |
                           v
                 坐标变换与失效门控
                           |
                           v
              /liu/mavros/vision_pose/pose
                           |
                           v
                  MAVROS -> PX4 EKF2
```

PX4 固件运行在飞控板上，Orin 不需要也不应启动 PX4 SITL。综合脚本启动的是
`/dev/ttyTHS0:921600` 上的 MAVROS 链路、雷达 SLAM、数据桥和只读监控器。

### 首次使用：只诊断，不向飞控发送位姿

一键安装脚本现在也会安装 MAVROS、MAVROS extras、诊断消息、NumPy 和
GeographicLib 数据。安装/增量编译后先复制本机配置：

```bash
cd /home/liu/study/FAST_LIO
cp shfile/airy_px4.env.example shfile/airy_px4.env
bash shfile/start_airy_px4.sh --diagnostic-only --test-seconds 30
```

`shfile/airy_px4.env` 是单机标定文件，已被 Git 忽略。第一次不复制也能运行诊断，
但安全默认值会保持 `PUBLISH_VISION=0`、`MOUNT_CONFIRMED=0`，桥接器不会向
MAVROS 发布未经标定的位姿。

当前板端已确认的硬件与改参前诊断基线为：

| 项目 | 当前实测 |
| --- | --- |
| 飞控串口 | `/dev/ttyTHS0:921600`，用户 `liu` 属于 `dialout` |
| 飞控/MAVROS | PX4 v1.13.3，MAVROS 已连接，时间同步 RTT 约 1.3 ms |
| 启动测试状态 | `connected=true`、`armed=false`；测试时为 `ALTCTL`，脚本不切模式 |
| PX4 融合参数（旧诊断） | 当时 `EKF2_AID_MASK=2`，没有开启视觉位置 |
| 活动估计器 | `SYS_MC_EST_GROUP=2`，EKF2 |
| 外部视觉延迟（旧诊断） | `EKF2_EV_DELAY=175 ms`，尚需 PX4 日志实测调节 |
| FAST-LIO 位姿 | 约 10 Hz；满足算法现状，但低于飞行验收使用的 30 Hz 门槛 |
| 安装方向诊断 | 单位四元数占位值下双 IMU 重力夹角 `178.73°`，该占位值明确不可用 |

以上表格记录的是 2026-08-27 改参前的实测，不能当作飞控当前参数快照。用户随后
已设置 `EKF2_AID_MASK=24` 和 `EKF2_HGT_MODE=3`，但尚未由本项目在飞控重启后复跑
只读诊断。PX4 v1.13 中 `24=8+16`，即 `EV_POS + EV_YAW`；`HGT_MODE=3` 表示视觉
高度。参数定义可查阅
[PX4 v1.13 参数表](https://docs.px4.io/v1.13/en/advanced_config/parameter_reference)。

新值写入不等于已经可飞 Position：默认 `ALLOW_VISION_YAW_FUSION=0`，而 `24` 包含
视觉 yaw，因此启动器会将整路视觉发布保持关闭并继续显示非就绪状态；它不会只丢弃
yaw 后继续发布位置。必须在完成安装外参、方向/航向和拆桨测试后才可考虑放开该门控。

修正后的综合脚本已在本机完成一次 30 秒全链路测试：只读同步 PX4 参数 907 项，
FCU 全程 `armed=false`，FAST-LIO 合法位姿约 10.01 Hz、时间年龄约 0.008 秒，
MAVROS timesync RTT 约 1.30 ms、偏移标准差约 0.014 ms；安全门控使视觉输出保持
0 Hz，测试断言通过后所有自启进程均被清理。Airy 主机时间自检还确认单帧逐点时间
跨度约 0.099793 秒，点云/逐点时间/IMU 都在 Orin 当前时间附近。

随后进行的 10 秒复验仍保持 `armed=false`、模式 `ALTCTL`，FAST-LIO 输入约
10.02 Hz，timesync 正常；新增的被动双 IMU 检查测得单位四元数占位值下重力夹角
为 `178.73°`。这证明当前传感器方向不能使用模板里的单位旋转。重力检查只能约束
横滚/俯仰，不能判断航向，也不能仅凭该数值判断应绕 X 轴还是 Y 轴旋转 180°；必须
先确认 Airy 原始 IMU 与 MAVROS IMU 的比力符号/坐标约定，再依据实际装机方向标定
完整四元数并做拆桨方向测试。

本次测试也发现并修正了 Airy 默认设备时钟问题：该设备当前给出开机相对时间而非
UTC，所以 `environment/rslidar_sdk/config/config_airy.yaml` 必须保持
`use_lidar_clock: false`。若未经 PTP/GPS 同步改成 `true`，桥会因位姿时间与 ROS/PX4
相差约 1787838951 秒而拒绝数据。

### 必须填写的安装外参

`config/airy.yaml` 的外参是“Airy 激光雷达到 Airy 内部 IMU”，并不是传感器到
飞机的安装外参。飞机配置还必须给出：

```bash
# 四元数 [x,y,z,w]：Airy 内部 IMU I -> 飞机 ROS base_link/FLU B
SENSOR_TO_BODY_Q_X=...
SENSOR_TO_BODY_Q_Y=...
SENSOR_TO_BODY_Q_Z=...
SENSOR_TO_BODY_Q_W=...

# Airy IMU 原点相对飞机控制/重心原点的位置，单位 m，表达在 base_link/FLU
SENSOR_POSITION_IN_BODY_X=...
SENSOR_POSITION_IN_BODY_Y=...
SENSOR_POSITION_IN_BODY_Z=...
```

FAST-LIO 使用的是 Airy 内置 IMU，因而 `/Odometry` 的 `body` 是 Airy IMU 机体系，
不是飞机 `base_link`。只把 `frame_id`/`child_frame_id` 改成 `base_link` 不会改变
任何向量或姿态数值，属于错误做法。桥接前必须真实完成：

```text
Airy IMU I --安装旋转/杆臂--> base_link/FLU B
camera_init --静止世界对齐--> ROS ENU --MAVROS--> PX4 NED/FRD
```

除旋转和杆臂外，还必须确保 Airy 点云/IMU/FAST-LIO 位姿时钟一致，点云约 10 Hz、
IMU 约 200 Hz 且无断流，MAVROS timesync 正常，并用 PX4 日志校准外部视觉频率、
延迟和创新量。修改消息标签不能替代这些数值变换与时序验证。

完成实测/标定后才能设置 `MOUNT_CONFIRMED=1`。桥会在飞控未解锁且飞机静止时，
利用飞控姿态把 FAST-LIO 的任意 `camera_init` 世界系对齐到 ROS ENU，并检查两颗
IMU 的重力方向是否与安装旋转一致。它还会把 Airy IMU 的轨迹补偿到飞机原点。
因此 PX4 的 `EKF2_EV_POS_X/Y/Z` 必须保持 0，不能再做第二次杆臂补偿。

当前模板中的单位四元数已被实测判定错误（重力夹角 `178.73°`），只能作为使配置
格式完整的禁用占位值，绝不能把 `MOUNT_CONFIRMED` 改为 1 后继续使用。

### PX4 参数只读门控

启动器只调用参数读取接口，**不会**调用 `param/set`。在 QGroundControl 中由操作者
确认外部位置源方案后配置 PX4 v1.13：

- `EKF2_AID_MASK` 的 value `8`（bit 3）必须开启视觉位置；
- 改参前诊断值 `2` 是光流；用户现已设置但尚待复核的 `24=8+16`，同时请求视觉
  位置和视觉 yaw；
- 用户已设置但尚待复核的 `EKF2_HGT_MODE=3` 表示视觉高度，修改后需重启飞控并
  检查高度融合、复位和失位降级行为；
- FAST-LIO 航向未独立标定时，不得放开 value `16` 的视觉 yaw。默认
  `ALLOW_VISION_YAW_FUSION=0` 会让含 bit 4 的 `24` 阻断整路视觉发布；只有完成安装
  外参、方向/航向标定与拆桨测试后才可改为 `1`；
- 当前桥不发送速度，不得开启 value `256` 的视觉速度；
- 桥已经输出 ROS ENU 并由 MAVROS 转为 PX4 NED，不应再开启 value `64` 的外部
  视觉旋转；
- `EKF2_EV_DELAY` 应根据 PX4 日志和创新量调节，不能凭猜测自动填写；
- 参数写入值、EKF 实际融合状态、失位后的降级行为以及解锁前检查必须在
  QGroundControl/PX4 日志中单独确认。

修改需要重启的 PX4 参数后，重启飞控，再重新运行诊断。不要通过关闭 EKF 解锁
检查或断路器来掩盖定位错误。

### 请求发布与日常一键启动

完成安装外参、PX4 参数、拆桨方向检查后，在 `shfile/airy_px4.env` 中设置：

```bash
MOUNT_CONFIRMED=1
DIRECTION_TEST_CONFIRMED=1
PUBLISH_VISION=1
```

若只读复核仍为 `EKF2_AID_MASK=24`，只有在视觉航向独立标定和拆桨航向测试也通过
后才能再设置 `ALLOW_VISION_YAW_FUSION=1`。否则保持默认 `0`，启动器将继续阻断
视觉发布；不要为了消除告警直接绕过门控。

随后日常启动只有一条命令：

```bash
cd /home/liu/study/FAST_LIO
bash shfile/start_airy_px4.sh
```

停止整套伴随计算机链路：

```bash
bash shfile/stop_airy_px4.sh
```

停止脚本优先使用会话 PID、启动时间和进程组清单；只有清单不存在时，才按本项目
绝对路径和 `/${UAV_NAME}/mavros` 命名空间清理当前用户的本机残留。它不会向飞控
发送模式、解锁或参数命令，可以重复执行，没有相关进程时也正常返回。启动器本次
创建的 roscore 会写入清单并随会话停止；复用的已有 ROS master 始终保留。

命令行的 `--publish-vision` 也只能提出发布请求，不能绕过安装外参、PX4 融合位、
重复杆臂补偿、视觉 yaw/速度等门控。启动后先让飞机固定静止至少 5 秒；桥只能在
`connected=true`、`armed=false` 时锁定世界对齐，之后即使操作者用遥控器解锁，
正常位姿流也会继续。数据回跳、突跳、非有限数、时间戳过期或雷达断流会立即停发，
不会用最新时间戳重复上一帧。

常用命令：

```bash
# 打印配置，不启动硬件
bash shfile/start_airy_px4.sh --dry-run

# 30 秒未解锁台架测试，结束后自动收集最终状态并清理进程
bash shfile/start_airy_px4.sh --diagnostic-only --test-seconds 30

# 查看一个已经运行的会话，不启动或停止任何组件
bash shfile/start_airy_px4.sh --status-only

# 查看停止目标，不发送信号
bash shfile/stop_airy_px4.sh --dry-run

# 停止本项目飞机定位链路
bash shfile/stop_airy_px4.sh
```

综合脚本绝不执行以下操作：自动 `set_mode`、自动 `arming`、自动起飞、写 PX4
参数、发送 OFFBOARD setpoint 或绕过 Preflight 检查。Position 模式切换和解锁
始终由遥控器与操作者完成。

### 状态含义与飞行前验收

读取总体状态：

```bash
rostopic echo -n 1 /airy_px4/monitor/diagnostics
rostopic echo -n 1 /airy_px4/bridge/diagnostics
```

| 状态 | 含义 |
| --- | --- |
| `BLOCKED_BY_STARTUP_GATE` | 安全默认值或 PX4 参数阻止了视觉发布 |
| `WAITING_STATIONARY_ALIGNMENT` | 等待未解锁、静止和双 IMU 重力一致性检查 |
| `PUBLISHING_NATIVE_RATE` | 桥正按 FAST-LIO 原始时间戳/原始频率发布 |
| `BENCH_ONLY_POSE_RATE_TOO_LOW` | 位姿链路已通，但频率低于飞行验收门槛 |
| `PX4_NOT_ACCEPTING_VISION` | 桥已发数据，但 PX4 estimator/local pose 尚未证明接收融合 |
| `POSITION_DATA_READY` | 自动技术门控与人工方向确认均通过；仍不代表允许直接起飞 |
| `FAULT_LATCHED` | 发生断流、回跳或突跳，已停发；排障后必须重启桥/综合脚本 |

当前 FAST-LIO 受 10 Hz 激光扫描更新限制，而 PX4 外部视觉通常建议 30～50 Hz。
脚本把 30 Hz 作为飞行验收门槛，不会把旧位姿换成新时间戳重复发布。要消除这个
阻塞，需要增加真实的高频 IMU 状态传播与正确的回溯校正，而不是简单上采样。

拆桨并固定飞机后至少完成：静置漂移、前/左/上与转向符号、MAVROS local pose
一致性、雷达断流立即停发、PX4 失位降级、三次冷启动重复测试。方向和失效测试通过
后才可设置 `DIRECTION_TEST_CONFIRMED=1`。飞行前还要检查电池状态、桨叶区域、遥控
失控保护和现场安全条件。

雷达网络、端口、逐点时间戳与外参注意事项见
`environment/README_AIRY.md`。

## Livox（原方案，继续保留）

Livox 源码和脚本没有删除。需要恢复 Livox 雷达时使用：

```bash
bash shfile/install_fastlio2_Livox.sh
source shfile/setup_fastlio2_Livox.bash
```

两套构建使用不同目录，Airy 构建会显式关闭
`livox_ros_driver/CustomMsg` 依赖，Livox 构建默认仍启用该支持。

## 安装模式与常用选项

```bash
# 默认：安装依赖，支持在线 UDP 和驱动直接读取离线 PCAP
bash shfile/install_fastlio2_Airy.sh --jobs 2

# 仅使用在线 Airy，不安装/链接 libpcap
bash shfile/install_fastlio2_Airy.sh --online-only --jobs 2

# 依赖已经就绪，只检查依赖并增量编译，全程不调用 sudo/apt
bash shfile/install_fastlio2_Airy.sh --build-only --online-only --jobs 2

# 显式将一个已有 NetworkManager 有线连接配置为 Airy 专网
bash shfile/install_fastlio2_Airy.sh --jobs 2 \
  --configure-network '有线连接 1' \
  --host-cidr 192.168.1.102/24 \
  --lidar-ip 192.168.1.200

# APT 索引已更新时
bash shfile/install_fastlio2_Airy.sh --skip-apt-update

# 安装 RViz 等完整 ROS 桌面组件
bash shfile/install_fastlio2_Airy.sh --desktop-full
```

`--online-only` 只禁用 rslidar_sdk 直接解析 PCAP 文件，不影响在线点云/IMU，也
不影响 ROS bag。`--build-only` 为保证零 sudo，不能与 `--configure-network`
同时使用。网络配置是显式选择项：它设置静态地址、清空网关、启用
`ipv4.never-default` 并禁用 IPv6，不会在默认安装时自动修改网卡。

完整参数说明：

```bash
bash shfile/install_fastlio2_Airy.sh --help
```

脚本默认要求 Ubuntu 20.04。`--allow-unsupported-os` 只用于已经自行配置好
ROS Noetic 的非标准系统，不保证其 APT 软件源存在所需二进制包。
