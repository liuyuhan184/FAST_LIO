# `shfile` 脚本接口与运行参考

本目录提供 RoboSense Airy + FAST-LIO2 + MAVROS/PX4 的安装、环境、自检、标定、桥接、
监控和综合启停工具。本文件是脚本接口参考；从零部署请阅读
[快速部署与操作手册](../README.md)，算法、坐标系、参数和二次开发说明见
[完整项目与二次开发手册](../AIRY_FASTLIO_MANUAL.md)。

> 涉及飞机的首次测试必须拆下全部桨叶、固定机体并保持飞控未解锁。脚本不会自动切换
> 模式、解锁、起飞、写 PX4 参数或发送 OFFBOARD 控制量。

## 1. 文件索引

| 文件 | 用途 | 是否直接运行 |
| --- | --- | --- |
| `install_fastlio2_Airy.sh` | 安装依赖、可选配置雷达网口、构建 Airy 驱动和 FAST-LIO | `bash` |
| `setup_fastlio2_Airy.bash` | 加载 ROS Noetic、驱动和 FAST-LIO 分架构 overlay | 必须 `source` |
| `check_airy_topics.sh` | 检查一条 Airy 点云/IMU、XYZIRT 字段和时间轴 | `bash` |
| `99-fastlio-airy.conf` | Airy UDP 接收缓冲 sysctl 模板 | 由安装脚本安装 |
| `airy_px4.env.example` | 飞机连接、外参、安全门和质量阈值模板 | 复制后编辑 |
| `start_airy_px4.sh` | 综合启动 roscore、MAVROS、FAST-LIO、桥和监控 | `bash` |
| `stop_airy_px4.sh` | 按会话元数据精确停止综合链路 | `bash` |
| `fastlio_to_mavros.py` | FAST-LIO 到飞机 ENU/FLU 位姿桥与失效保护 | 由启动器调用 |
| `monitor_airy_px4.py` | 只读链路/PX4 状态监控 | 由启动器调用 |
| `calibrate_airy_body.py` | 双 IMU 三轴动态估计 Airy IMU 到飞机 FLU 的旋转 | 手工运行 |
| `install_fastlio2_Livox.sh` | 保留的 Livox 安装构建链路 | `bash` |
| `setup_fastlio2_Livox.bash` | 保留的 Livox 环境加载 | 必须 `source` |

运行时生成的 `airy_px4.env`、构建目录和 `runtime/` 已被 `.gitignore` 排除。

## 2. 最常用命令

```bash
# 新机安装/编译
bash shfile/install_fastlio2_Airy.sh --jobs 2

# 手工运行 ROS 命令前加载环境
source shfile/setup_fastlio2_Airy.bash

# 驱动已启动时检查 Airy 输入
bash shfile/check_airy_topics.sh

# 第一次创建本机飞机配置
cp shfile/airy_px4.env.example shfile/airy_px4.env

# 拆桨、未解锁，只诊断 30 秒
bash shfile/start_airy_px4.sh --diagnostic-only --test-seconds 30

# 拆桨、未解锁，限时请求发布 90 秒
bash shfile/start_airy_px4.sh --publish-vision --test-seconds 90

# 日常持续启动；仍受私有配置和全部硬门约束
bash shfile/start_airy_px4.sh

# 查看运行状态
bash shfile/start_airy_px4.sh --status-only

# 预览并停止
bash shfile/stop_airy_px4.sh --dry-run
bash shfile/stop_airy_px4.sh
```

综合启动器会自行加载 Airy 环境；在另一个终端执行 `rostopic`、`roslaunch`、`rosbag`
或标定器前仍应 `source shfile/setup_fastlio2_Airy.bash`。

## 3. Airy 安装脚本

### 3.1 接口

```text
bash shfile/install_fastlio2_Airy.sh [选项]
```

| 选项 | 作用 |
| --- | --- |
| `--jobs N` | 指定正整数并行编译数 |
| `--build-only` | 不调用 sudo/apt，只检查依赖并构建 |
| `--online-only` | 关闭 PCAP 解析，不要求 `libpcap-dev` |
| `--skip-apt-update` | 安装依赖前跳过 `apt-get update` |
| `--desktop-full` | 安装 `ros-noetic-desktop-full`，包含 RViz |
| `--allow-unsupported-os` | 在非 20.04 的其他 Ubuntu 版本尝试，不支持其他发行版 |
| `--configure-network NAME` | 修改明确指定的 NetworkManager 有线连接 |
| `--host-cidr IPv4/PREFIX` | 板端雷达专网地址，默认 `192.168.1.102/24` |
| `--lidar-ip IPv4` | 连通检查目标，默认 `192.168.1.200` |
| `-h`、`--help` | 显示脚本内权威帮助 |

对应环境变量包括 `BUILD_JOBS`、`SKIP_APT_UPDATE`、`INSTALL_DESKTOP_FULL`、
`ALLOW_UNSUPPORTED_UBUNTU`、`BUILD_ONLY`、`ONLINE_ONLY`、`AIRY_HOST_CIDR` 和
`AIRY_LIDAR_IP`。网卡连接名只能通过 `--configure-network` 明确传入，避免误改接口。

### 3.2 行为和副作用

默认完整运行会：

1. 检查 Ubuntu、ROS 发行版、CPU 架构和仓库源码；
2. 通过 APT 安装 ROS/PCL/Eigen/MAVROS/NumPy/GeographicLib 等依赖；
3. 安装 MAVROS GeographicLib 数据；
4. 可选修改指定 NetworkManager 连接；
5. 把 `99-fastlio-airy.conf` 安装到 `/etc/sysctl.d/` 并应用；
6. 以 Release、`XYZIRT`、`ENABLE_IMU_DATA_PARSE=ON` 构建驱动；
7. 以 Release、`ENABLE_LIVOX_SUPPORT=OFF` 构建 Airy FAST-LIO；
8. 检查动态库、CMake cache、ROS launch、Shell/Python 语法和 Python 自测。

脚本重复运行会复用源码和做增量构建。它不会克隆缺失的项目源码，也不会添加 ROS APT
软件源。

`--build-only` 不使用 sudo/apt，也不能与 `--configure-network` 同用；但它仍会更新
构建文件。`--online-only` 与默认 PCAP 构建使用不同宏，切换后脚本会重新配置。

### 3.3 输出路径

构建标签为 `noetic-<dpkg架构>`：

```text
environment/rslidar_ws/build/noetic-<arch>/
environment/rslidar_ws/devel/noetic-<arch>/
build/noetic-<arch>-airy/
```

不要跨 amd64/ARM64 复制构建产物。Orin 编译被系统杀死时使用 `--jobs 1`。

## 4. Airy 环境脚本

正确用法：

```bash
source shfile/setup_fastlio2_Airy.bash
```

直接执行会报错。脚本拒绝与已经加载的其他 ROS 发行版混用，自动选择当前架构 overlay，
并导出：

- `FAST_LIO_ROOT`；
- `RSLIDAR_SDK_ROOT`；
- `RSLIDAR_WS`。

默认行为是清除陈旧 `ROS_IP/ROS_HOSTNAME`，并使用本机 master：

```text
ROS_MASTER_URI=http://127.0.0.1:11311
```

有意使用远端 ROS master 时，必须在加载前设置：

下面的地址只是示例，必须换成真实管理网地址：

```bash
export AIRY_KEEP_ROS_NETWORK=1
export ROS_MASTER_URI=http://192.168.31.10:11311
export ROS_IP=192.168.31.20
source shfile/setup_fastlio2_Airy.bash
```

这只适用于手工 `source` 后运行的 ROS 命令。综合 `start_airy_px4.sh` 会清除显式
`ROS_IP/ROS_HOSTNAME`，只保留 `ROS_MASTER_URI`；其权威支持拓扑是本机 master。若要
让综合栈使用远端 master，必须先改造并验证所有节点的回连地址。

## 5. Airy 话题自检

驱动已经启动后运行：

```bash
source shfile/setup_fastlio2_Airy.bash
bash shfile/check_airy_topics.sh
```

脚本检查：

- ROS master 可达；
- `/rslidar_points` 和 `/rslidar_imu_data` 存在；
- 一条点云非空，字段包含 `x/y/z/intensity/ring/timestamp`；
- 一条 IMU 数值有限；
- 点云 header、逐点 timestamp、IMU 和 ROS 当前时间没有明显跨纪元。

它只取少量/单条消息，不证明 10 分钟持续频率、网络无丢包或 FAST-LIO 精度。通过后还
要使用 `rostopic hz`、`ip -s link` 和 UDP 统计做长测。

## 6. 飞机本机配置

### 6.1 创建和安全属性

```bash
cp shfile/airy_px4.env.example shfile/airy_px4.env
```

`airy_px4.env` 被 Git 忽略，应按“本机 + 本架飞机”独立维护。迁移到新 Orin、换雷达
安装位或换飞机时，必须安全迁移并复核，或重新标定。

启动器用 Bash `source` 读取该文件，因此它是可信 Shell 配置，不是允许任意外来内容的
普通数据文件。推荐只写简单 `KEY=value`；不要从不可信来源复制命令替换、函数或其他
Shell 代码。

可以使用其他配置文件：

```bash
bash shfile/start_airy_px4.sh --config /absolute/path/aircraft.env
bash shfile/stop_airy_px4.sh  --config /absolute/path/aircraft.env
```

### 6.2 连接变量

| 变量 | 模板值/含义 |
| --- | --- |
| `UAV_NAME` | `liu`；形成 `/<UAV_NAME>/mavros`，可改为合法 ROS 名 |
| `FCU_URL` | `/dev/ttyTHS0:921600`；按实际设备修改 |
| `GCS_URL` | 默认空；需要时填明确 QGC UDP 地址 |
| `MAV_SYS_ID`、`MAV_COMP_ID` | MAVLink system/component ID |
| `EXPECTED_PX4_MAJOR/MINOR` | 当前默认 `1/13`，用于固件硬门 |

模板中的用户名式 namespace 只是默认值，不是协议要求。改名后标定器的默认 MAVROS
话题也要用参数覆盖。

### 6.3 组件开关

| 变量 | `1` 时 | `0` 时的前提 |
| --- | --- | --- |
| `START_ROSCORE` | 创建或复用 master | 必须已有可达 master |
| `START_MAVROS` | 启动本会话 MAVROS | 必须已有 `/<UAV>/mavros` 且 FCU connected |
| `START_FASTLIO` | 启动 Airy + FAST-LIO | 仍须在启动器规定的等待窗口内看到 `/Odometry` |
| `START_BRIDGE` | 启动位姿桥 | 为 0 时视觉发布被禁止 |
| `START_MONITOR` | 启动只读监控 | 为 0 时视觉发布被禁止 |

这些开关用于复用已有组件，不是多 UAV 隔离机制。整套脚本按单机、单 UAV、单定位栈
设计。

### 6.4 发布安全门

| 变量 | 含义 |
| --- | --- |
| `PUBLISH_VISION` | 请求发布；单独设 1 不足以真正发布 |
| `MOUNT_CONFIRMED` | 安装旋转和杆臂已填写并人工复核 |
| `DIRECTION_TEST_CONFIRMED` | 拆桨正方向/符号和失效测试已完成 |
| `ALLOW_APPROXIMATE_DIRECTION_PUBLISH` | 允许方向未确认时持续发布的高风险豁免；有界测试不需要它 |
| `ALLOW_VISION_YAW_FUSION` | PX4 AID mask 包含 EV yaw 时的额外许可 |

推荐状态推进：

```text
全部 0
  -> diagnostic-only
  -> 双 IMU 标定、杆臂测量
  -> MOUNT_CONFIRMED=1
  -> 限时拆桨方向测试
  -> DIRECTION_TEST_CONFIRMED=1
  -> 正式请求 PUBLISH_VISION=1
```

`--test-seconds N` 本身允许在 `DIRECTION_TEST_CONFIRMED=0` 时做有界真实发布，其他门
仍必须全部通过，此时应保持 `ALLOW_APPROXIMATE_DIRECTION_PUBLISH=0`。把该变量设为
`1` 会允许 `TEST_SECONDS=0` 的方向未确认持续发布；它不会把方向状态伪装成已确认，
监控仍会告警，不应成为日常或飞行配置。

### 6.5 外参和阈值

```text
SENSOR_TO_BODY_Q_X/Y/Z/W
```

是把 Airy 内部 IMU 向量旋转到飞机 `base_link`/FLU 的四元数，顺序为 `xyzw`。

```text
SENSOR_POSITION_IN_BODY_X/Y/Z
```

是从项目明确选定的飞机 `base_link` 原点到 Airy IMU 原点的向量，表达在 FLU，单位
米。重心、飞控 IMU 原点和机架几何中心不能混称；应先统一本机定义。桥已补偿杆臂，
因此 PX4 `EKF2_EV_POS_X/Y/Z` 应为零。

质量参数的模板默认值和启动器允许范围如下；超出范围会在访问硬件前直接退出：

| 参数 | 默认值 | 允许范围 | 含义 |
| --- | ---: | ---: | --- |
| `MIN_FASTLIO_RATE_HZ` | `8.0` | `8～100` | 允许持续发布的最低 LIO 频率 |
| `FLIGHT_MIN_POSE_RATE_HZ` | `30.0` | `30～100` | 项目飞行验收门槛；当前约 10 Hz 未达到 |
| `MIN_LOCAL_POSE_RATE_HZ` | `2.0` | `1～20` | PX4 下行本地位姿持续性门槛 |
| `ALIGN_STABLE_SECONDS` | `5.0` | `5～60` | 静止世界对齐时长 |
| `MAX_SOURCE_AGE_S` | `0.5` | `0.05～0.5` | 源测量最大年龄 |
| `MAX_SOURCE_GAP_S` | `0.30` | `0.3～1.0` | 相邻 header 时间最大间隔 |
| `MAX_RECEIPT_STALL_S` | `0.50` | `0.5～2.0` | ROS 回调/进程接收最大停顿 |
| `MAX_POSITION_JUMP_M` | `1.0` | `0.05～5` | 对齐后单步位置跳变限制 |
| `MAX_ORIENTATION_JUMP_DEG` | `45.0` | `1～90` | 单步姿态跳变限制 |
| `MAX_GRAVITY_MISMATCH_DEG` | `10.0` | `1～15` | 双 IMU 重力最大夹角 |

源 header 缺帧和本机调度停顿是两个独立故障，不应通过一起放宽来掩盖网络丢包。

## 7. 综合启动器

### 7.1 命令行

```text
bash shfile/start_airy_px4.sh [选项]
```

| 选项 | 行为 |
| --- | --- |
| `--config FILE` | 使用指定私有配置 |
| `--diagnostic-only` | 强制禁发，只做整链路诊断 |
| `--publish-vision` | 请求发布，仍须通过全部硬门 |
| `--test-seconds N` | 运行 N 秒未解锁台架测试后自动收集状态并退出；N 至少 10 |
| `--dry-run` | 打印解析后的配置和计划，不访问硬件 |
| `--status-only` | 不启停组件，只读取当前 MAVROS/bridge/monitor ROS 链路状态 |
| `-h`、`--help` | 显示帮助 |

配置优先级：脚本安全默认值、私有 env、命令行。`--diagnostic-only` 强制禁发；
`--publish-vision` 无法绕过门控。

### 7.2 会让整次启动退出的条件

无论是否请求视觉发布，以下基础条件失败都会终止启动并清理本会话：

- 配置格式、路径、依赖或 ROS 环境无效；
- 有冲突活动会话、重复节点或 vision 输入已有其他发布者；
- roscore 无法创建/复用；
- MAVROS FCU 未在等待窗口内达到 `connected=true、armed=false`；
- `/Odometry` 未在启动器规定的等待窗口内出现；
- 已启用的 bridge/monitor 没有发布首条诊断；
- 持续模式下由本会话启动的任一组件进程退出。

持续模式检查组件是否仍存活，但监控器是只读的，不会反向关闭 bridge。真正的数据断流、
FCU/timesync 丢失和跳变保护由 bridge 自身处理。

### 7.3 只阻止视觉发布的门控

以下条件失败通常不会阻止诊断栈启动，而是把 `PUBLISH_EFFECTIVE` 保持为 `0`：

- 未请求 `PUBLISH_VISION`，或指定 `--diagnostic-only`；
- `MOUNT_CONFIRMED=0`；
- 方向未确认，且既不是限时测试也没有高风险持续发布许可；
- `START_BRIDGE=0` 或 `START_MONITOR=0`；
- Airy UDP `rmem_max` 小于 `4194304`；
- MAVROS vision 输入没有正确 subscriber；
- autopilot 不是 PX4，或固件不匹配默认 1.13.x；
- PX4 参数表无法同步；
- `EKF2_AID_MASK` 不含 EV position `8`，含未获准 EV yaw `16`，或含被禁止的
  EV rotation `64`/未实现 EV velocity `256`；
- `EKF2_HGT_MODE!=3`、`SYS_MC_EST_GROUP!=2`；
- `EKF2_EV_POS_X/Y/Z` 任一非零或无法读取。

`--test-seconds N` 允许 `DIRECTION_TEST_CONFIRMED=0` 时做限时真实发布，但不会绕过
publish 请求、安装确认、UDP、subscriber、PX4 参数、bridge/monitor 等其他门。
`EKF2_EV_DELAY` 只记录，不会由脚本修改。全部 PX4 检查是当前 1.13.x 方案，不应直接
套到其他版本。

### 7.4 限时测试验收

只有使用 `--test-seconds` 时，倒计时结束才会综合断言：组件存活、飞控始终未解锁、
FAST-LIO 输入频率/新鲜度、视觉实际输出、bridge/monitor 诊断、timesync、PX4 通用
solution flags 和 local pose 连续性等，并写 `final_status.txt`。监控仍会保留
`EV_FUSION_UNVERIFIED`，限时测试通过也不等于 EV 融合或飞行验收通过。

### 7.5 运行目录

默认目录：

```text
runtime/airy_px4/<会话>/
  session.meta
  processes.tsv
  logs/ros/             # roslaunch 日志目录
  logs/ros_home/        # 本会话 ROS_HOME
  logs/roscore.log      # 仅本会话创建 roscore 时
  logs/mavros.log       # 仅 START_MAVROS=1 时
  logs/fastlio.log      # 仅 START_FASTLIO=1 时
  logs/bridge.log       # 仅 START_BRIDGE=1 时
  logs/monitor.log      # 仅 START_MONITOR=1 时
  final_status.txt       # 限时测试结束后
runtime/airy_px4/active_<UAV>.session   # 活动期间的会话指针
runtime/airy_px4_locks/                 # 固定在项目内的并发锁
```

`SESSION_ROOT` 可设为绝对路径；相对路径会按项目根解析。活动指针随它移动，但锁目录固定
在项目 `runtime/airy_px4_locks/`。历史会话不会由停止器删除。

## 8. 位姿桥

`fastlio_to_mavros.py` 通常只由综合启动器调用。主要输入：

- `/Odometry`；
- `/rslidar_imu_data`；
- `/<UAV>/mavros/imu/data`；
- `/<UAV>/mavros/state`；
- `/<UAV>/mavros/timesync_status`。

输出：

- `/<UAV>/mavros/vision_pose/pose`；
- `/airy_px4/bridge/diagnostics`。

它执行 5 秒左右的未解锁静止对齐，应用 `I -> B` 旋转和杆臂，保留 FAST-LIO 的原始
测量时间并按原生约 10 Hz 发布。它不插值、不重复旧位姿、不发送速度或协方差。
它也不读取 FAST-LIO 特征数、几何退化指标或源 covariance；只要 `/Odometry` 仍满足
时间、频率和跳变门，遮挡/退化场景不保证自动停发。

输入 `/Odometry` 必须是 `header.frame_id=camera_init`、
`child_frame_id=body`。桥同时求出 FAST-LIO 世界 `W ->` 本地 ENU 的静止对齐，输出
PoseStamped 使用 `header.frame_id=odom`，但桥本身不广播该 TF。

常见 bridge 状态：

| 状态 | 含义 |
| --- | --- |
| `BLOCKED_BY_STARTUP_GATE` | 启动器明确禁止视觉发布 |
| `MOUNT_UNCONFIRMED` | 安装外参未确认 |
| `WAITING_FASTLIO` | 等待有效里程计 |
| `FASTLIO_STALE` | 最近源数据过期 |
| `FCU_DISCONNECTED` | 对齐前没有新鲜、已连接的 FCU state |
| `ARMED_BEFORE_ALIGNMENT` | 静止对齐完成前飞控已解锁 |
| `WAITING_STATIONARY_ALIGNMENT` | 等待未解锁静止窗口和双 IMU一致性 |
| `MAVROS_NOT_SUBSCRIBED` | vision 输出没有 MAVROS subscriber |
| `PUBLISHING_NATIVE_RATE` | 正按原始测量时间和频率发布 |
| `FAULT_LATCHED` | 严重故障已锁存，处理原因后重启会话 |

完成对齐后，跳变、源时间断档、receipt stall、timesync/FCU 丢失等严重故障会锁存；
对齐前的异常通常拒绝样本或重新等待。单帧非有限或过期消息可能只被拒绝，后续有效帧
可以恢复。诊断时查看实际键 `latched_fault` 和 `last_rejection`。

离线数学自测：

```bash
source shfile/setup_fastlio2_Airy.bash
python3 shfile/fastlio_to_mavros.py --self-test
```

## 9. 只读监控器

`monitor_airy_px4.py` 聚合 bridge、FAST-LIO、MAVROS state、local pose、timesync、
estimator status 和 PX4 statustext，输出：

```text
/airy_px4/monitor/diagnostics
```

重点状态：

- `DIAGNOSTIC_GATE_BLOCKED`：主动诊断禁发，属预期；
- `BRIDGE_WAITING`：桥仍在初始化/对齐；
- `VISION_STREAM_NOT_READY`：请求发布但视觉流不满足要求；
- `PX4_LOCAL_POSE_NOT_CONTINUOUS`：PX4 下行本地位姿不连续；
- `BENCH_ONLY_POSE_RATE_TOO_LOW`：链路可跑，但真实位姿低于项目飞行门槛；
- `DIRECTION_TEST_NOT_CONFIRMED`：拆桨方向测试尚未确认；
- `EV_FUSION_UNVERIFIED`：现有 ROS 信息不能证明 EKF2 已融合 EV。

监控器故意不输出 `POSITION_DATA_READY`。`ESTIMATOR_STATUS` solution flags 不包含唯一
数据来源信息；进入 POSCTL 也只证明 Commander 接受过模式。融合证据仍需 ULog/uORB。

## 10. 双 IMU 安装旋转标定

标定器只求 Airy IMU `I ->` 飞机 FLU `B` 的旋转，不求：

- Airy LiDAR `L -> I` 内部外参；
- Airy IMU 到飞机重心的杆臂；
- 时间延迟；
- PX4 参数。

拆桨、未解锁，终端 1：

```bash
bash shfile/start_airy_px4.sh --diagnostic-only
```

终端 2：

```bash
source shfile/setup_fastlio2_Airy.bash
python3 shfile/calibrate_airy_body.py
```

默认流程为 3 秒准备、6 秒静止偏置、45 秒三轴动态采集，最大配对时间差 12 ms。按提示
围绕 X/Y/Z 三轴缓慢、充分、双向转动。工具只读，不写 env、不切模式、不解锁；失败时
不会输出可直接采用的外参。

常用参数：

```text
--sensor-topic TOPIC
--fcu-topic TOPIC
--state-topic TOPIC
--expected-fcu-frame FRAME
--confirm-props-removed
--state-timeout S
--start-delay S
--bias-seconds S
--capture-seconds S
--max-pair-dt-ms MS
--self-test
```

质量门参数及默认值：

| 选项 | 默认值 | 作用 |
| --- | ---: | --- |
| `--min-stationary-pairs` | `150` | 最少静止配对样本 |
| `--min-pairs` | `300` | 鲁棒筛选后最少动态配对 |
| `--max-stationary-rate` | `0.06` | 静止去偏后 P95 最大角速度，rad/s |
| `--min-dynamic-rate` / `--max-dynamic-rate` | `0.18 / 1.80` | 有效动态角速度范围，rad/s |
| `--min-rate-ratio` / `--max-rate-ratio` | `0.65 / 1.45` | FCU/Airy 角速度模长比 |
| `--outlier-angle-deg` | `18` | 鲁棒筛选最大角方向残差 |
| `--outlier-rate-error` | `0.25` | 最大角速度向量误差，rad/s |
| `--min-retained-fraction` | `0.65` | 最小保留样本比例 |
| `--min-excitation-ratio` | `0.10` | 三轴方向矩阵最小/最大特征值比 |
| `--max-rms-angle-deg` / `--max-p95-angle-deg` | `6 / 10` | 拟合角残差门槛 |
| `--max-rms-rate-error` | `0.12` | 角速度向量 RMS 上限，rad/s |
| `--min-gravity-norm` / `--max-gravity-norm` | `7.0 / 12.5` | 静止重力模长范围，m/s² |
| `--max-gravity-angle-deg` | `8` | 映射后双 IMU 重力夹角上限 |

以 `python3 shfile/calibrate_airy_body.py --help` 为完整、权威接口。除非有标定数据依据，
不要为了得到 PASS 而放宽质量门。

非交互运行只有在人工确认拆桨后才可使用 `--confirm-props-removed`。输出四元数顺序为
`[x,y,z,w]`，`q` 与 `-q` 等价。填写 env、量取杆臂、完成平移/三轴方向/失效测试后，
才可确认安装。

离线合成数据自测：

```bash
source shfile/setup_fastlio2_Airy.bash
python3 shfile/calibrate_airy_body.py --self-test
```

## 11. 停止器

```text
bash shfile/stop_airy_px4.sh [--config FILE] [--dry-run]
```

停止器优先查找与本机配置、当前 UID 和 boot ID 匹配的活动会话，并以 PID、进程启动
时间和 PGID 校验归属。它会停止会话创建的 Airy/FAST-LIO、bridge、monitor、MAVROS，
以及确由本会话创建的 roscore；复用的已有 ROS master 保留。

它不会向 PX4 发送模式、解锁、参数或控制命令。重复执行时“没有相关进程”视为成功，
不会删除历史日志。

若会话元数据丢失，脚本才按项目绝对路径和 MAVROS namespace 对当前用户做保守 fallback
匹配。fallback 会保留无法证明归属的旧 roscore，但可能停止当前用户从本项目 build
路径手工启动的 Airy/FAST-LIO，以及同 namespace 的 MAVROS。手工并行运行多个同名栈
时先使用：

```bash
bash shfile/stop_airy_px4.sh --dry-run
```

## 12. Livox 保留链路

Livox 使用独立脚本：

```bash
bash shfile/install_fastlio2_Livox.sh
source shfile/setup_fastlio2_Livox.bash
```

它依赖 `environment/Livox-SDK/` 和 `environment/ws_livox/`，不属于 Airy + PX4 综合
启动流程。Airy 主线的权威入口始终是带 `_Airy` 后缀的安装和环境脚本；不要交叉加载
Airy/Livox overlay 或把两套构建目录混用。

## 13. 维护者检查

修改本目录后至少运行：

```bash
for file in shfile/*.sh shfile/*.bash; do
  bash -n "$file"
done
python3 -m py_compile shfile/*.py
source shfile/setup_fastlio2_Airy.bash
python3 shfile/fastlio_to_mavros.py --self-test
python3 shfile/calibrate_airy_body.py --self-test
git diff --check
```

涉及硬件或门控逻辑时，再按以下顺序回归：

1. 驱动话题自检；
2. 固定 rosbag 回放；
3. `--diagnostic-only`；
4. 拆桨、未解锁限时发布；
5. ULog/uORB 融合与失效测试；
6. 三次冷启动重复性。

任何“让脚本更容易发布”的修改都必须保留未解锁检查、安装确认、时间/断流保护、PX4
版本化参数门和日志可追溯性。
