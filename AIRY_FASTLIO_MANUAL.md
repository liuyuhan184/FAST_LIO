# RoboSense Airy + FAST-LIO2 完整项目与二次开发手册

本文档面向第一次接触本仓库、需要理解系统原理或继续开发的工程人员。内容覆盖项目
边界、目录、数据流、安装构建、参数、坐标系标定、PX4/MAVROS 桥接、验收、排障和
扩展方法。

如果目标只是把一台新机器尽快部署起来并运行，请先阅读
[快速部署与操作手册](README.md)；脚本的逐文件接口、命令行选项和状态定义见
[shfile 脚本参考](shfile/README.md)。

> 安全说明：所有涉及飞机、飞控、外部视觉或 Position 模式的首次测试都必须拆下全部
> 桨叶、固定机体并保持飞控未解锁。本项目不会自动切换模式、解锁、起飞、写 PX4
> 参数或发送 OFFBOARD 控制量。ROS 话题存在、遥控器能够进入 Position，均不能单独
> 证明 EKF2 已正确融合外部视觉，更不能替代动态方向、失效保护和 ULog 验收。

## 0. 项目定位与当前能力边界

本仓库以开源 FAST-LIO2 为算法主体，增加并整合了以下工程能力：

- RoboSense Airy 的在线 UDP 驱动、点云和内置 IMU 输入；
- Airy `XYZIRT PointCloud2` 的逐点时间预处理；
- amd64/ARM64 分架构的一键安装与构建；
- Jetson Orin NX/Orin Nano 的无桌面运行路径；
- FAST-LIO 位姿到 MAVROS/PX4 外部视觉位姿的安全桥；
- Airy 内部 IMU 到飞机机体系的双 IMU 旋转标定工具；
- 参数只读检查、安全门控、运行监控、会话日志和精确停止；
- 原有 Livox 构建链路的独立保留。

必须先理解以下限制：

1. 当前算法是局部激光惯性里程计和增量局部地图，不含回环检测、全局图优化、地图
   加载、全局重定位或跨任务地图复用。源码读取了 `map_file_path` 参数，但没有实际
   加载地图。
2. `/Odometry` 随激光扫描更新，当前 Airy 实测约 10 Hz。桥只按原始测量时间发布
   `geometry_msgs/PoseStamped`，不插值、不重复旧位姿、不伪造时间戳，也不发送速度
   或协方差。
3. `FLIGHT_MIN_POSE_RATE_HZ=30` 是本项目的保守飞行验收门槛，不是 PX4 协议的绝对
   下限。当前约 10 Hz 能用于链路和台架验证，但仍低于该门槛。
4. 监控器不能仅靠现有 MAVROS 话题证明 EV 已被 EKF2 融合；
   `ev_fusion_verified` 因而保持为 `False`。观察到 `POSCTL` 只表示 Commander 曾经
   接受模式。
5. 同一 ROS master 当前只支持一套飞机定位链路。`/Odometry`、桥节点和
   `/airy_px4/*` 使用全局名称，仅修改 `UAV_NAME` 不能实现同 master 多机并行。
6. RSView 目录中的程序是 x86-64 版本，不能直接在 Orin ARM64 上运行。

## 1. 系统总览

### 1.1 硬件组成

典型部署包含：

- RoboSense Airy 激光雷达及其独立供电、接口线或接口盒；
- Jetson Orin NX/Orin Nano，或 Ubuntu 20.04 amd64 调试电脑；
- 独立雷达有线网口；
- 运行 PX4 1.13.x 的飞控；
- Orin 与飞控之间的 UART/USB MAVLink 链路；
- 可选的 QGroundControl 管理链路和远端 RViz 调试电脑。

PX4 固件运行在飞控上。Orin 运行 ROS master、MAVROS、Airy 驱动、FAST-LIO、坐标桥和
监控器；综合脚本不会在 Orin 上启动 PX4 SITL。

### 1.2 ROS 节点与话题

```text
Airy UDP 6699/7788/6688
          │
          ▼
  rslidar_sdk_node
    ├─ /rslidar_points          sensor_msgs/PointCloud2, XYZIRT
    └─ /rslidar_imu_data        sensor_msgs/Imu
          │
          ▼
  /laserMapping
    ├─ /Odometry                nav_msgs/Odometry
    ├─ /cloud_registered        世界系配准点云
    ├─ /cloud_registered_body   Airy 内部 IMU 系点云
    ├─ /path                    仅 path_en=true 时发布
    └─ TF camera_init -> body
          │
          ▼
  /fastlio_to_mavros_bridge
    ├─ 安装旋转、杆臂和世界系对齐
    ├─ 时间、频率、跳变、断流、FCU/timesync 安全门
    ├─ /<UAV_NAME>/mavros/vision_pose/pose
    └─ /airy_px4/bridge/diagnostics
          │
          ▼
  MAVROS: ROS ENU/FLU -> PX4 NED/FRD -> MAVLink
          │
          ▼
  PX4 EKF2 外部视觉位置
```

综合链路还启动 `/airy_px4_readiness_monitor`，读取 FAST-LIO、MAVROS 状态、本地位置和估计器
状态，发布 `/airy_px4/monitor/diagnostics`。它是只读监控，不向飞控发控制命令。

### 1.3 主要输入和输出

| 接口 | 类型 | 典型频率 | 说明 |
| --- | --- | ---: | --- |
| `/rslidar_points` | `sensor_msgs/PointCloud2` | 约 10 Hz | 必须包含 `x/y/z/intensity/ring/timestamp` |
| `/rslidar_imu_data` | `sensor_msgs/Imu` | 约 200 Hz | Airy 内部 IMU |
| `/Odometry` | `nav_msgs/Odometry` | 约 10 Hz | pose/pose covariance 有效；`twist` 未填，默认零且不能当速度 |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | 约 10 Hz | FAST-LIO 局部世界系中的点云 |
| `/<UAV>/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | 约 10 Hz | 桥接成功时给 MAVROS 的输入 |
| `/<UAV>/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | 由 PX4/MAVROS 决定 | PX4 估计结果的间接观察，不是原输入回显 |

源码还注册了 `/Laser_map` 和 `/cloud_effected`，但当前对应发布调用已注释；不能把这两个
话题没有数据当作算法失败。`path_en: false` 时 `/path` 同样不会发布。

## 2. 数据、坐标系与时间轴

### 2.1 坐标系定义

本文统一使用：

| 符号 | 坐标系 | 定义 |
| --- | --- | --- |
| `L` | LiDAR | Airy 激光测量坐标系 |
| `I` | Airy IMU | Airy 内部 IMU 坐标系；FAST-LIO 的 `body` |
| `B` | aircraft body | 飞机 ROS `base_link`，FLU：x 前、y 左、z 上 |
| `W` | FAST-LIO world | 每次启动形成的 `camera_init` 局部世界系 |
| `E` | ROS local world | 桥在未解锁静置阶段建立的 ENU 世界系 |

数据变换链为：

```text
LiDAR L
  -- Airy 内部外参 R_IL, t_IL --> Airy IMU I
  -- 安装旋转 R_BI 和杆臂 r_BI --> 飞机 base_link/FLU B
  -- 启动静置对齐 W -> E       --> ROS ENU/FLU
  -- MAVROS                     --> PX4 NED/FRD
```

`/cloud_registered_body` 名称中的 `body` 是 `I`，不是飞机的 `B`。改变 ROS
`frame_id` 字符串不会旋转数值；真正对齐必须在数学上应用外参。

### 2.2 两组不能混用的外参

Airy 内部 LiDAR 到 IMU 外参位于 `config/airy.yaml`：

```text
p_I = R_IL p_L + t_IL
```

飞机安装外参位于私有 `shfile/airy_px4.env`：

- `SENSOR_TO_BODY_Q_*`：把 `I` 中的向量旋转到 `B`，四元数顺序为 `[x,y,z,w]`；
- `SENSOR_POSITION_IN_BODY_*`：从项目明确选定的飞机 `base_link` 原点指向 Airy IMU
  原点的杆臂 `r_BI`，单位米，表达在 `B` 中。

FAST-LIO 输出的是 IMU 原点在 `W` 中的轨迹。桥将其换算为飞机原点，核心关系可写为：

```text
p_EB = R_EW p_WI + t_EW - R_EB r_BI
```

因此 PX4 的 `EKF2_EV_POS_X/Y/Z` 必须保持为零，否则同一杆臂会补偿两次。

### 2.3 `camera_init` 与 ENU

`camera_init` 是 FAST-LIO 每次启动后根据初始状态形成的局部世界系，不应直接宣称为
ENU。桥在飞控连接、未解锁、双 IMU 重力一致且机体静止时，用一段稳定窗口建立
`W -> E` 对齐并把飞机外部视觉原点归零。这个操作不会重置 PX4 home 或 EKF 原点。

### 2.4 时间轴

当前 Airy 配置使用：

```yaml
use_lidar_clock: false
ts_first_point: true
time_sync_en: false
time_offset_lidar_to_imu: 0.0
```

原因是本项目实测设备时钟为开机相对时间，而不是可直接与 ROS/PX4 对齐的 Unix/UTC。
驱动使用主机接收时间，使点云 header、逐点时间和 IMU 处于同一个 ROS 主机时间域。
MAVROS timesync 另行估计 ROS 主机与飞控时钟的映射；PX4 内部时间并不与 ROS 共用同一
纪元。只有完成 PTP/GPS UTC 同步并实测验证后，才可把 `use_lidar_clock` 改为 `true`。

Airy 预处理器读取 XYZIRT 中每个点的绝对 `timestamp`，以时间戳最早的有效点为扫描起点，
转换为毫秒偏移存入 FAST-LIO 点的 `curvature`；时间无序时会排序。里程计时间戳对应
扫描末端，桥保留这个原始测量时间。它不会用当前系统时间覆盖，也不会重复消息升频。

`EKF2_EV_DELAY` 只能依据 PX4 ULog 中的创新和实际链路延迟调整，不能凭一次观测固定
照抄；综合启动器只读记录该参数，不会修改它。

## 3. 仓库目录与文件职责

### 3.1 根目录和算法源码

| 路径 | 作用 |
| --- | --- |
| `README.md` | 新机快速部署与日常操作手册 |
| `AIRY_FASTLIO_MANUAL.md` | 本完整项目与二次开发手册 |
| `.gitignore` | 排除构建物、运行日志、数据包和飞机私有配置 |
| `CMakeLists.txt` | FAST-LIO 构建、依赖、消息和可执行文件定义 |
| `package.xml` | ROS1 包元数据与依赖；保留了上游 Livox 依赖元数据 |
| `src/laserMapping.cpp` | ROS 主节点、测量同步、点面残差、迭代滤波、地图维护和发布 |
| `src/preprocess.cpp/.h` | 不同雷达点型预处理；Airy `lidar_type=5` 适配在这里 |
| `src/IMU_Processing.hpp` | IMU 初始化、传播和逐点运动畸变补偿 |
| `include/use-ikfom.hpp` | 状态、过程模型、观测模型和雅可比 |
| `include/IKFoM_toolkit/` | 迭代误差状态滤波框架 |
| `include/ikd-Tree/` | 增量动态 KD-Tree |
| `include/common_lib.h` | 公共点型、状态和坐标辅助定义 |
| `include/Exp_mat.h`、`so3_math.h` | SO(3) 指数映射与数学工具 |
| `msg/Pose6D.msg` | 扫描内 IMU 传播轨迹使用的数据结构 |
| `doc/` | FAST-LIO2 论文、系统结构图和项目资料 |
| `LICENSE` | 根项目许可证文件 |

### 3.2 配置和启动文件

| 路径 | 作用 |
| --- | --- |
| `config/airy.yaml` | Airy 输入、噪声、内部外参、发布和 PCD 参数 |
| `config/*.yaml` | Livox、Velodyne、Ouster 等上游雷达配置 |
| `launch/mapping_airy.launch` | Airy 驱动与 FAST-LIO 的主启动入口 |
| `launch/mapping_*.launch` | 其他雷达启动入口 |
| `rviz_cfg/loam_livox.rviz` | RViz 显示配置，Airy 也复用该布局 |

### 3.3 外部环境与厂家软件

| 路径 | 作用 |
| --- | --- |
| `environment/rslidar_sdk/` | 随仓库维护的 RoboSense ROS1 驱动 v1.5.19 和 `rs_driver` 源码 |
| `environment/rslidar_sdk/config/config_airy.yaml` | Airy 端口、时钟、距离和 ROS 话题 |
| `environment/rslidar_ws/` | 安装脚本生成的驱动 Catkin 工作区与分架构产物 |
| `environment/RSView_ubu20_4.3.15_0514/` | 厂家 x86-64 可视化/诊断工具 |
| `environment/Livox-SDK/` | 保留的 Livox SDK |
| `environment/ws_livox/` | 保留的 Livox ROS 工作区 |
| `environment/README_AIRY.md` | Airy 驱动目录的补充说明；综合使用以根文档为准 |

`environment/rslidar_sdk/src/rs_driver/` 是仓库内源码树，不是要求执行
`git submodule update` 的 Git 子模块。

### 3.4 脚本和生成物

`shfile/` 包含安装、环境加载、话题自检、坐标标定、桥接、监控和综合启停。逐文件说明
见 [shfile 脚本参考](shfile/README.md)。常见生成目录：

| 路径 | 内容 | 是否提交 |
| --- | --- | --- |
| `build/noetic-<arch>-airy/` | FAST-LIO 分架构构建及 devel overlay | 否 |
| `environment/rslidar_ws/build/noetic-<arch>/` | 驱动构建目录 | 否 |
| `environment/rslidar_ws/devel/noetic-<arch>/` | 驱动运行 overlay | 否 |
| `runtime/airy_px4/` | 综合会话元数据、日志和最终状态 | 否 |
| `Log/` | 跟踪的分析工具，以及被忽略的运行 `txt/log/csv` 数据 | 分情况 |
| `PCD/` | 可选点云地图输出 | 否 |
| `shfile/airy_px4.env` | 本机/本架飞机私有配置和外参 | 否 |

表中标为“否”的路径和 `Log/` 的运行数据已由 `.gitignore` 排除。不要把 amd64 构建
产物复制到 ARM64，也不要提交 rosbag、PCAP、PCD、串口配置或某一架飞机的私有外参。

## 4. 接线、网络、串口与系统准备

### 4.1 Airy 供电和网线

```text
Airy 航插
  └─ 原厂或符合规格的接口线/接口盒
       ├─ RJ45 -> Orin/电脑独立雷达网口
       └─ DC   -> 符合设备规格的独立电源
```

普通 RJ45 不能默认作为 PoE 供电。首次上电应确认航插锁紧、接口定义正确、供电稳定、
网口 `LOWER_UP`，再检查数据。网口有 carrier 只证明物理链路存在，不代表雷达已经发送
UDP。

### 4.2 默认网络参数

| 项目 | 当前默认值 |
| --- | --- |
| Airy IP | `192.168.1.200` |
| 板端/目标 IP | `192.168.1.102/24` |
| 点云 MSOP | UDP `6699` |
| DIFOP | UDP `7788` |
| 内置 IMU | UDP `6688` |
| Web 管理页 | `http://192.168.1.200` |

先识别实际网卡和 NetworkManager 连接名：

```bash
ip -br link
nmcli -t -f NAME,DEVICE connection show
```

不要误改 SSH 管理网口。安装脚本只在显式传入 `--configure-network` 时修改指定连接：

```bash
bash shfile/install_fastlio2_Airy.sh --jobs 2 \
  --configure-network '实际 Airy 有线连接名' \
  --host-cidr 192.168.1.102/24 \
  --lidar-ip 192.168.1.200
```

配置会清空该连接网关、设置 `ipv4.never-default` 并关闭 IPv6，避免雷达专网抢默认
路由。验收命令：

```bash
AIRY_IFACE=enp1s0  # 换成实际雷达网卡名
ip -br -4 address
ip route get 192.168.1.200
ping -c 3 192.168.1.200
sudo timeout 5 tcpdump -c 3 -ni "$AIRY_IFACE" 'udp port 6699'
sudo timeout 10 tcpdump -c 1 -ni "$AIRY_IFACE" 'udp port 7788'
sudo timeout 5 tcpdump -c 3 -ni "$AIRY_IFACE" 'udp port 6688'
```

以 UDP 抓包为最终依据。只有 6699 而没有 7788 时，`wait_for_difop: true` 会让驱动
等待标定/配置数据而不正常输出点云。

### 4.3 UDP 接收缓冲

Airy 数据量较大。完整安装会安装 `shfile/99-fastlio-airy.conf` 到
`/etc/sysctl.d/99-fastlio-airy.conf`。检查：

```bash
sysctl net.core.rmem_max net.core.netdev_max_backlog
```

当前目标为 `rmem_max=8388608`、`netdev_max_backlog=5000`；综合视觉发布硬门至少要求
`rmem_max>=4194304`。只执行 `--build-only` 不会写系统配置。

### 4.4 飞控串口

模板默认：

```text
FCU_URL=/dev/ttyTHS0:921600
```

实际设备可能不同。检查：

```bash
ls -l /dev/ttyTHS0 /dev/serial/by-id 2>/dev/null
groups
```

需要时把当前用户加入 `dialout`，然后注销并重新登录：

```bash
sudo usermod -aG dialout "$USER"
```

同时确认 PX4 对应 TELEM 口协议、波特率、电平和交叉接线正确。不要用不匹配电平直接
连接。

### 4.5 主机时间和多机 ROS

MAVROS timesync、Airy 主机时间和 ROS 时间必须连续。运行时不要突然手工大幅校时。
`setup_fastlio2_Airy.bash` 默认清除旧的 `ROS_IP/ROS_HOSTNAME` 并将 master 设置为
本机，避免上一次网络残留：

```text
ROS_MASTER_URI=http://127.0.0.1:11311
```

确实使用远端 ROS master 时，在 `source` 前显式保留网络变量：

下面的地址只是管理网示例，必须按实际网络修改：

```bash
export AIRY_KEEP_ROS_NETWORK=1
export ROS_MASTER_URI=http://192.168.31.10:11311
export ROS_IP=192.168.31.20
source shfile/setup_fastlio2_Airy.bash
```

这里应使用可互访的管理网，通常不是 `192.168.1.102` 雷达专网。多机 ROS 网络可用不
等于同一 master 支持多架飞机；后者当前仍需整体命名空间改造。上述方式适用于手工
`source` 后运行的 ROS 命令。综合 `start_airy_px4.sh` 只保留 `ROS_MASTER_URI`，会清除
显式 `ROS_IP/ROS_HOSTNAME`，权威支持拓扑仍是本机 master；远端 master 的综合一键
启动需要先改造并验证节点回连地址。

## 5. 新机安装、构建与环境加载

### 5.1 支持环境

权威主线是 Ubuntu 20.04 + ROS Noetic。安装脚本识别 `amd64/i386/arm64/armhf`，但当前
实机重点验证平台是 amd64 调试电脑和 Jetson Orin ARM64。脚本仍硬性要求 Ubuntu；
`--allow-unsupported-os` 只允许在非 20.04 的其他 Ubuntu 版本尝试，不支持 Debian 等
其他发行版，也不代表得到正式支持。

新机必须已经配置可用的 Ubuntu APT 和 ROS Noetic 软件源。脚本会安装依赖，但不会
替用户添加 ROS 软件源，也不会下载仓库中缺失的驱动源码。

### 5.2 安装模式

在仓库根目录执行：

```bash
# 完整依赖、在线 UDP 与离线 PCAP 支持
bash shfile/install_fastlio2_Airy.sh --jobs 2

# Orin 内存不足
bash shfile/install_fastlio2_Airy.sh --jobs 1

# 只需要在线 UDP，不链接 libpcap
bash shfile/install_fastlio2_Airy.sh --online-only --jobs 2

# 依赖已齐，只检查并增量构建；不使用 sudo/apt
bash shfile/install_fastlio2_Airy.sh --build-only --online-only --jobs 2
```

其他选项：

- `--skip-apt-update`：安装前不更新 APT 索引；
- `--desktop-full`：安装含 RViz 的 ROS desktop-full；
- `--allow-unsupported-os`：允许在非 20.04 Ubuntu 尝试；
- `--configure-network NAME`、`--host-cidr`、`--lidar-ip`：配置指定雷达连接。

`--build-only` 保证不调用 sudo/apt，因此不能与网络配置同时使用；它仍会写项目构建
目录。完整选项以 `bash shfile/install_fastlio2_Airy.sh --help` 为准。

### 5.3 构建内容和关键宏

安装脚本执行的 Airy 权威构建配置包括：

- 驱动 Release 构建；
- `POINT_TYPE=XYZIRT`；
- `ENABLE_IMU_DATA_PARSE=ON`；
- 默认支持 PCAP，或在 `--online-only` 下明确关闭；
- FAST-LIO Release、C++14；
- Airy 构建使用 `ENABLE_LIVOX_SUPPORT=OFF`，不要求 Livox CustomMsg。

构建标签为 `noetic-$(dpkg --print-architecture)`。例如 ARM64 的 FAST-LIO overlay 位于
`build/noetic-arm64-airy/devel/`，驱动 overlay 位于
`environment/rslidar_ws/devel/noetic-arm64/`。

`CMakeLists.txt` 当前在 x86 上根据 CPU 数启用 2～3 个匹配线程，非 x86 使用单线程
匹配路径。这是 Orin 性能二次优化的重要入口，但修改后必须用固定 rosbag 做精度和
延迟回归，不能只看 CPU 占用。

### 5.4 环境加载

每个手工运行 ROS 命令的新终端都执行：

```bash
cd /path/to/FAST_LIO
source shfile/setup_fastlio2_Airy.bash
```

它依次加载 ROS Noetic、当前架构的 rslidar overlay 和 FAST-LIO overlay，并导出
`FAST_LIO_ROOT`、`RSLIDAR_SDK_ROOT`、`RSLIDAR_WS`。不要假设根目录存在传统
`devel/setup.bash`。综合启动器会自行加载环境，单独执行 `roslaunch`、`rostopic`、
`rosbag` 或标定器时仍需手工加载。

## 6. 分层首次联调

排障应严格按“硬件网络 → 驱动 → 时间字段 → LIO → MAVROS → 桥 → PX4 融合”的顺序。

### 6.1 网络层

确认板端 IP、到 `192.168.1.200` 的路由和三个 UDP 端口。长时间测试还应观察：

```bash
ip -s link
nstat -az UdpInErrors UdpRcvbufErrors
```

接口丢包或 UDP receive buffer error 持续增加时，先处理网络和系统缓冲，不要放宽桥的
数据间隔门槛掩盖问题。

### 6.2 单独启动驱动

终端 1：

```bash
source shfile/setup_fastlio2_Airy.bash
roslaunch rslidar_sdk start_airy.launch
```

终端 2：

```bash
source shfile/setup_fastlio2_Airy.bash
bash shfile/check_airy_topics.sh
```

自检验证一条点云和一条 IMU、字段、有限数值和时间轴；它不是持续频率或长期无丢包
证明。继续手工观察；`rostopic hz` 持续运行，按 `Ctrl+C` 后再执行下一条：

```bash
rostopic hz /rslidar_points /rslidar_imu_data
rostopic echo -n 1 /rslidar_points/fields
```

通过后停止单独驱动，避免随后出现两个进程同时监听相同 UDP 端口。

### 6.3 启动 Airy + FAST-LIO

板端无 RViz：

```bash
roslaunch fast_lio mapping_airy.launch
```

桌面调试：

```bash
roslaunch fast_lio mapping_airy.launch rviz:=true
```

先保持雷达静止，等待 `IMU Initial Done`，再缓慢移动。`rostopic hz` 持续运行，按
`Ctrl+C` 后再执行下一条：

```bash
rostopic hz /Odometry /cloud_registered
rostopic echo -n 1 /Odometry
```

若驱动已由别处启动或正在回放 bag：

```bash
roslaunch fast_lio mapping_airy.launch start_driver:=false
```

独立验收结束后在 FAST-LIO `roslaunch` 终端按 `Ctrl+C`，确认 `/laserMapping` 和
`/rslidar_sdk_node` 已退出，再进入综合链路；综合启动器会主动拒绝重复节点。

### 6.4 飞机链路只诊断

先创建本机配置：

```bash
test -f shfile/airy_px4.env || \
  cp shfile/airy_px4.env.example shfile/airy_px4.env
```

检查串口和 `UAV_NAME` 后，拆桨、未解锁执行：

```bash
bash shfile/start_airy_px4.sh --diagnostic-only --test-seconds 30
```

诊断模式强制禁止向 MAVROS 发布视觉位姿，即使私有配置请求发布也不例外。此阶段只
验证 roscore、MAVROS、FCU、FAST-LIO、桥和监控器能建立连接。

## 7. FAST-LIO2 原理与源码导读

### 7.1 处理流程

Airy 主线每帧执行：

1. `rslidar_sdk_node` 发布 XYZIRT 点云和内置 IMU。
2. `Preprocess` 的 Airy handler 读取逐点时间，以时间戳最早的有效点为扫描起点，转换为毫秒
   偏移并在必要时按时间排序；异常点、盲区点和明显异常的扫描偏移会被剔除。
3. `laserMapping.cpp` 缓存点云和 IMU，等待 IMU 覆盖扫描末端后组成一个测量组。
4. `IMU_Processing.hpp` 完成初始重力/偏置估计、状态传播和逐点去畸变。
5. 当前帧体素降采样，在 ikd-Tree 中寻找近邻并拟合局部平面。
6. 以点到平面残差构建观测，IKFoM 迭代更新位置、姿态、速度、IMU 偏置、重力和可选
   内部外参。
7. 将新点增量插入 ikd-Tree，并删除滑动局部立方体之外的地图点。
8. 发布里程计、TF、可选点云、路径、日志和 PCD。

### 7.2 Airy 适配要点

- `lidar_type: 5` 对应 RoboSense Airy 标准 `PointCloud2`。
- 当前算法真正用于运动补偿的是逐点 `timestamp`。
- `ring` 被驱动保留并由自检检查，但 Airy raw handler 的当前匹配流程不使用它。
- `scan_line: 96` 是型号描述，在当前 Airy 直接原始点模式中不参与运算。
- 当前 Airy handler 不支持代码中的传统特征提取；应保持
  `feature_extract_enable=false`。
- Airy handler 当前直接把逐点秒时间之差乘以 1000 转为毫秒，并不读取
  `timestamp_unit`；配置中的 `timestamp_unit: 0` 只是保持接口一致，修改它不会改变
  `lidar_type=5` 的换算。若扩展其他时间单位，必须改 handler 并做回归。

### 7.3 状态和地图

滤波状态包括姿态、位置、速度、陀螺和加速度计偏置、重力，以及启用时的 LiDAR–IMU
内部外参。ikd-Tree 提供增量插入、近邻搜索和局部删除，适合实时局部地图维护。

这套局部状态不会因为保存 PCD 就自动获得全局一致性。没有回环约束时，长距离轨迹仍会
累计漂移；重新启动后也不会从旧 PCD 中重定位。

### 7.4 关键源码入口

阅读建议：

1. 从 `launch/mapping_airy.launch` 和 `config/airy.yaml` 理解启动参数；
2. 在 `src/preprocess.cpp` 搜索 Airy/lidar type 5，理解点型与时间；
3. 阅读 `src/IMU_Processing.hpp` 的初始化、传播与 undistort；
4. 阅读 `src/laserMapping.cpp` 的回调、同步、观测更新、地图增量和发布；
5. 再进入 `include/use-ikfom.hpp`、`IKFoM_toolkit/` 和 `ikd-Tree/`。

## 8. 参数体系

参数分为驱动 YAML、FAST-LIO YAML、launch、本机飞机 env 和 PX4 参数五层。修改前先
确定故障属于哪一层。

### 8.1 Airy 驱动参数

文件：`environment/rslidar_sdk/config/config_airy.yaml`

| 参数 | 当前值 | 含义和修改原则 |
| --- | --- | --- |
| `msg_source` | `1` | 在线 UDP 输入 |
| `lidar_type` | `RSAIRY` | 必须匹配 Airy 解码器 |
| `msop_port` | `6699` | 点云端口，需与设备一致 |
| `difop_port` | `7788` | 配置/标定端口 |
| `imu_port` | `6688` | 内置 IMU 端口 |
| `host_address` | `0.0.0.0` | 在本机可用接口接收 |
| `group_address` | `0.0.0.0` | 当前单播配置 |
| `min_distance` | `0.2` m | 驱动近距离裁剪 |
| `max_distance` | `60.0` m | 驱动远距离裁剪 |
| `use_lidar_clock` | `false` | 当前必须用主机时间 |
| `dense_points` | `true` | 输出稠密点 |
| `ts_first_point` | `true` | header/时间以首点为基准 |
| `wait_for_difop` | `true` | 收到 DIFOP 后正常输出 |
| `ros_frame_id` | `rslidar` | 点云和 Airy IMU 的消息 frame 字符串 |
| `ros_queue_length` | `100` | 当前 ROS1 路径不读取；点云/IMU 发布队列源码固定为 `10/1000` |

修改驱动话题后，必须同步修改 FAST-LIO 的 `lid_topic` 和 `imu_topic`。改变点型或 IMU
解析宏需要重新构建，不是重启 launch 就能生效。`ros_frame_id` 只是消息标识，不会把
LiDAR 数值自动旋转到内部 IMU 或飞机坐标系，也不能替代 §2 的两组外参。

### 8.2 FAST-LIO YAML

文件：`config/airy.yaml`

| 参数 | 当前值 | 说明 |
| --- | ---: | --- |
| `common/lid_topic` | `/rslidar_points` | 点云输入 |
| `common/imu_topic` | `/rslidar_imu_data` | Airy 内置 IMU 输入 |
| `time_sync_en` | `false` | Airy PointCloud2 路径不实现自动偏移估计，设 true 也不会自动校时 |
| `time_offset_lidar_to_imu` | `0.0` | 代码执行 `imu_stamp=raw_imu_stamp-offset`；仅经测量后调整 |
| `lidar_type` | `5` | Airy handler |
| `timestamp_unit` | `0` | Airy 当前 handler 不读取此项；逐点字段必须实际为秒 |
| `blind` | `0.5` m | 算法近距离盲区 |
| `acc_cov`、`gyr_cov` | `0.1` | IMU 测量噪声 |
| `b_acc_cov`、`b_gyr_cov` | `0.0001` | 偏置随机游走 |
| `fov_degree` | `360` | 当前虽被读取并计算，但后续匹配未引用，Airy 主线实际不生效 |
| `det_range` | `60.0` m | 局部地图检测范围，应与驱动上限协调 |
| `extrinsic_est_en` | `true` | 在线细化内部外参，不会自动写回 YAML |
| `extrinsic_T/R` | 设备标称初值 | `p_I=R_IL p_L+t_IL`，正式部署应核对本机标定 |

发布项：

- `path_en=false`：不累积 `/path`；
- `scan_publish_en=true`：发布 `/cloud_registered`；
- `dense_publish_en=true`：发布稠密注册点云；
- `scan_bodyframe_pub_en=true`：发布 `/cloud_registered_body`；
- `pcd_save_en=false`：默认不保存 PCD；
- `interval=-1`：若启用 PCD 会一直累计，长时间运行有内存风险。

板端减负首先考虑关闭不需要的注册点云和 body 点云，而不是削弱状态估计输入。

### 8.3 Launch 参数

文件：`launch/mapping_airy.launch`

| 参数 | 当前值 | 影响 |
| --- | ---: | --- |
| `start_driver` | `true` | 是否包含 Airy 驱动 |
| `rviz` | `false` | 板端默认不启 RViz |
| `point_filter_num` | `3` | 越大保留点越少，CPU 降低但细节减少 |
| `max_iteration` | `3` | 每帧滤波迭代上限 |
| `filter_size_surf` | `0.3` m | 当前帧体素尺寸 |
| `filter_size_map` | `0.4` m | 地图体素尺寸 |
| `cube_side_length` | `500` m | 局部地图立方体尺度 |
| `feature_extract_enable` | `false` | Airy 主线必须保持直接点模式 |
| `runtime_pos_log_enable` | `false` | 默认不写位置运行日志 |

Orin 负载高时，可从增加 `point_filter_num`、`filter_size_surf`、`filter_size_map` 和关闭
点云发布开始。每次只改一类参数，用同一个 bag 比较轨迹、重影、延迟、CPU 和内存。

### 8.4 本机飞机配置

文件：`shfile/airy_px4.env`，由模板复制生成。优先级为：

```text
脚本安全默认值 -> airy_px4.env -> 命令行选项
```

该文件会被 Bash `source`，不是普通 dotenv 数据解析器；只允许使用可信、简单的 Shell
键值内容。它按机器和飞机独立维护，已被 Git 忽略。

| 类别 | 变量 | 说明 |
| --- | --- | --- |
| 连接 | `UAV_NAME` | MAVROS 命名空间的一部分 |
| 连接 | `FCU_URL`、`GCS_URL` | 飞控串口和可选 GCS 转发 |
| 连接 | `MAV_SYS_ID`、`MAV_COMP_ID` | MAVLink ID |
| 版本 | `EXPECTED_PX4_MAJOR/MINOR` | 当前只读固件门控 |
| 组件 | `START_ROSCORE/MAVROS/FASTLIO/BRIDGE/MONITOR` | 是否由本会话启动组件 |
| 安全 | `PUBLISH_VISION` | 仅提出发布请求 |
| 安全 | `MOUNT_CONFIRMED` | 确认旋转和杆臂已填写 |
| 安全 | `DIRECTION_TEST_CONFIRMED` | 确认拆桨方向/符号测试完成 |
| 安全 | `ALLOW_APPROXIMATE_DIRECTION_PUBLISH` | 允许方向未确认时持续发布的高风险豁免；有界测试不需要它 |
| 安全 | `ALLOW_VISION_YAW_FUSION` | PX4 使用 EV yaw 时的额外许可 |
| 外参 | `SENSOR_TO_BODY_Q_*` | Airy IMU `I -> B`，`xyzw` |
| 杆臂 | `SENSOR_POSITION_IN_BODY_*` | `base_link` 原点到 Airy IMU 原点，B 系，米 |
| 质量 | `MIN_FASTLIO_RATE_HZ` | 启动和持续发布最低 LIO 频率 |
| 验收 | `FLIGHT_MIN_POSE_RATE_HZ` | 项目飞行门槛；不直接控制发布 |
| 质量 | `MIN_LOCAL_POSE_RATE_HZ` | PX4 本地下行位姿持续性门槛 |
| 对齐 | `ALIGN_STABLE_SECONDS` | 未解锁静止对齐窗口 |
| 时间 | `MAX_SOURCE_AGE_S` | 测量最大新鲜度 |
| 时间 | `MAX_SOURCE_GAP_S` | 相邻 header 测量时间最大间隔 |
| 调度 | `MAX_RECEIPT_STALL_S` | ROS 回调/进程接收停顿门槛 |
| 跳变 | `MAX_POSITION_JUMP_M` | 对齐后位置单步跳变限制 |
| 跳变 | `MAX_ORIENTATION_JUMP_DEG` | 姿态单步跳变限制 |
| 对齐 | `MAX_GRAVITY_MISMATCH_DEG` | 双 IMU 映射后重力夹角限制 |
| 会话 | `SESSION_ROOT` | 可选运行目录 |

`START_*` 不是任意组合：关闭 roscore 需要已有 master；关闭 MAVROS 需要已有
`/<UAV>/mavros`；关闭 FAST-LIO 仍要求 45 秒内看到 `/Odometry`；关闭 bridge 或
monitor 会让视觉发布被禁止。它们主要用于接入已有单机组件，而不是多栈并行。

### 8.5 PX4 1.13.x 参数门控

综合启动器只读检查，不写参数。当前方案要求：

| 参数 | 要求 | 原因 |
| --- | --- | --- |
| `SYS_MC_EST_GROUP` | `2` | 使用 EKF2 |
| `EKF2_AID_MASK` | 必须包含 `8` | 启用 EV position |
| `EKF2_AID_MASK` bit `16` | 仅获准时允许 | EV yaw 需要独立标定和许可 |
| `EKF2_AID_MASK` bit `64` | 禁止 | 当前桥不采用 EV rotation 模式 |
| `EKF2_AID_MASK` bit `256` | 禁止 | 当前桥没有视觉速度输出 |
| `EKF2_HGT_MODE` | `3` | 视觉高度模式 |
| `EKF2_EV_POS_X/Y/Z` | 全部 `0` | 杆臂已由桥补偿，避免重复 |
| `EKF2_EV_DELAY` | 只记录 | 必须通过 ULog 实测调节 |

未使用视觉航向时可按项目方案只开 EV position；若使用 `EKF2_AID_MASK=24`，其中包含
EV position `8` 和 EV yaw `16`，还必须在完成航向/方向验证后设置
`ALLOW_VISION_YAW_FUSION=1`。否则启动器会阻断整路发布，这是安全设计。

这些位定义和数值只针对当前已验证的 PX4 1.13.x 方案。升级 PX4 前必须重新核对官方
参数、MAVROS 插件和消息语义，不能沿用数值而不检查版本。

## 9. Airy、FAST-LIO 与 PX4 的综合运行

### 9.1 启动阶段

`start_airy_px4.sh` 的逻辑顺序是：

```text
读取并校验私有配置
  -> 加载当前架构 ROS 环境
  -> 复用或创建 roscore
  -> 启动 MAVROS，确认 FCU connected 且 disarmed
  -> 读取固件和 PX4 参数，计算视觉发布门控
  -> 启动 Airy + FAST-LIO，等待 /Odometry
  -> 启动坐标桥
  -> 启动只读监控
  -> 持续运行或在限时测试结束时收集状态并清理
```

启动时飞控已解锁会被拒绝。脚本不会为方便测试而自动解锁或切换模式。

### 9.2 视觉发布硬门

视觉位姿只有在以下条件同时满足时才可能发布：

- 明确请求 `PUBLISH_VISION=1` 或命令行 `--publish-vision`；
- 已确认安装外参，且方向已确认、处于限时未解锁测试，或显式允许近似方向；
- bridge 和 monitor 都启用；
- UDP 接收上限满足要求；
- MAVROS 已连接，启动和静止对齐阶段飞控未解锁，vision topic 有 MAVROS subscriber；
- autopilot 和 PX4 1.13.x 版本匹配；
- PX4 参数表已同步且全部门控通过；
- FAST-LIO、双 IMU、timesync、测量新鲜度和静置对齐通过。

`--publish-vision` 只是请求，不能绕过任何硬门。`--diagnostic-only` 优先级最高，强制
禁发。`--test-seconds` 最少 10 秒；限时模式全程要求飞控未解锁。持续模式只要求启动
和静止对齐时未解锁；对齐完成后允许操作者按遥控器流程切模式/解锁，桥会在 FCU、时间
和数据持续健康时继续发布。

### 9.3 桥接器行为

`fastlio_to_mavros.py` 输入 `/Odometry`、Airy IMU、MAVROS IMU、FCU state 和
timesync；输出 PoseStamped 和诊断。它在启动后执行静止世界对齐，数值应用 `I -> B`
旋转及杆臂，并保留原测量时间和原生约 10 Hz 频率。

桥严格要求输入 `header.frame_id=camera_init`、`child_frame_id=body`；输出
PoseStamped 的 `header.frame_id=odom`。这里的 `odom` 只标识桥建立的局部 ENU 世界，
桥本身不发布对应 TF。除 env 中可调阈值外，还有以下代码默认门槛：

| 参数 | 默认值 |
| --- | ---: |
| Airy/FCU IMU 最大年龄 | `0.25 s` |
| FCU state、timesync 最大年龄 | `2.0 s` |
| timesync 最大 RTT | `20 ms` |
| timesync offset 标准差上限 | `5 ms` |
| 静止对齐最大角速度 | `0.15 rad/s` |
| 对齐期间最大姿态变化 | `3°` |

部分严重故障会锁存，例如对齐后突跳、源时间断档、接收停顿、timesync/FCU 丢失等；
修复原因后需重启综合会话。单帧非有限值或已过期数据可仅拒绝该帧，后续有效帧可能
恢复，不能简单概括为“所有异常都锁存”。

### 9.4 监控器行为

监控器只读观察数据持续性、FCU 状态、PX4 local pose、估计器 solution flags 和模式。
它不会把来源无关的 `ESTIMATOR_STATUS` flags 解释成“Airy EV 已融合”，也不会因为
进入 POSCTL 就输出飞行就绪。完整融合证据必须来自 PX4 ULog/uORB。

### 9.5 日常命令

```bash
# 只诊断
bash shfile/start_airy_px4.sh --diagnostic-only --test-seconds 30

# 拆桨、未解锁的 90 秒发布测试
bash shfile/start_airy_px4.sh --publish-vision --test-seconds 90

# 日常持续运行；私有配置仍需全部门控通过
bash shfile/start_airy_px4.sh

# 查看已运行会话
bash shfile/start_airy_px4.sh --status-only

# 预览停止对象
bash shfile/stop_airy_px4.sh --dry-run

# 正常停止
bash shfile/stop_airy_px4.sh
```

## 10. 雷达与飞机坐标系标定

完整标定至少包含三部分：Airy 内部 `L -> I`、安装旋转 `I -> B`、杆臂 `r_BI`。此外
还要验证时间延迟、世界系方向和 PX4 融合。照片只能给安装方向初值，不能替代三轴动态
标定和量尺。

### 10.1 Airy 内部 LiDAR 到 IMU 外参

`config/airy.yaml` 中：

```yaml
mapping:
    extrinsic_est_en: true
    extrinsic_T: [tx, ty, tz]
    extrinsic_R: [r00, r01, r02,
                  r10, r11, r12,
                  r20, r21, r22]
```

定义必须是：

```text
p_I = R_IL p_L + t_IL
```

优先从本台 Airy 的 DIFOP/厂家标定数据获得，并核对：

- 厂家给出的方向是 `L -> I` 还是 `I -> L`；
- 平移单位是米还是毫米；
- 矩阵是行主序还是列主序；
- `R R^T≈I`、`det(R)≈1`。

若厂家给的是逆变换：

```text
R_IL = R_LI^T
t_IL = -R_LI^T t_LI
```

`extrinsic_est_en=true` 只是在正确初值附近在线细化，不能修复任意错误方向，也不会把
估计值自动写回 YAML。验收应观察静止漂移、转动时墙面重影、边缘分层和轨迹一致性。

### 10.2 安装旋转 `I -> B`

拆下全部桨叶，保证 MAVROS 已连接且飞控未解锁。终端 1：

```bash
bash shfile/start_airy_px4.sh --diagnostic-only
```

终端 2：

```bash
source shfile/setup_fastlio2_Airy.bash
python3 shfile/calibrate_airy_body.py
```

交互式工具会要求人工确认拆桨，默认执行：

1. 延迟 3 秒；
2. 完全静止 6 秒，估计两颗 IMU 的陀螺偏置和重力；
3. 在 45 秒内围绕飞机 X、Y、Z 三轴缓慢、充分、双向转动；
4. 以最大 12 ms 时间差配对 Airy 和 FCU IMU；
5. 鲁棒拟合把 Airy IMU 角速度旋转到飞机 FLU 的四元数。

非交互自动化必须在人工确认拆桨后显式加 `--confirm-props-removed`。话题不是默认
`/liu/...` 时，传入 `--sensor-topic`、`--fcu-topic` 和 `--state-topic`。

主要默认质量门槛包括：

- 静止配对至少 150，动态有效配对至少 300；
- 动态角速度约 `0.18～1.80 rad/s`；
- 保留样本比例至少 0.65；
- 三轴激励比至少 0.10；
- 角方向 RMS 不超过 6°、P95 不超过 10°；
- 角速度向量 RMS 误差不超过 `0.12 rad/s`；
- 映射后双 IMU 重力夹角不超过 8°。

只有工具最终 PASS 时才考虑采用输出。把 `[x,y,z,w]` 填入：

```bash
SENSOR_TO_BODY_Q_X=...
SENSOR_TO_BODY_Q_Y=...
SENSOR_TO_BODY_Q_Z=...
SENSOR_TO_BODY_Q_W=...
```

记录 PASS 结果后回到终端 1，按 `Ctrl+C` 停止 `--diagnostic-only` 会话；异常退出时运行
`bash shfile/stop_airy_px4.sh`。确认旧会话和节点已清理后，再编辑私有 env 并开始有界
发布测试，否则会被活动会话锁拒绝。

`q` 和 `-q` 表示同一个旋转。不要把 `wxyz` 顺序误填成 `xyzw`。标定器只读，不写
配置、不求杆臂、不求 Airy 内部外参、不调时间延迟，也不切模式或解锁。

静态重力只能约束横滚/俯仰，不能唯一确定航向。因此即使重力夹角很小，也必须进行三轴
动态和正方向测试；看到约 180° 重力误差时也不能仅凭照片猜绕 X 还是 Y 翻转。

### 10.3 杆臂 `r_BI`

先明确桥要输出的飞机 `base_link` 原点，并在 PX4/MAVROS 配置、机械图和本文中保持同一
定义；它可能被项目定义在重心、飞控参考点或其他车体基准，三者不能混称。再用卷尺或
CAD 测量该 `base_link` 原点到 Airy 内部 IMU 原点的向量，表达在飞机 FLU：

```text
x：向前为正
y：向左为正
z：向上为正
单位：米
```

例如传感器 IMU 原点位于机体原点前方 8 cm、右侧 2 cm、上方 5 cm，则：

```bash
SENSOR_POSITION_IN_BODY_X=0.08
SENSOR_POSITION_IN_BODY_Y=-0.02
SENSOR_POSITION_IN_BODY_Z=0.05
```

示例只说明符号，不是本机标定值。测量时应明确 Airy 内部 IMU 的物理原点；若只能量到
外壳基准，要结合机械图纸换算。完成后将 PX4 `EKF2_EV_POS_X/Y/Z` 保持为零。

### 10.4 照片近似值的正确用途

照片可用于确定“雷达大致朝上/朝下、接口朝哪边”等先验，并帮助规划拆桨测试，但无法
可靠给出：

- 相机透视下的精确三轴旋转；
- Airy 内部 IMU 轴与外壳轴的关系；
- IMU 原点到飞机重心的厘米级杆臂；
- 航向符号和动态时间延迟。

若必须先跑通全流程，可把近似结果仅保存在本机被忽略的 `airy_px4.env`，保持
`ALLOW_APPROXIMATE_DIRECTION_PUBLISH=0`，并使用 `--test-seconds` 在拆桨、未解锁
状态做有界测试；限时模式本身已提供方向未确认豁免。不得把近似值写入通用模板或复制
给另一架飞机。只有明确接受方向未确认下持续发布的额外风险时，该变量才会解除
`TEST_SECONDS=0` 的方向门，不推荐用于常规流程。

### 10.5 拆桨方向和失效测试

标定、填写并人工复核旋转和杆臂后，先在私有配置中设置：

```bash
MOUNT_CONFIRMED=1
DIRECTION_TEST_CONFIRMED=0
ALLOW_APPROXIMATE_DIRECTION_PUBLISH=0
```

此时使用 `--publish-vision --test-seconds N` 做有界发布；限时模式允许方向尚未确认，
但不会绕过安装、FCU、时钟、PX4 参数等其他门控。在视觉发布期间完成：

1. 沿飞机前、左、上三个正方向分别平移，检查 PX4 local pose 符号；
2. 分别绕机体 X、Y、Z 正方向缓慢转动，检查姿态符号和连续性；
3. 尽量绕已定义的飞机 `base_link` 原点转动，检查位置是否因杆臂错误画大圆；
4. 保持静止，检查重力一致、位置噪声和慢漂；
5. 断开雷达网络或制造真实话题断流，确认桥停止发布并锁存，PX4 按预期拒绝/退出
   Position；
6. 单独测试遮挡、低纹理和几何退化，检查轨迹、创新和模式降级；当前桥不读取 FAST-LIO
   特征数、退化指标或 covariance，仍有扫描/里程计时不保证主动停发；
7. 恢复数据后确认锁存故障不会被错误自动清除，按流程重启；
8. 重启三次，检查世界方向、原点和模式接受的重复性。

平移和转向可在一次应当通过的限时会话中完成；断雷达/断流应另开限时会话，预期桥
锁存、停止发布并让该次验收以失败状态结束。恢复硬件、停止旧会话后再重新启动，不能
为了让失效测试“通过”而放宽门槛。

方向和失效测试全部通过后才可设置：

```bash
MOUNT_CONFIRMED=1
DIRECTION_TEST_CONFIRMED=1
ALLOW_APPROXIMATE_DIRECTION_PUBLISH=0
```

在 ULog 验收完成前仍可保持 `PUBLISH_VISION=0`，继续通过命令行做有界发布。若融合
EV yaw，还必须完成独立航向验证后才允许 `ALLOW_VISION_YAW_FUSION=1`。

### 10.6 ULog/uORB 验收

ROS 和 MAVROS 只能证明数据链存在。PX4 侧至少检查：

- 外部视觉位置/航向融合控制位是否真实生效；
- innovations、innovation test ratios 和 rejection flags；
- estimator reset、position/height/yaw validity；
- EV 延迟与 IMU 时间对齐；
- Position 进入、保持和失位降级过程；
- 静态和动态方向符号；
- 飞控重启后的参数和融合重复性。

`EKF2_EV_DELAY` 应结合日志逐步调节。不要用 PX4 local pose 有 30 Hz 来推断输入也有
30 Hz；本地状态是 EKF 内部高频输出，当前真实 EV 输入仍约 10 Hz。

上述方向、失效和 ULog 验收全部通过后，日常持续运行配置才设为：

```bash
PUBLISH_VISION=1
MOUNT_CONFIRMED=1
DIRECTION_TEST_CONFIRMED=1
ALLOW_APPROXIMATE_DIRECTION_PUBLISH=0
```

`EKF2_AID_MASK` 包含 EV yaw（当前 PX4 1.13 的 value `16`）时，还必须设置已经通过独立
航向验收的 `ALLOW_VISION_YAW_FUSION=1`；否则保持为 `0` 并采用不含 EV yaw 的参数方案。

## 11. 会话、日志与停止机制

### 11.1 会话目录

每次综合启动在默认 `runtime/airy_px4/` 下创建独立目录，典型内容：

```text
runtime/airy_px4/<时间_UAV_PID>/
  ├─ session.meta
  ├─ processes.tsv
  ├─ logs/
  │   ├─ ros/
  │   ├─ ros_home/
  │   ├─ roscore.log          # 本会话创建时
  │   ├─ mavros.log           # 对应 START_* 为 1 时
  │   ├─ fastlio.log
  │   ├─ bridge.log
  │   └─ monitor.log
  └─ final_status.txt       # 限时测试结束后生成
runtime/airy_px4/active_<UAV>.session
runtime/airy_px4_locks/
```

`session.meta` 和 `processes.tsv` 记录 boot ID、启动时间、PID/PGID、进程 start ticks
和组件身份，并结合当前 UID 与 `/proc` owner 做归属校验。组件日志只在本会话实际启动
对应组件时存在；历史日志不会因正常停止被删除。

### 11.2 正常退出

前台运行时按一次 `Ctrl+C`，启动器会清理本会话创建的组件。也可在另一个终端执行：

```bash
bash shfile/stop_airy_px4.sh
```

停止器优先使用活动会话元数据，校验进程归属后先 TERM、再处理未退出的目标。若本次只
复用了已有 roscore，则不会停止该共享 master。

没有可用会话元数据时会使用保守的单栈 fallback 匹配。因此脚本按“单机、单 UAV、单
定位栈”设计；在同一用户下手工运行多个同名栈会增加误匹配风险。先用 `--dry-run`
预览。

## 12. 数据记录、回放、可视化与性能

### 12.1 rosbag

最小原始输入包：

```bash
rosbag record -O airy_input.bag \
  /rslidar_points \
  /rslidar_imu_data
```

开发回归包可增加：

```bash
rosbag record -O airy_full.bag \
  /rslidar_points \
  /rslidar_imu_data \
  /Odometry \
  /cloud_registered
```

回放：

```bash
# 终端 1
source shfile/setup_fastlio2_Airy.bash
roslaunch fast_lio mapping_airy.launch start_driver:=false

# 终端 2
source shfile/setup_fastlio2_Airy.bash
rosbag play --pause airy_input.bag
```

按空格开始。每轮回放前重启 FAST-LIO，避免继承上一轮滤波器和地图状态。bag、PCAP、
PCD 默认被 Git 忽略；重要数据应存入专门的数据归档，不要依赖仓库工作区。

### 12.2 PCAP 与 RSView

默认完整驱动构建保留离线 PCAP 能力；`--online-only` 会禁用它。RSView 仅在 x86-64
运行：

```bash
cd environment/RSView_ubu20_4.3.15_0514
./run_rsview.sh
```

在线 Airy 参数：`RSAIRY`、local `192.168.1.102`、group `0.0.0.0`、MSOP `6699`、
DIFOP `7788`。排障时先停止 ROS 驱动，避免两个程序同时监听端口。如果打包导致 `.so`
软链接变成小文本文件，可运行目录内的 `repair_symlinks.sh`。

Orin 上使用 `rslidar_sdk`，需要可视化时通过管理网在远端电脑运行 RViz。

### 12.3 PCD

需要保存点云时在 `config/airy.yaml` 启用：

```yaml
pcd_save:
    pcd_save_en: true
    interval: 500
```

板端建议用正数分段。`interval: -1` 会累计所有点，长时间运行可能耗尽内存。退出时按
一次 `Ctrl+C` 并等待写盘完成，不要直接断电。

### 12.4 性能观测

统一 bag 和参数下，在不同终端按需记录：

```bash
top
free -h
ip -s link
rostopic hz /rslidar_points /rslidar_imu_data /Odometry
```

Jetson 还应观察温度、功耗模式和降频。性能优化必须同时报告轨迹质量、点云重影、端到
端延迟、丢包、CPU、内存和温度，不能只报告平均帧率。

## 13. 验收矩阵

### 13.1 分层验收

| 层级 | 必须看到 | 不能据此声称 |
| --- | --- | --- |
| 网络 | 三个 UDP 端口持续到达、无增长丢包 | ROS 驱动正确 |
| 驱动 | 点云/IMU、XYZIRT、同一时间轴 | FAST-LIO 精度正确 |
| FAST-LIO | 初始化完成、里程计连续、点云少重影 | 飞机坐标已对齐 |
| MAVROS | FCU connected、timesync 正常、未解锁 | EV 已融合 |
| 桥 | `PUBLISHING_NATIVE_RATE`、vision topic 连续 | 30 Hz 飞行门槛已满足 |
| PX4 模式 | Commander 接受 POSCTL | EKF 创新和方向已通过 |
| ULog/uORB | 融合位、创新、延迟、失效符合预期 | 装桨飞行已经安全 |

### 13.2 飞行前检查表

- [ ] Airy 航插、线束和独立供电固定可靠；
- [ ] 雷达专网固定，6699/7788/6688 长时间持续；
- [ ] `rmem_max` 和实际 socket 缓冲满足要求，UDP buffer error 不增长；
- [ ] 驱动为 `XYZIRT + IMU parse ON`；
- [ ] 点云、逐点时间和 Airy IMU 处于 ROS 主机时间域，MAVROS 到 PX4 的 timesync
      映射健康；
- [ ] 本台 Airy 的 `L -> I` 外参正确；
- [ ] 本架飞机的 `I -> B` 旋转和杆臂已标定；
- [ ] 静态、前/左/上、三轴转动和绕原点测试通过；
- [ ] FAST-LIO 长时间连续，场景退化行为已了解；
- [ ] 飞控版本和全部 PX4 参数门控通过；
- [ ] 真实 EV 输入频率达到项目飞行门槛；
- [ ] ULog/uORB 证明位置/高度/可选航向融合和创新正常；
- [ ] 断雷达、断流、时间跳变和定位失效时 PX4 正确降级；
- [ ] 三次冷启动重复性通过；
- [ ] 先完成安全场地、系留、低高度和逐步扩大包线的飞行计划。

## 14. 分层故障树

| 现象 | 主要原因 | 处理 |
| --- | --- | --- |
| 网口无 `LOWER_UP` | 网线、接口盒、航插或供电 | 从物理层逐项检查 |
| 有 carrier 但抓包为 0 | 雷达未完整上电、目标 IP/端口错误 | 检查供电、Web 配置和目标 IP |
| 只有 6699 没有点云 | DIFOP 7788 缺失且等待开启 | 抓 7788，检查设备配置 |
| 没有 IMU 话题 | 6688 无流量或 IMU 宏关闭 | 抓包并重跑安装构建 |
| 缺 `ring/timestamp` | 驱动不是 XYZIRT | 检查 CMake cache，重建驱动 |
| 时间与 ROS 相差巨大 | 错用设备开机相对时钟 | 保持 `use_lidar_clock=false` |
| `/Odometry` 不输出 | IMU 未初始化、输入断流/不同步 | 静置，检查话题时间覆盖 |
| 点云重影/墙面分层 | 逐点时间、丢包或 `L -> I` 外参错误 | 按时间、网络、外参顺序排查 |
| `/Odometry` 有而 vision 0 Hz | 发布未请求或安全门未通过 | 看启动总结和 bridge 日志 |
| AID mask 为 24 仍被阻断 | 包含 EV yaw 但未许可 | 标定航向后授权，或使用不含 yaw 的方案 |
| source timestamp gap | 原始 header 真实缺帧 | 检查 UDP、系统缓冲和 LIO |
| receipt stall | Orin 调度/负载停顿 | 关闭不必要可视化和点云，查温度/负载 |
| 双 IMU 重力相差近 180° | 安装旋转方向错误 | 拆桨运行三轴动态标定 |
| vision 有流但 PX4 无本地位置 | EKF 拒绝、延迟/创新/参数错误 | 查 ULog/uORB，不要只看 ROS |
| Position 无法进入 | local position 无效或 Commander preflight 失败 | 查 QGC 提示、PX4 日志和监控诊断 |
| 停止后还有重复节点 | 手工进程不属于会话或多栈并行 | `--dry-run`、`rosnode list`、按归属处理 |
| 编译 `Killed/cc1plus` | 内存不足 | `--jobs 1`，关闭高内存程序 |
| 找不到根 `devel/setup.bash` | 本项目使用分架构 overlay | source Airy setup 脚本 |
| RSView 报库文件太短 | 软链接被压平 | 运行 `repair_symlinks.sh` |

常用只读诊断如下。持续运行的 `tcpdump`、`rostopic hz` 应按需单独执行，并用
`Ctrl+C` 结束：

```bash
UAV_NAME=liu  # 换成 airy_px4.env 中的实际值
ip -br address
ip route
ip -s link
sudo tcpdump -ni any 'udp port 6699 or udp port 7788 or udp port 6688'
rostopic list | sort
rostopic hz /rslidar_points /rslidar_imu_data /Odometry \
  "/${UAV_NAME}/mavros/vision_pose/pose" \
  "/${UAV_NAME}/mavros/local_position/pose"
rostopic echo -n 1 /airy_px4/bridge/diagnostics
rostopic echo -n 1 /airy_px4/monitor/diagnostics
```

执行前必须把示例网卡、管理网地址和 `UAV_NAME` 换成真实值。

## 15. 二次开发指南

### 15.1 修改类型与构建范围

| 修改内容 | 最小动作 |
| --- | --- |
| YAML/launch 参数 | 重启对应节点；通常无需重编译 |
| Bash/Python 脚本 | 语法检查、自测、重启综合会话 |
| C++、消息或 CMake | 用相同模式重新构建 FAST-LIO |
| 驱动点型、IMU 宏或 `rs_driver` | 重新运行 Airy 安装构建脚本 |
| PX4 参数 | 在 QGC 修改、重启/确认生效，再做限时验收 |

推荐静态回归：

```bash
for file in shfile/*.sh shfile/*.bash; do
  bash -n "$file"
done
python3 -m py_compile shfile/*.py
source shfile/setup_fastlio2_Airy.bash
python3 shfile/fastlio_to_mavros.py --self-test
python3 shfile/calibrate_airy_body.py --self-test
```

随后依次执行：构建检查、驱动话题自检、固定 bag 回放、诊断模式、拆桨限时发布、PX4
ULog 验收和三次冷启动。

### 15.2 新增另一种雷达

至少需要：

1. 定义或确认 ROS 点类型和逐点时间单位；
2. 在预处理枚举/分支中注册新的 `lidar_type`；
3. 实现过滤、逐点时间归一化、排序和异常处理 handler；
4. 提供驱动 YAML、FAST-LIO YAML 和 launch；
5. 更新话题自检的字段和时间规则；
6. 建立正确 `L -> I` 外参和同步策略；
7. 用 bag 覆盖静止、快速转动、丢包和乱序回归。

不要把 ring 数量或厂家型号名称直接等同于现有 Airy handler 的语义。

### 15.3 提高位姿输出频率

真正的 30～50 Hz 状态输出需要增加 IMU 速率状态传播，并正确处理延迟到达的 LiDAR
校正、状态重放/重传播、协方差和时间一致性。可行方向是：

- 维护最近一次 LiDAR 校正后的高频 IMU 传播状态；
- LiDAR 更新到来时对缓存状态进行校正和重传播；
- 为高频输出提供明确测量/传播时间和协方差；
- 用固定 bag 验证传播漂移、校正跳变和 PX4 延迟。

简单重复最后一帧、线性插值旧位姿、提高 ROS timer 频率或改写时间戳只能制造数字上的
高频，会掩盖断流并破坏融合，不可作为实现方案。

### 15.4 改用 MAVROS odometry 插件

当前桥使用 `PoseStamped`。若改用 `nav_msgs/Odometry`/MAVROS odometry，需要同时定义：

- parent/child frame 和 ENU/FLU 到 NED/FRD 的唯一转换位置；
- 线速度、角速度表达坐标系；
- pose/twist covariance 的来源；
- 杆臂对位置和速度的影响；
- PX4 对位置、速度、姿态、yaw 各融合位的版本化配置。

FAST-LIO 当前 `/Odometry` 只填 pose 和 pose covariance，`twist` 没有填充；桥转换为
PoseStamped 时也不会转发源 covariance。不能只换话题类型而把默认零 twist 当作真实
速度，或把缺失/随意填写的协方差当作有效不确定度。

### 15.5 回环、地图与重定位

建议把回环/图优化或重定位实现为消费 `/Odometry`、关键帧点云和描述子的独立模块，并
明确局部 `W` 到全局地图的变换。需要新增地图格式、版本、加载、初始位姿、失败检测和
重定位质量指标；当前 FAST-LIO 本身不能被描述为已经支持这些功能。

### 15.6 多 UAV 和命名空间

真正并行多机需要对以下内容整体 namespace/remap：

- FAST-LIO 输入输出和节点名；
- 驱动话题及 UDP 端口/网卡；
- bridge、monitor 节点和 `/airy_px4/*` 诊断；
- session ownership、停止匹配和日志目录；
- MAVROS namespace 和 FCU URL。

仅改变 `UAV_NAME` 只影响 MAVROS 路径，不足以解决全局 `/Odometry` 等冲突。

### 15.7 Git 工作流

提交前检查：

```bash
git status --short
git diff --check
git diff -- README.md AIRY_FASTLIO_MANUAL.md shfile/README.md
```

应提交源码、脚本、模板和通用文档；不要提交：

- `shfile/airy_px4.env` 和本机外参；
- `build/`、`devel/`、runtime 日志；
- bag、PCAP、PCD、core 和 Python cache；
- 串口设备身份、私有 GCS 地址或仅对某架飞机成立的安全许可。

## 16. 当前实测基线、已知限制与许可证

### 16.1 已验证基线

截至 2026-08-28，本项目在 Jetson Orin、Ubuntu 20.04、ROS Noetic、Airy 和 PX4
1.13.3 的台架链路上完成过：

- 三路 UDP、XYZIRT 点云、Airy IMU 和 FAST-LIO 输出验证；
- `/Odometry -> PoseStamped -> MAVROS vision` 输入发布链已验证，同时可观察到 PX4
  local pose/POSCTL；后两者不是输入回显，也尚未证明 EV 融合；
- 拆桨、未解锁条件下的限时运行与 POSCTL 接受性测试；
- 参数只读门控、故障锁存、会话清理和停止脚本测试。

当前模板安全默认仍为禁发。历史台架使用过近似安装方向和零杆臂来跑通流程，这类值只
属于本机私有实验，不是通用标定，也不写入本手册。已观察到的真实外部位姿频率约
10 Hz，低于项目 30 Hz 飞行门槛；POSCTL 被接受也没有替代 ULog/uORB 融合证明。

### 16.2 已知工程限制

- 无回环、地图加载和全局重定位；
- 视觉桥无速度、协方差和高频 IMU 传播输出；
- bridge 只按里程计新鲜度、频率、时间和跳变门控，不具备显式的 FAST-LIO 几何退化/
  定位质量停发信号；
- 监控无法独立证明 EV 融合；
- 单 ROS master 只支持一套全局命名的 Airy/PX4 栈；
- Airy 主线不使用 ring 做匹配，也不支持传统特征提取；
- Orin 当前 CMake 匹配线程策略偏保守；
- Livox 脚本是独立旧链路，不属于 Airy + PX4 综合启动；
- `package.xml` 保留了上游 BSD/Livox 元数据，与 Airy 的可选构建和根许可证声明存在
  历史差异，分发前应由项目维护者做一次正式许可证清点。

### 16.3 许可证和上游资料

根目录实际许可证以 [LICENSE](LICENSE) 为准；当前文件为 GPL-2.0。仓库内厂家驱动等
第三方组件可能使用不同许可证，例如 `environment/rslidar_sdk` 的 BSD-3-Clause，
分发或商用时应分别遵守各目录的许可证和设备 SDK 条款。

算法背景资料位于 `doc/Fast_LIO_2.pdf`，上游项目包括 FAST_LIO、ikd-Tree、IKFoM 和
RoboSense rslidar_sdk。论文和上游 README 用于理解算法，不应替代本仓库已经版本化的
Airy 构建、坐标桥和安全门控说明。
