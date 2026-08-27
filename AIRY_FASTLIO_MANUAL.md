# RoboSense Airy + FAST-LIO2 快速上手手册

本文档面向第一次接触本项目的开发者，适用于：

- Ubuntu 20.04 + ROS Noetic；
- x86/AMD64 调试电脑；
- NVIDIA Jetson Orin NX / Orin Nano，以及其他 ARM64 开发板；
- RoboSense Airy、`rslidar_sdk` v1.5.19 和本项目的 FAST-LIO2 Airy 适配。

建议严格按“接线与网络 → 编译 → 驱动自检 → FAST-LIO”的顺序操作。
网络层没有数据时，不要先调 ROS 或 FAST-LIO 参数。

> 飞行前必须使用本台 Airy 的 LiDAR–IMU 标定值，另行标定 Airy 内部 IMU 到
> 飞机 `base_link`/重心的安装外参，并完成录包回放、点云重影和拆桨失效测试。
> 当前外部位姿约 10 Hz，尚未达到本项目 30 Hz 的保守飞行验收门槛。

## 1. 项目结构与数据流

```text
Airy
  ├─ UDP 6699：MSOP 点云
  ├─ UDP 7788：DIFOP 配置、状态和标定
  └─ UDP 6688：内置 IMU
          │
          ▼
    rslidar_sdk_node
      ├─ /rslidar_points
      └─ /rslidar_imu_data
          │
          ▼
    fastlio_mapping
      ├─ /Odometry
      ├─ /cloud_registered
      └─ /cloud_registered_body
```

飞机端综合链路在此基础上继续为：

```text
/Odometry（camera_init -> Airy 内部 IMU body）
          │
          ▼
fastlio_to_mavros.py
  ├─ Airy IMU -> 飞机 base_link/FLU 安装外参
  ├─ 飞控未解锁时的静止世界对齐与双 IMU 重力检查
  ├─ 时间戳、新鲜度、频率、跳变、断流与 timesync 门控
  └─ /liu/mavros/vision_pose/pose（ROS ENU/FLU）
          │
          ▼
MAVROS（ENU/FLU -> PX4 NED/FRD）
          │
          ▼
PX4 EKF2 外部视觉位置
```

桥接器会让飞机外部视觉原点在启动对齐时归零，但不会重置 PX4 home/EKF。它保留
FAST-LIO 测量时间，不重复旧位姿、不伪造高频；断流、时间回退或对齐后突跳会停发
并锁存故障，排障后必须重启综合脚本。Position 模式与解锁仍由遥控器控制。

常用文件：

| 路径 | 作用 |
| --- | --- |
| `environment/rslidar_sdk/` | RoboSense 官方 ROS1 驱动和 `rs_driver` |
| `environment/rslidar_ws/` | 驱动的 Catkin 编译工作空间 |
| `environment/rslidar_sdk/config/config_airy.yaml` | Airy 驱动、端口和话题配置 |
| `config/airy.yaml` | FAST-LIO 输入、时间、外参和发布配置 |
| `launch/mapping_airy.launch` | 同时启动 Airy 驱动与 FAST-LIO |
| `shfile/install_fastlio2_Airy.sh` | 安装依赖并编译驱动和 FAST-LIO |
| `shfile/setup_fastlio2_Airy.bash` | 加载当前 CPU 架构的运行环境 |
| `shfile/check_airy_topics.sh` | 自动检查点云、IMU、字段和时间戳 |
| `shfile/start_airy_px4.sh` | 飞机定位链路综合一键启动与会话记录 |
| `shfile/stop_airy_px4.sh` | 精确停止飞机定位链路相关进程 |
| `shfile/airy_px4.env.example` | 单机串口、安装外参与安全门控模板 |
| `shfile/fastlio_to_mavros.py` | ENU/FLU 坐标桥与失效保护 |
| `shfile/monitor_airy_px4.py` | FAST-LIO/MAVROS/PX4 只读状态监控 |
| `environment/RSView_ubu20_4.3.15_0514/` | x86-64 版 RSView |

本项目为 Airy 增加了 `lidar_type: 5`，接收标准
`sensor_msgs/PointCloud2`。驱动必须使用 `XYZIRT` 点型，其中 `ring` 和
`timestamp` 均会保留；Airy 预处理器使用逐点 `timestamp` 做运动补偿，不能
改回 `XYZI`。

## 2. 接线、供电与网络

### 2.1 正确接线

Airy 调试线束包含数据和供电分支。推荐连接方式：

```text
Airy 航插口
  └─ Airy 接口线/接口盒
       ├─ RJ45 ── 电脑或开发板独立有线网口
       └─ DC 电源 ── Airy 原装或符合规格的电源
```

首次上电检查：

1. 航插完全插入并锁紧；
2. RJ45 插入的是与 Airy 相连的有线网卡；
3. 单独连接电源，不能把普通网口当作 PoE 供电；
4. 原装调试电源规格为 DC 12 V、3.34 A、40 W；使用机载电源时应以设备
   规格书和实际线束定义为准；
5. 接口盒电源指示灯应常亮，雷达旋转部件应正常启动。

有以太网 `carrier` 不代表雷达已经完整启动。若抓包为 0，优先检查电源、
航插、接口盒和网线。

### 2.2 出厂网络参数

| 项目 | 默认值 |
| --- | --- |
| Airy 设备 IP | `192.168.1.200` |
| 电脑/数据目标 IP | `192.168.1.102` |
| 子网掩码 | `255.255.255.0`（`/24`） |
| MSOP | UDP `6699` |
| DIFOP | UDP `7788` |
| IMU | UDP `6688` |
| Web UI | `http://192.168.1.200` |

如果设备曾被修改过，以其当前 Web UI 或抓包结果为准。

### 2.3 设置 Ubuntu 有线网口

先识别接口和 NetworkManager 连接名：

```bash
ip -br link
nmcli -t -f NAME,DEVICE connection show
```

下面以连接名 `有线连接 1` 为例。不要选择正在用于 SSH 的接口。

```bash
AIRY_CONN="有线连接 1"
sudo nmcli connection modify "$AIRY_CONN" \
  ipv4.method manual \
  ipv4.addresses 192.168.1.102/24 \
  ipv4.gateway '' \
  ipv4.dns '' \
  ipv4.never-default yes \
  ipv6.method disabled \
  connection.autoconnect yes
sudo nmcli connection up "$AIRY_CONN"
```

不设置网关，并启用 `ipv4.never-default`，可以避免雷达网口抢占 Wi-Fi 或
其他网口的默认路由。

### 2.4 网络验收

将 `AIRY_IF` 换成实际网卡名：

```bash
AIRY_IF=enx00e04c58006c
ip -br -4 address show dev "$AIRY_IF"
ip route get 192.168.1.200
ping -I "$AIRY_IF" -c 3 192.168.1.200
ip neigh show dev "$AIRY_IF"
sudo tcpdump -c 30 -ni "$AIRY_IF" \
  'arp or udp port 6699 or udp port 7788 or udp port 6688'
```

正常结果应满足：

- 网卡是 `UP`、`LOWER_UP`，地址为 `192.168.1.102/24`；
- 到 `192.168.1.200` 的路由经过 Airy 网卡；
- Ping 无丢包，邻居表能看到 Airy 的 MAC；
- 抓包能看到 MSOP、DIFOP 和 IMU 数据。

Airy 在当前调试电脑上协商为 `100 Mb/s full duplex` 时也能稳定工作，不要
仅因不是 1 Gb/s 就判定故障。是否丢包应以 `ip -s link` 和点云连续性为准。

不知道雷达当前 IP 时，在启动抓包后给雷达重新上电：

```bash
sudo tcpdump -eni "$AIRY_IF" arp
```

Airy 的 ARP 请求中通常同时包含设备源 IP 和它正在寻找的数据目标 IP。若
上电后仍然一帧都没有收到，问题位于供电、线束、接口盒或雷达硬件，不在
ROS 配置。

## 3. 一键安装与编译

所有命令都从 FAST_LIO 根目录执行，例如：

```bash
cd /home/liu/study/FAST_LIO
```

首次安装：

```bash
bash shfile/install_fastlio2_Airy.sh
```

Jetson Orin 或内存较小的平台推荐：

```bash
bash shfile/install_fastlio2_Airy.sh --jobs 2
```

如果编译器因内存不足被杀死，改为 `--jobs 1`。有桌面的调试电脑可使用
`--desktop-full` 安装 RViz；APT 索引刚更新过时可使用
`--skip-apt-update`。

默认模式安装 `libpcap-dev`，同时支持在线 UDP 和驱动直接读取 PCAP。若只使用
在线 Airy，可以不安装 libpcap：

```bash
bash shfile/install_fastlio2_Airy.sh --online-only --jobs 2
```

依赖已经完整安装、只需增量编译时，可保证不调用 `sudo/apt`：

```bash
bash shfile/install_fastlio2_Airy.sh --build-only --online-only --jobs 2
```

若已有 NetworkManager 有线连接，也可以在完整安装时显式配置 Airy 专网：

```bash
bash shfile/install_fastlio2_Airy.sh --jobs 2 \
  --configure-network '有线连接 1'
```

`--build-only` 不能与 `--configure-network` 同时使用。安装脚本要求 Ubuntu
20.04 的 APT/ROS Noetic 软件源已经可用，并且不会在线补齐缺失源码。

安装脚本会：

1. 请求 sudo 权限并检测 Ubuntu、ROS 和 CPU 架构；
2. 安装 PCL、Eigen、yaml-cpp、libpcap、Catkin 等依赖；
3. 以 `POINT_TYPE=XYZIRT`、`ENABLE_IMU_DATA_PARSE=ON` 编译驱动；
4. 以 `ENABLE_LIVOX_SUPPORT=OFF` 编译 Airy 版 FAST-LIO；
5. 检查程序、动态库、ROS 包和 launch 文件。

### 3.1 为什么根目录没有普通的 `devel/`

驱动使用 `catkin_make`，但脚本按 ROS 版本和 CPU 架构隔离输出，避免把 x86
产物复制到 ARM 板后误用：

```text
# amd64
environment/rslidar_ws/devel/noetic-amd64/
build/noetic-amd64-airy/devel/

# arm64 / Jetson Orin
environment/rslidar_ws/devel/noetic-arm64/
build/noetic-arm64-airy/devel/
```

因此不要手工寻找根目录 `devel/setup.bash`，也不要复制其他架构的构建目录。

### 3.2 每个新终端都要加载环境

```bash
source shfile/setup_fastlio2_Airy.bash
```

该脚本会自动选择本机架构的编译结果，并默认设置：

```text
ROS_MASTER_URI=http://127.0.0.1:11311
ROS_HOSTNAME=未设置
ROS_IP=未设置
```

这可以清除 `.bashrc` 或旧网络留下的无效 ROS 地址，避免 `roslaunch` 一直
连接不存在的主机。成功输出类似：

```text
[INFO] Airy + FAST-LIO2 环境已加载（noetic-amd64，ROS master: http://127.0.0.1:11311）。
```

## 4. 首次运行与验收

### 4.1 先单独验证 Airy 驱动

终端一：

```bash
cd /home/liu/study/FAST_LIO
source shfile/setup_fastlio2_Airy.bash
roslaunch rslidar_sdk start_airy.launch
```

终端二：

```bash
cd /home/liu/study/FAST_LIO
source shfile/setup_fastlio2_Airy.bash
bash shfile/check_airy_topics.sh
```

通过时应看到：

```text
[OK] /rslidar_points: sensor_msgs/PointCloud2, ... 点
[OK] 点字段: intensity, ring, timestamp, x, y, z
[OK] /rslidar_imu_data: sensor_msgs/Imu
[OK] 点云/IMU 时间差: ... 秒
[OK] Airy ROS 输入满足 FAST-LIO 接口要求。
```

当前设备的实测基线仅用于快速判断是否明显异常：

| 项目 | 实测值 |
| --- | ---: |
| Ping | 0% 丢包，约 0.15 ms |
| `/rslidar_points` | 约 10 Hz |
| `/rslidar_imu_data` | 约 200 Hz |
| 单帧原始点数 | 本次约 56,000～60,000，随场景变化 |

自检通过后，在终端一按一次 `Ctrl+C`，避免两个驱动同时占用 UDP 端口。

### 4.2 启动 Airy + FAST-LIO

无桌面或无人机板端：

```bash
source shfile/setup_fastlio2_Airy.bash
roslaunch fast_lio mapping_airy.launch
```

调试电脑同时启动 RViz：

```bash
source shfile/setup_fastlio2_Airy.bash
roslaunch fast_lio mapping_airy.launch rviz:=true
```

启动后保持设备静止数秒，等待 `IMU Initial Done`，再缓慢平移和旋转。
初始化时不要剧烈振动或直接起飞。

检查输出：

```bash
rostopic hz /Odometry
rostopic hz /cloud_registered
rostopic echo -n 1 /Odometry/header
```

当前完整链路实测结果：

- `/Odometry` 约 10 Hz；
- `/cloud_registered` 持续发布；
- RViz Fixed Frame 使用 `camera_init`；
- 测试期间没有 MSOP 超时、丢包或 FAST-LIO 运行异常。

主要输出：

| 话题 | 内容 |
| --- | --- |
| `/Odometry` | FAST-LIO 位姿和里程计 |
| `/cloud_registered` | 世界坐标系下的配准点云 |
| `/cloud_registered_body` | 当前机体坐标系点云 |

当前源码虽然注册了 `/Laser_map` 和 `/cloud_effected` publisher，但相应发布调用
已注释；`path_en: false` 时 `/path` 也不发布，不能用这些话题判定算法失败。

如果驱动已在其他 launch 中运行，或正在回放 rosbag：

```bash
roslaunch fast_lio mapping_airy.launch start_driver:=false
```

### 4.3 飞机端综合启动（先诊断）

一键安装脚本也会安装 MAVROS、MAVROS extras、诊断消息、NumPy 与 GeographicLib
数据。飞控板上的 PX4 固件由飞控自行运行，Orin 只启动 MAVROS、Airy、FAST-LIO、
坐标桥和监控器，不启动 PX4 SITL。首次装机执行：

```bash
cd /home/liu/study/FAST_LIO
cp shfile/airy_px4.env.example shfile/airy_px4.env
bash shfile/start_airy_px4.sh --diagnostic-only --test-seconds 30
```

脚本不会自动切模式、解锁、写 PX4 参数、起飞或发送 OFFBOARD 控制量。2026-08-27
改参前诊断读到 `EKF2_AID_MASK=2`，当时未开启视觉位置；FAST-LIO 约 10 Hz，模板
单位旋转下 Airy 与飞控 IMU 的重力夹角为 `178.73°`，所以视觉输出为 0。用户随后
已设置 `EKF2_AID_MASK=24`、`EKF2_HGT_MODE=3`，但尚未由本项目在飞控重启后只读复核。
PX4 v1.13 中 `24=EV_POS+EV_YAW`，`HGT_MODE=3` 为视觉高度；当前默认
`ALLOW_VISION_YAW_FUSION=0` 会因 `24` 含视觉 yaw 而阻断整路视觉发布，并非只去掉
yaw。参数定义见
[PX4 v1.13 参数表](https://docs.px4.io/v1.13/en/advanced_config/parameter_reference)。

FAST-LIO 使用 Airy 内置 IMU，但仅把消息 `frame_id` 改成 `base_link` 不会旋转实际
数值。必须标定并数值应用 Airy IMU 到飞机 `base_link`/FLU 的完整旋转和到重心的
杆臂，把 `camera_init` 世界系对齐到 ROS ENU，再由 MAVROS 正确转换到 PX4 NED/FRD；
点云、IMU、位姿的时钟/频率、MAVROS timesync 和 `EKF2_EV_DELAY` 也必须实测正确。
只有完成安装外参、方向/航向标定、拆桨方向/断流/失位测试并复核 PX4 实际融合状态
后，才可考虑把 `ALLOW_VISION_YAW_FUSION` 改为 `1`。完整门控说明见
`shfile/README.md`。

只有完成上述工作并解决真实位姿频率低于 30 Hz 的问题后，才可在本机私有配置中
请求发布；日常启动命令为：

```bash
bash shfile/start_airy_px4.sh
```

停止 Airy、FAST-LIO、桥接器、监控器和 MAVROS：

```bash
bash shfile/stop_airy_px4.sh
```

该脚本不会向 PX4 发送控制命令；它会优先按综合启动会话记录精确停止进程。启动器
本次创建的 roscore 会随会话停止，复用的已有 ROS master 始终保留。

## 5. 使用 RSView 检查原始点云

仓库内的 RSView 安装包只支持 x86-64，不能直接在 Jetson Orin/ARM64 上运行。

```bash
cd /home/liu/study/FAST_LIO/environment/RSView_ubu20_4.3.15_0514
./run_rsview.sh
```

本项目已经完成以下修复：

- RSView 预配置类型为 `RSAIRY`；
- `run_rsview.sh` 自动设置本地动态库路径；
- 如果压缩或复制过程把 `.so` 软链接变成小文本文件，会自动调用
  `repair_symlinks.sh` 修复；
- 原占位文件备份在 `.flattened_symlink_backup/`。

不要使用 `sudo ./run_rsview.sh`，也不要把 RSView 自带库复制到系统目录。
若仍出现 `libpcap.so.1: file too short`，执行：

```bash
./repair_symlinks.sh
./run_rsview.sh
```

在 RSView 中打开在线雷达时，使用下面这组已经实机验证的参数：

| RSView 参数 | 值 | 说明 |
| --- | --- | --- |
| `Lidar Type` | `RSAIRY` | 状态栏不能显示为 `RSHELIOS` 等其他型号 |
| `Group Address` | `0.0.0.0` | Airy 当前为单播；不要填写雷达 IP `192.168.1.200` |
| `Local Address` | `192.168.1.102` | 电脑雷达网口的静态 IP |
| `MSOP Port` | `6699` | 点云数据端口 |
| `DIFOP Port` | `7788` | 配置和标定数据端口 |

打开成功后，底部状态栏应同时显示 `Lidar Type: RSAIRY`、
`Live Sensor Stream`，并且 `Frame` 持续增长。Airy 的 IMU 使用 UDP `6688`，
但它由 `rslidar_sdk` 处理，不是 RSView 在线点云窗口的端口参数。

RSView 与 ROS 驱动会同时监听相同 UDP 端口。排障时先停止 ROS 驱动，只运行
RSView；RSView 验证完成后再退出 RSView并启动 ROS 驱动。

ARM 板端使用 `rslidar_sdk` 发布 ROS 话题，在远端电脑用 RViz 显示即可。

## 6. 开发时需要修改的配置

### 6.1 驱动配置

文件：`environment/rslidar_sdk/config/config_airy.yaml`

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `lidar_type` | `RSAIRY` | Airy 解码器 |
| `msop_port` | `6699` | 点云端口 |
| `difop_port` | `7788` | 配置和标定端口 |
| `imu_port` | `6688` | 内置 IMU 端口 |
| `host_address` | `0.0.0.0` | 在本机接口接收 |
| `use_lidar_clock` | `false` | 当前设备无 UTC 同步，点云/逐点时间/IMU 使用 Orin 主机时间 |
| `wait_for_difop` | `true` | 收到 DIFOP 后再正常输出 |
| `ros_frame_id` | `rslidar` | 原始消息坐标系 |

默认话题是 `/rslidar_points` 和 `/rslidar_imu_data`。修改话题后，必须同步
修改 `config/airy.yaml` 的 `lid_topic` 和 `imu_topic`。

本机在线测试发现 Airy 设备时钟是开机相对秒，并非 Unix/UTC；若设为 `true`，
`/Odometry` 会与 ROS/PX4 系统时间相差约 1787838951 秒，安全桥会拒绝数据。
因此当前必须保持 `false`。只有为雷达配置并验证 PTP/GPS UTC 同步后，才可使用
雷达时钟。`shfile/check_airy_topics.sh` 会检查点云 header、逐点 `timestamp` 和
IMU 是否都在 ROS 当前时间附近，不能只检查点云与 IMU 彼此接近。

### 6.2 FAST-LIO 配置

文件：`config/airy.yaml`

必须保留的 Airy 参数：

```yaml
common:
    lid_topic: /rslidar_points
    imu_topic: /rslidar_imu_data
    time_sync_en: false
    time_offset_lidar_to_imu: 0.0

preprocess:
    lidar_type: 5
    scan_line: 96
    timestamp_unit: 0
    blind: 0.5
```

Airy 点云和内置 IMU 使用同一雷达时间轴，正常情况下保持
`time_sync_en: false` 和零偏移。只有确认存在固定偏移后，才修改
`time_offset_lidar_to_imu`。

### 6.3 LiDAR–IMU 外参

FAST-LIO 使用：

```text
P_imu = R_L_I × P_lidar + T_L_I
```

`config/airy.yaml` 中的 `extrinsic_T` 和 `extrinsic_R` 是标称初值，不是
所有设备通用的最终值。正式部署应从本台 Airy 的 DIFOP/标定资料获取外参，
确认方向为 LiDAR 到 IMU、平移单位为米，然后检查：

- 静止时里程计是否快速漂移；
- 平移和转动时是否重影、分层或撕裂；
- 旋转矩阵是否满足 `R × Rᵀ ≈ I` 且 `det(R) ≈ 1`。

`extrinsic_est_en: true` 不能代替正确的初始方向和基本标定。

### 6.4 Jetson Orin / ARM64 性能参数

`launch/mapping_airy.launch` 的常用参数：

| 参数 | 当前值 | 调大后的效果 |
| --- | ---: | --- |
| `point_filter_num` | `3` | 保留点更少、CPU 压力更低 |
| `filter_size_surf` | `0.3` | 当前帧更稀疏 |
| `filter_size_map` | `0.4` | 地图更稀疏、内存更低 |

Orin 或其他 ARM64 板端负载过高时，可从 `point_filter_num=4～6`、
`filter_size_surf=0.4～0.6`、`filter_size_map=0.5～0.8` 开始测试。每次只改
一类参数，并用同一个 rosbag 比较定位效果和资源占用。

### 6.5 修改源码后重新编译

安装脚本支持增量构建，修改驱动、FAST-LIO 或配置后可直接重新执行：

```bash
bash shfile/install_fastlio2_Airy.sh --jobs 2 --skip-apt-update
```

不要把 `environment/rslidar_ws/build/devel` 或 x86 构建产物复制到 ARM 板。
应在目标架构上原生编译。

## 7. 录包与回放

首次地面测试至少记录原始点云和 IMU：

```bash
rosbag record -O airy_input.bag \
  /rslidar_points \
  /rslidar_imu_data
```

记录 FAST-LIO 输出：

```bash
rosbag record -O airy_full.bag \
  /rslidar_points \
  /rslidar_imu_data \
  /cloud_registered \
  /Odometry
```

回放时先启动不含驱动的 FAST-LIO：

```bash
# 终端一
source shfile/setup_fastlio2_Airy.bash
roslaunch fast_lio mapping_airy.launch start_driver:=false

# 终端二
source shfile/setup_fastlio2_Airy.bash
rosbag play --pause airy_input.bag
```

按空格开始。每轮回放前重启 FAST-LIO，避免沿用上一轮滤波器和地图状态。

PCD 默认关闭。需要保存时修改 `config/airy.yaml`：

```yaml
pcd_save:
    pcd_save_en: true
    interval: 500
```

开发板上使用正数分段保存。`interval: -1` 会累计全部点，长时间运行可能
耗尽内存。结束时按一次 `Ctrl+C`，等待文件写完后再断电。

## 8. Jetson Orin / ARM64 和多机 ROS

Jetson Orin / ARM64 板端建议：

- 使用 64 位 Ubuntu 用户空间，对应 `arm64`；
- 编译从 `--jobs 2` 开始，内存不足时使用 `--jobs 1`；
- 板端保持 `rviz:=false`，默认关闭 PCD 和轨迹累积；
- 雷达使用独立有线网口，不在该接口上传输无关大流量；
- 长时间运行时监控 `top`、`free -h`、温度和 `ip -s link`。

默认环境脚本使用本机 ROS master。如果确实需要远端 master，在加载环境前
显式保留多机配置：

```bash
export AIRY_KEEP_ROS_NETWORK=1
export ROS_MASTER_URI=http://192.168.0.10:11311
export ROS_IP=192.168.0.20
source shfile/setup_fastlio2_Airy.bash
```

以上地址只是格式示例，必须换成实际可互访的管理网地址。雷达专网地址
`192.168.1.102` 通常不应作为多机 ROS 的管理地址。

## 9. 故障快速定位

按以下顺序检查，不要跳到后面的算法参数：

```text
供电/航插/网线
  → 网卡 IP 与路由
  → UDP 6699/7788/6688
  → rslidar_sdk 点云与 IMU
  → XYZIRT 字段和时间戳
  → FAST-LIO 初始化、外参与输出
```

| 现象 | 检查与处理 |
| --- | --- |
| 网口没有 `LOWER_UP` | 检查网线、接口盒、航插和雷达供电 |
| 有 carrier，但抓包 0 帧 | 雷达未完整上电、线束/接口盒异常或接错网口；这不是 ROS 问题 |
| Ping 不通，但有 ARP/UDP | 检查设备当前 IP、电脑掩码和路由；以 UDP 抓包为最终依据 |
| `ERRCODE_MSOPTIMEOUT` | `sudo tcpdump -ni <接口> udp port 6699`；检查目标 IP 和端口 |
| 有 6699、没有点云 | 检查 7788；`wait_for_difop: true` 会等待 DIFOP |
| 没有 IMU 话题 | 检查 UDP 6688，并确认编译缓存中 `ENABLE_IMU_DATA_PARSE:BOOL=ON` |
| 缺少 `ring/timestamp` | 确认编译缓存中 `POINT_TYPE:STRING=XYZIRT`，重新运行安装脚本 |
| `roslaunch` 一直连接旧 IP | 重新 `source shfile/setup_fastlio2_Airy.bash`；它会使用本机 master |
| 找不到 `devel/setup.bash` | 使用环境脚本；实际目录按 `noetic-amd64/noetic-arm64` 隔离 |
| UDP 端口占用 | 用 `sudo ss -lunp` 查看占用者，停止重复的驱动进程 |
| 有输入但没有 `/Odometry` | 静置等待 IMU 初始化，检查点云/IMU连续性和时间覆盖关系 |
| 点云重影或位姿方向错误 | 依次检查逐点时间戳、时钟、丢包和 LiDAR→IMU 外参 |
| RSView 报 `libpcap...file too short` | 在 RSView 目录运行 `./repair_symlinks.sh` |
| RSView 正常启动但没有点云 | 确认状态栏型号为 `RSAIRY`；在线雷达使用本地地址 `192.168.1.102`、组地址 `0.0.0.0`、端口 `6699/7788`，并先停止 ROS 驱动 |
| APT 找不到 `ros-noetic-*` | 先按 ROS Noetic 官方方式配置 Ubuntu 20.04 软件源，再重新运行安装脚本 |
| 停止驱动时出现 `std::system_error` | 若只在 `Ctrl+C` 退出阶段出现且进程正常结束，可忽略；运行中出现则继续排查 |
| 编译出现 `Killed/cc1plus` | 使用 `--jobs 1`，关闭 RViz、浏览器等高内存程序 |

常用诊断命令：

```bash
ip -br address
ip route
ip -s link
sudo tcpdump -ni any 'udp port 6699 or udp port 7788 or udp port 6688'
rostopic hz /rslidar_points
rostopic hz /rslidar_imu_data
rostopic hz /Odometry
rostopic echo -n 1 /rslidar_points/fields
```

## 10. 飞行前验收

- [ ] 航插锁紧，供电稳定，雷达可持续运行；
- [ ] Airy 和开发板使用固定 IP，三个 UDP 端口均有数据；
- [ ] `check_airy_topics.sh` 完整通过；
- [ ] 点云约 10 Hz、IMU 约 200 Hz，长时间无明显丢包；
- [ ] 点云包含 `ring` 和 `timestamp`；
- [ ] 已写入本台 Airy 的正确 LiDAR–IMU 外参；
- [ ] 静止时里程计稳定，缓慢运动时点云无明显重影或分层；
- [ ] 已完成原始 rosbag 记录和回放；
- [ ] Orin/ARM64 板端在预期时长内没有过热、降频或内存耗尽；
- [ ] FAST-LIO 到机体/飞控坐标系的变换已单独标定；
- [ ] 已完成地面低动态测试后再进入飞行测试。
