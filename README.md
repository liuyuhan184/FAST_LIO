# RoboSense Airy + FAST-LIO2（Jetson Orin 部署版）

本仓库基于 [HKU-MARS/FAST_LIO](https://github.com/hku-mars/FAST_LIO)，当前主线是将
RoboSense Airy 的点云和内置 IMU 接入 FAST-LIO2，并在 NVIDIA Jetson Orin NX / Orin
Nano（ARM64、Ubuntu 20.04、ROS Noetic）上完成一键安装、原生编译和联合启动。

项目已在 Orin 板端完成在线实测：Airy 点云约 **10 Hz**、内置 IMU 约 **200 Hz**，
FAST-LIO 的里程计和配准点云稳定输出约 **10 Hz**。驱动使用 `XYZIRT` 点型，保留
`ring`，并使用逐点 `timestamp` 做运动补偿；不能改回 `XYZI`。

> **飞行安全提示：** [`config/airy.yaml`](config/airy.yaml) 中的 LiDAR–IMU 外参只是
> 标称初值，且不包含 Airy 内部 IMU 到飞机 `base_link`/重心的安装外参。
> `/Odometry` 的 `camera_init -> body` 也不是可直接送入 MAVROS 的 ENU/FLU 位姿。
> 当前 FAST-LIO 实测约 10 Hz，低于本项目外部视觉飞行验收使用的 30 Hz 门槛。
> 未完成这些标定、频率改进和拆桨失效测试前，只允许台架诊断，不能直接飞行。

更完整的接线、RSView、录包、多机 ROS 和参数说明见
[`AIRY_FASTLIO_MANUAL.md`](AIRY_FASTLIO_MANUAL.md)。脚本说明见
[`shfile/README.md`](shfile/README.md)。

## 安装与使用（先看这里）

### 1. 配置雷达网口

当前实测网络参数如下：

| 项目 | 配置 |
| --- | --- |
| Orin 雷达网口 | `eth0` |
| Orin 静态 IP | `192.168.1.102/24` |
| Airy IP | `192.168.1.200` |
| MSOP 点云 | UDP `6699` |
| DIFOP 配置/标定 | UDP `7788` |
| Airy 内置 IMU | UDP `6688` |

当前板端已经将 `eth0` 持久配置为 `192.168.1.102/24`，且该连接没有网关，不会
抢占 Wi-Fi 的默认路由。新板或系统重装后，可修改 `eth0` 现有的 NetworkManager
连接：

```bash
AIRY_CONN="$(nmcli -g GENERAL.CONNECTION device show eth0)"
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

如果 `AIRY_CONN` 显示为 `--`，先新建连接：

```bash
sudo nmcli connection add type ethernet ifname eth0 con-name airy-eth0 \
  ipv4.method manual ipv4.addresses 192.168.1.102/24 \
  ipv4.never-default yes ipv6.method disabled
sudo nmcli connection up airy-eth0
```

确认地址和路由：

```bash
ip -br -4 address show dev eth0
ip route get 192.168.1.200
ping -I eth0 -c 3 192.168.1.200
```

也可以让安装脚本配置一个**已经存在**的有线连接；只有显式传入该选项才会修改
NetworkManager。先用 `nmcli connection show` 确认连接名，并确保没有选中当前
SSH 所使用的管理网连接：

```bash
bash shfile/install_fastlio2_Airy.sh --jobs 2 \
  --configure-network '有线连接 1'
```

### 2. 一键安装并编译

在项目根目录执行：

```bash
cd /home/liu/study/FAST_LIO
bash shfile/install_fastlio2_Airy.sh --jobs 2
```

前提是 Ubuntu 20.04 的 APT 软件源（包括 ROS Noetic 软件源）已经可用，且仓库源码
完整。脚本会安装工程所需的软件包，但不会改写系统的软件源或在线下载缺失源码。

安装脚本会安装依赖，按当前 CPU 架构分别编译 RoboSense 驱动和 FAST-LIO，并检查
程序、动态库、ROS 包与 launch 文件。Airy 驱动会强制使用：

- `POINT_TYPE=XYZIRT`；
- `ENABLE_IMU_DATA_PARSE=ON`；
- 在线数据和 PCAP 解析所需依赖；
- Release 模式的 ARM64 原生产物。

Orin 编译时建议从 `--jobs 2` 开始；如出现 `Killed` 或 `cc1plus` 被终止，改用
`--jobs 1`。APT 索引刚更新过时可增加 `--skip-apt-update`；需要在有桌面的电脑上
安装 RViz 等完整组件时可增加 `--desktop-full`。

三种构建方式：

| 方式 | 命令 | 用途 |
| --- | --- | --- |
| 完整安装（默认） | `bash shfile/install_fastlio2_Airy.sh --jobs 2` | 安装依赖，支持在线 UDP 和离线 PCAP |
| 仅在线雷达 | `bash shfile/install_fastlio2_Airy.sh --online-only --jobs 2` | 不要求 `libpcap-dev`，禁用驱动直接读取 PCAP |
| 只重新编译 | `bash shfile/install_fastlio2_Airy.sh --build-only --online-only --jobs 2` | 不调用 `sudo/apt`，先验证现有依赖再增量构建 |

`--build-only` 不能与 `--configure-network` 同时使用。若本机已经安装
`libpcap-dev`，只重新编译完整版本时应省略 `--online-only`。

源码或配置修改后可直接增量构建。例如完整模式：

```bash
bash shfile/install_fastlio2_Airy.sh --jobs 2 --skip-apt-update
```

ARM64 构建产物位于：

```text
environment/rslidar_ws/devel/noetic-arm64/lib/rslidar_sdk/rslidar_sdk_node
build/noetic-arm64-airy/devel/lib/fast_lio/fastlio_mapping
```

`amd64` 调试机使用对应的 `noetic-amd64` 目录。不同架构的产物彼此隔离，不能把
x86 构建目录复制到 Orin 上使用，也不要寻找根目录下普通的 `devel/setup.bash`。

本次板端已实际通过 `--build-only --online-only` 完成增量编译验证：在线 Airy 不受
影响，但该模式不能由驱动直接读取 PCAP。默认完整安装会安装 libpcap 开发包并恢复
PCAP 解析支持；ROS bag 的记录与回放不受这个选项影响。

### 3. 每次开机后启动

> **板子重启后以及每个新终端中，都必须重新 `source` 环境脚本。**

日常使用只需：

```bash
cd /home/liu/study/FAST_LIO
source shfile/setup_fastlio2_Airy.bash
roslaunch fast_lio mapping_airy.launch
```

该 launch 会同时启动 Airy 驱动和 FAST-LIO。启动后让雷达保持静止数秒，看到
`IMU Initial Done` 后再缓慢移动。不要在 IMU 初始化期间剧烈晃动设备或直接起飞。

板端默认不启动 RViz；有桌面的调试机可使用：

```bash
roslaunch fast_lio mapping_airy.launch rviz:=true
```

正常退出请按一次 `Ctrl+C`。若开启 PCD 保存，应等待写盘完成后再断电。

### 4. 飞机端综合一键启动

一键安装脚本同时安装 MAVROS、MAVROS extras 和所需 GeographicLib 数据。飞机端
综合脚本会启动 Airy、FAST-LIO、MAVROS/PX4 串口链路、安全坐标桥和只读监控器。
PX4 固件仍运行在飞控板上；脚本不会启动 SITL，也绝不会自动切模式、解锁、写参数、
起飞或发送 OFFBOARD 控制量。

首次装机必须先做未解锁诊断：

```bash
cd /home/liu/study/FAST_LIO
cp shfile/airy_px4.env.example shfile/airy_px4.env
bash shfile/start_airy_px4.sh --diagnostic-only --test-seconds 30
```

2026-08-27 的改参前诊断实测飞控链路为 `/dev/ttyTHS0:921600`、PX4 v1.13.3；
当时读到 `EKF2_AID_MASK=2`，没有开启视觉位置。用户随后已在飞控中设置
`EKF2_AID_MASK=24`、`EKF2_HGT_MODE=3`，但这两个新值尚未由本项目在飞控重启后
复跑只读诊断确认，不能覆盖下文的旧测试基线。按
[PX4 v1.13 参数定义](https://docs.px4.io/v1.13/en/advanced_config/parameter_reference)，
`24=8+16` 表示 `EV_POS + EV_YAW`，`HGT_MODE=3` 表示以视觉为主要高度源。

当前安全默认值仍是 `PUBLISH_VISION=0`、`MOUNT_CONFIRMED=0` 和
`ALLOW_VISION_YAW_FUSION=0`。由于 `24` 包含视觉 yaw，启动器会把它视为硬门控并令
整路视觉位姿保持不发布，而不是只去掉 yaw 后继续发位置。`HGT_MODE=3` 也不会绕过
该门控。旧的未解锁诊断还测得：模板单位旋转下 Airy 与飞控 IMU 重力方向相差
`178.73°`，该占位值与当前轴向/IMU 约定明确不兼容。

FAST-LIO 确实使用 Airy 内置 IMU，但修改 ROS 消息的 `frame_id` 只会更改标签，
不会旋转加速度、角速度、位置或姿态数值。送入 PX4 前必须真实完成以下数值链路：

```text
Airy IMU 轴 I
  -> 安装旋转与杆臂 -> 飞机 base_link/FLU B
  -> camera_init 世界系静止对齐 -> ROS ENU
  -> MAVROS -> PX4 NED/FRD
```

同时必须验证点云/IMU/位姿时钟一致、Airy IMU 与 FAST-LIO 频率稳定、MAVROS
timesync 正常，并通过飞行日志实测 `EKF2_EV_DELAY`；仅改坐标系名称不满足要求。

完成以下项目后，才可在本机 `shfile/airy_px4.env` 中设置
`MOUNT_CONFIRMED=1`、`DIRECTION_TEST_CONFIRMED=1` 和 `PUBLISH_VISION=1`：

- 标定 Airy 内部 IMU 到飞机 ROS `base_link`/FLU 的旋转和到重心的平移；
- 拆桨验证前/左/上平移、横滚/俯仰/航向符号及 ENU/FLU 到 NED/FRD 转换；
- 重启飞控后只读复核 `EKF2_AID_MASK=24`、`EKF2_HGT_MODE=3` 及 EKF 实际融合状态；
- 视觉航向独立标定和拆桨方向/航向测试完成前保持
  `ALLOW_VISION_YAW_FUSION=0`，不得绕过启动器门控；
- 保持 `EKF2_EV_POS_X/Y/Z=0`，因为桥已经做杆臂补偿；
- 拆桨完成静止、断雷达停发、PX4 失位降级和三次冷启动测试；
- 根据 PX4 日志和创新量复核高度、航向与 `EKF2_EV_DELAY`，不能只看参数已写入；
- 解决真实外部位姿只有约 10 Hz、低于 30 Hz 飞行验收门槛的问题。

若复核后仍使用含 `EV_YAW` 的 `EKF2_AID_MASK=24`，还必须在航向标定和拆桨航向
测试全部通过后单独设置 `ALLOW_VISION_YAW_FUSION=1`；未满足时保持 `0`，视觉发布
会继续被安全阻断。

完成配置后的日常命令为：

```bash
bash shfile/start_airy_px4.sh
```

停止本项目启动的 Airy、FAST-LIO、桥接器、监控器和 MAVROS：

```bash
bash shfile/stop_airy_px4.sh
```

停止脚本优先使用 `runtime/` 中记录的 PID、启动时间和进程组清单，不使用宽泛
`pkill`；清单不存在时才按本项目绝对路径清理当前用户的本机残留。综合启动器本次
创建的 roscore 会写入清单并随会话停止，复用的已有 ROS master 始终保留。

总体状态在 `/airy_px4/monitor/diagnostics`，桥状态在
`/airy_px4/bridge/diagnostics`。只有状态达到 `POSITION_DATA_READY` 才表示脚本内
门控通过；Position 模式与解锁仍由操作者通过遥控器完成。参数含义、外参定义、
状态表和完整拆桨验收步骤见 [`shfile/README.md`](shfile/README.md)。

## 首次安装后的自检

建议先检查网络，再单独启动驱动，最后启动完整 SLAM。网络层没有 UDP 数据时，
不要先修改 FAST-LIO 参数。

### 1. 检查 Airy 数据包

```bash
sudo tcpdump -c 30 -ni eth0 \
  'arp or udp port 6699 or udp port 7788 or udp port 6688'
```

应能看到来自 `192.168.1.200` 的 MSOP、DIFOP 和 IMU 数据。实测链路为
`100 Mb/s full duplex` 时也可稳定工作；是否异常应以 UDP 连续性和
`ip -s link show eth0` 的错误/丢包计数为准。

### 2. 单独验证驱动与 XYZIRT

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
rostopic hz /rslidar_points
rostopic hz /rslidar_imu_data
```

自检必须确认：

- `/rslidar_points` 是非空 `sensor_msgs/PointCloud2`；
- 点字段包含 `x/y/z/intensity/ring/timestamp`；
- `/rslidar_imu_data` 是有效的 `sensor_msgs/Imu`；
- 点云和 IMU 时间戳有效且处于同一时间轴；
- 点云约 10 Hz、IMU 约 200 Hz。

检查通过后先在终端一按 `Ctrl+C` 停止单独驱动，再运行联合 launch，避免重复的
驱动进程同时监听 UDP 端口。

### 3. 检查 FAST-LIO 输出

```bash
rostopic hz /Odometry
rostopic hz /cloud_registered
rostopic echo -n 1 /Odometry
```

主要输出话题：

| 话题 | 内容 |
| --- | --- |
| `/Odometry` | FAST-LIO 位姿和里程计 |
| `/cloud_registered` | 世界坐标系下的配准点云 |
| `/cloud_registered_body` | 当前机体坐标系点云 |

如果驱动已由其他 launch 启动，或正在回放 rosbag，只启动算法：

```bash
roslaunch fast_lio mapping_airy.launch start_driver:=false
```

## 本机实测状态（2026-08-27）

测试平台为 Jetson Orin NX、Ubuntu 20.04、ROS Noetic、ARM64。以下数据只是本次
环境的验收基线，不是对所有 Airy、Orin 功耗模式、场景和参数组合的保证值。

| 检查项 | 实测结果 |
| --- | --- |
| 点云 | 约 10.0 Hz，96 线完整（`ring=0～95`），约 5.7～6 万点/帧 |
| 逐点时间 | 单帧跨度约 `0.1 s`（实测 `0.099979 s`），字段为 `timestamp` |
| IMU | 约 200 Hz，与最近点云典型时间差约 `0.665 ms` |
| FAST-LIO | `IMU Initial Done`；`/Odometry` 与注册点云约 10 Hz |
| 30 秒静置 | 末端位移约 4 mm，最大偏移约 11 mm，姿态变化约 0.08° |
| Airy 主机时间 | 单帧约 0.099793 s；cloud/point/IMU 均通过 ROS 当前时间新鲜度检查 |
| PX4 串口 | `/dev/ttyTHS0:921600`；PX4 v1.13.3，EKF2，MAVROS timesync RTT 约 1.30 ms |
| PX4 参数（用户后续设置、待复核） | `EKF2_AID_MASK=24`（EV_POS+EV_YAW）、`EKF2_HGT_MODE=3`（视觉高度） |
| 综合诊断（改参前） | 当时 `EKF2_AID_MASK=2`；30 s 全程未解锁，LIO 约 10.01 Hz、视觉输出 0 Hz |
| 安装方向诊断 | 10 s 未解锁复验；单位旋转下双 IMU 重力夹角 `178.73°`，发布被正确阻断 |
| 资源 | 驱动约 12% CPU / 52 MB RSS；FAST-LIO 约 34% CPU / 193 MB RSS |
| 温度 | 测试最高约 56 °C |

启动边界偶尔出现一次残缺扫描警告，但随后数据连续、无重复警告时通常不影响运行。
若警告持续出现，则必须检查 UDP 丢包、网卡统计和供电。

## 常见故障

排查顺序固定为：**供电/接线 → IP/路由 → UDP 端口 → ROS 驱动 → XYZIRT/时间戳
→ FAST-LIO 初始化与外参**。

| 现象 | 处理方法 |
| --- | --- |
| `eth0` 没有 `LOWER_UP` | 检查雷达供电、航插、接口盒和网线；普通网口不能替代 Airy 供电 |
| Ping 不通或抓包为 0 | 确认 Orin 为 `.102/24`、Airy 为 `.200`，检查接错网口、供电和当前设备 IP |
| `ERRCODE_MSOPTIMEOUT` | 用 `tcpdump` 检查 UDP `6699` 是否到达 `eth0` |
| 有 `6699` 但没有点云 | 检查 DIFOP `7788`；驱动默认等待 DIFOP 后正常输出 |
| 没有 IMU 话题 | 检查 UDP `6688`，重新运行安装脚本确认 IMU 解析已启用 |
| 缺少 `ring` 或 `timestamp` | 驱动不是 `XYZIRT` 构建；重新运行一键安装脚本 |
| 新终端找不到 ROS 包/程序 | 重新执行 `source shfile/setup_fastlio2_Airy.bash` |
| `roslaunch` 尝试连接旧 IP | 重新 source 环境脚本；默认会清除旧的 `ROS_IP/ROS_HOSTNAME` 并使用本机 master |
| 有输入但没有 `/Odometry` | 静置等待 `IMU Initial Done`，再检查点云/IMU 频率、时间戳和外参 |
| 点云重影、分层或位姿方向错误 | 依次检查丢包、逐点时间、时钟和 LiDAR→IMU 外参 |
| 编译进程被杀死 | 使用 `--jobs 1`，关闭 RViz、浏览器等高内存程序后重试 |

常用诊断命令：

```bash
ip -br address
ip route
ip -s link show eth0
sudo tcpdump -ni eth0 'udp port 6699 or udp port 7788 or udp port 6688'
rostopic hz /rslidar_points
rostopic hz /rslidar_imu_data
rostopic hz /Odometry
```

## 关键配置与飞行前要求

| 文件 | 作用 |
| --- | --- |
| [`environment/rslidar_sdk/config/config_airy.yaml`](environment/rslidar_sdk/config/config_airy.yaml) | `RSAIRY` 型号、端口、时钟与 ROS 话题 |
| [`config/airy.yaml`](config/airy.yaml) | FAST-LIO 话题、Airy 预处理、外参与发布参数 |
| [`launch/mapping_airy.launch`](launch/mapping_airy.launch) | 驱动与 FAST-LIO 联合启动 |
| [`shfile/install_fastlio2_Airy.sh`](shfile/install_fastlio2_Airy.sh) | 依赖安装、分架构编译和构建验证 |
| [`shfile/setup_fastlio2_Airy.bash`](shfile/setup_fastlio2_Airy.bash) | 加载当前架构的 ROS 运行环境 |
| [`shfile/check_airy_topics.sh`](shfile/check_airy_topics.sh) | 点云、IMU、字段和时间戳自检 |
| [`shfile/start_airy_px4.sh`](shfile/start_airy_px4.sh) | Airy/FAST-LIO/MAVROS/PX4 综合安全启动 |
| [`shfile/stop_airy_px4.sh`](shfile/stop_airy_px4.sh) | 停止综合链路及清理异常残留进程 |
| [`shfile/airy_px4.env.example`](shfile/airy_px4.env.example) | 飞机串口、安装外参与门控模板 |
| [`shfile/fastlio_to_mavros.py`](shfile/fastlio_to_mavros.py) | ENU/FLU 坐标桥与断流/时间/跳变保护 |
| [`shfile/monitor_airy_px4.py`](shfile/monitor_airy_px4.py) | PX4/MAVROS/FAST-LIO 只读验收监控 |

FAST-LIO 的外参定义为 LiDAR 到 IMU：

```text
P_imu = R_L_I × P_lidar + T_L_I
```

正式装机前必须完成：

- 从本台 Airy 的 DIFOP/标定资料获取真实 `extrinsic_R` 和 `extrinsic_T`；
- 确认旋转方向、平移单位和 FAST-LIO 到机体/飞控坐标系的变换；
- 录制原始点云与 IMU rosbag，并完成重复回放；
- 检查静止漂移以及平移/旋转时的点云重影、分层和撕裂；
- 先做地面低动态测试，再逐步进入飞行测试；
- 长时间运行时监控温度、降频、内存和网卡丢包。

`extrinsic_est_en: true` 不能替代正确的初始外参和设备标定。

## 项目改进与上游 FAST-LIO2

FAST-LIO2 是基于迭代扩展卡尔曼滤波和增量 ikd-Tree 的紧耦合激光惯性里程计，
直接使用原始点进行扫描到地图匹配，兼顾实时性和复杂环境下的鲁棒性。本仓库在上游
基础上增加了：

- RoboSense Airy 标准 `PointCloud2` 接入（`lidar_type: 5`）；
- `XYZIRT`、96 线 ring 和逐点时间戳处理；
- Airy 内置 IMU 接入与同源时钟配置；
- Orin/ARM64 与 x86 分架构构建目录；
- 一键依赖安装、编译验证、环境加载、联合启动和话题自检脚本。

上游资料：

- [FAST-LIO2: Fast Direct LiDAR-inertial Odometry](doc/Fast_LIO_2.pdf)
- [FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package](https://arxiv.org/abs/2010.08196)
- [FAST_LIO 上游仓库](https://github.com/hku-mars/FAST_LIO)

## 鸣谢与许可证

感谢 HKU-MARS FAST-LIO/FAST-LIO2、ikd-Tree、IKFoM 的作者与贡献者，以及
RoboSense 提供的 `rslidar_sdk` 和 `rs_driver`。上游还参考了 LOAM、
Livox-Mapping、LINS 和 Loam-Livox 等工作。

本项目主许可证见 [`LICENSE`](LICENSE)（GPL-2.0）。RoboSense 驱动及仓库内其他
第三方组件分别遵循其自身许可证，详见
[`environment/rslidar_sdk/LICENSE`](environment/rslidar_sdk/LICENSE) 和相应源码目录。
