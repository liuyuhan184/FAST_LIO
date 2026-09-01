# RoboSense Airy + FAST-LIO2 快速部署与操作手册

本仓库是在 FAST-LIO2 基础上完成的 RoboSense Airy、Jetson Orin 和
PX4/MAVROS 部署版本。本手册只保留新机安装、首次验收和日常运行所需步骤，适合第一次
接触项目的操作者直接照做。

需要理解源码、算法数据流、全部参数、坐标标定、故障机理或进行二次开发，请阅读
[完整项目与二次开发手册](AIRY_FASTLIO_MANUAL.md)。脚本的完整命令行、环境变量和
状态定义见 [shfile 脚本参考](shfile/README.md)。

> 飞机调试必须拆下全部桨叶并保持飞控未解锁。本项目的综合脚本不会自动切换模式、
> 解锁、起飞、写 PX4 参数或发送 OFFBOARD 控制量。当前 FAST-LIO 真实位姿约 10 Hz，
> 低于本项目设置的 30 Hz 飞行验收门槛；完成链路部署不等于已经具备装桨飞行条件。

## 1. 适用环境

| 项目 | 要求或当前支持 |
| --- | --- |
| 操作系统 | Ubuntu 20.04，64 位系统优先 |
| ROS | ROS Noetic |
| 计算平台 | Jetson Orin NX/Orin Nano（ARM64）或 amd64 调试电脑 |
| 雷达 | RoboSense Airy，点云、DIFOP、内置 IMU 三路 UDP |
| 飞控链路 | PX4 1.13.x + MAVROS；当前模板默认串口 `/dev/ttyTHS0:921600` |
| 项目目录 | 命令中的 `/path/to/FAST_LIO` 代表本仓库根目录，必须换成实际路径 |

新机需要提前具备可用的 Ubuntu APT 和 ROS Noetic 软件源。安装脚本会安装依赖，但不会
添加软件源，也不会在线下载仓库中缺失的源码。

纯净系统先检查基础工具和 ROS 包源：

```bash
command -v git
command -v nmcli
command -v tcpdump
apt-cache policy ros-noetic-ros-base
```

前三条有缺失时可先执行：

```bash
sudo apt-get update
sudo apt-get install -y git network-manager tcpdump
```

`ros-noetic-ros-base` 必须显示非空 `Candidate`。若没有候选版本，应先按 ROS Noetic 的
Ubuntu 20.04 安装说明配置仍可用的软件源，或使用项目已经验证的 Noetic 系统镜像；
不要继续运行安装脚本。为避免过期密钥，本文不复制固定的软件源 key。

Airy 必须使用符合设备规格的独立供电。普通 RJ45 只传输网络数据，不能默认当作 PoE
供电。上飞机前还要确认线束、供电电压和接口定义符合设备资料。

## 2. 获取项目

```bash
git clone https://github.com/liuyuhan184/FAST_LIO.git
cd FAST_LIO
```

如果项目由压缩包或其他方式传入新机，请确认 `environment/rslidar_sdk/`、
`environment/rslidar_sdk/src/rs_driver/`、`src/`、`include/` 和 `shfile/` 均完整。

## 3. 配置 Airy 网口

Airy 当前默认网络参数：

| 项目 | 默认值 |
| --- | --- |
| 雷达 IP | `192.168.1.200` |
| 板端/数据目标 IP | `192.168.1.102/24` |
| MSOP 点云 | UDP `6699` |
| DIFOP 配置与标定 | UDP `7788` |
| 内置 IMU | UDP `6688` |

先找到雷达使用的网卡和 NetworkManager 连接名：

```bash
ip -br link
nmcli -t -f NAME,DEVICE connection show
```

不要选择正在用于 SSH 的管理网口。推荐在完整安装时显式让脚本配置现有有线连接：

```bash
cd /path/to/FAST_LIO
AIRY_CONNECTION='有线连接 1'  # 换成 nmcli 显示的实际连接名
bash shfile/install_fastlio2_Airy.sh --jobs 2 \
  --configure-network "$AIRY_CONNECTION" \
  --host-cidr 192.168.1.102/24 \
  --lidar-ip 192.168.1.200
```

脚本只会修改明确传给 `--configure-network` 的连接：设置静态地址、清空网关、启用
`ipv4.never-default` 并关闭 IPv6，避免雷达网口抢占默认路由。若不希望安装脚本修改
网络，省略这三个网络参数并自行把网卡设为 `192.168.1.102/24`。

网络验收：

先把下面的 `enp1s0` 换成实际雷达网卡名：

```bash
AIRY_IFACE=enp1s0
ip -br -4 address
ip route get 192.168.1.200
ping -c 3 192.168.1.200
sudo timeout 5 tcpdump -c 3 -ni "$AIRY_IFACE" 'udp port 6699'
sudo timeout 10 tcpdump -c 1 -ni "$AIRY_IFACE" 'udp port 7788'
sudo timeout 5 tcpdump -c 3 -ni "$AIRY_IFACE" 'udp port 6688'
```

应同时看到 `6699`、`7788` 和 `6688`。有网口 carrier 但抓不到 UDP 时，应先检查
Airy 供电、航插、接口盒、网线和雷达目标 IP，而不是先修改 FAST-LIO 参数。

## 4. 一键安装与编译

如果上一节没有同时运行安装，执行：

```bash
cd /path/to/FAST_LIO
bash shfile/install_fastlio2_Airy.sh --jobs 2
```

Orin 内存紧张或编译出现 `Killed/cc1plus` 时改为：

```bash
bash shfile/install_fastlio2_Airy.sh --jobs 1
```

依赖已安装、只做增量编译且只使用在线 UDP 时：

```bash
bash shfile/install_fastlio2_Airy.sh --build-only --online-only --jobs 2
```

完整安装会完成以下工作：

- 安装 ROS、PCL、Eigen、MAVROS、GeographicLib、NumPy 等依赖；
- 以 `POINT_TYPE=XYZIRT`、`ENABLE_IMU_DATA_PARSE=ON` 编译 Airy 驱动；
- 编译关闭 Livox 消息依赖的 Airy 版 FAST-LIO2；
- 在 Orin 多核平台默认以 3 个 OpenMP 匹配线程构建；
- 按 CPU 架构隔离构建结果，避免 x86 与 ARM 产物混用；
- 安装 `/etc/sysctl.d/99-fastlio-airy.conf`，提高 UDP 接收上限；
- 运行脚本语法、Python、自测、动态库和 ROS launch 检查。

安装完成后检查：

```bash
sysctl net.core.rmem_max net.core.netdev_max_backlog
```

期望至少为：

```text
net.core.rmem_max = 8388608
net.core.netdev_max_backlog = 5000
```

## 5. 首次雷达与 FAST-LIO 验收

每个新终端都先加载环境：

```bash
cd /path/to/FAST_LIO
source shfile/setup_fastlio2_Airy.bash
```

不要寻找根目录下传统的 `devel/setup.bash`。本项目按 ROS 版本和 CPU 架构保存构建
结果，环境脚本会自动选择正确目录。

### 5.1 单独验证 Airy 驱动

终端 1：

```bash
cd /path/to/FAST_LIO
source shfile/setup_fastlio2_Airy.bash
roslaunch rslidar_sdk start_airy.launch
```

终端 2：

```bash
cd /path/to/FAST_LIO
source shfile/setup_fastlio2_Airy.bash
bash shfile/check_airy_topics.sh
```

自检必须确认：

- `/rslidar_points` 是非空 `sensor_msgs/PointCloud2`；
- 点字段包含 `x/y/z/intensity/ring/timestamp`；
- `/rslidar_imu_data` 正常且数值有限；
- 点云 header、逐点时间和 IMU 与 ROS 系统时间处于同一时间轴。

通过后在终端 1 按一次 `Ctrl+C`，避免重复驱动同时监听 UDP。

### 5.2 启动 FAST-LIO

板端无 RViz：

```bash
cd /path/to/FAST_LIO
source shfile/setup_fastlio2_Airy.bash
roslaunch fast_lio mapping_airy.launch
```

有桌面的调试电脑：

```bash
cd /path/to/FAST_LIO
source shfile/setup_fastlio2_Airy.bash
roslaunch fast_lio mapping_airy.launch rviz:=true
```

飞行配置为降低 Orin 负载，默认关闭 `/cloud_registered`、
`/cloud_registered_body` 和稠密点云。需要临时看降采样后的配准点云时使用：

```bash
roslaunch fast_lio mapping_airy.launch rviz:=true publish_clouds:=true
```

如确有需要，可再加 `dense_cloud:=true` 或 `publish_body_cloud:=true`；这些选项只用于地面
调试，重启后会恢复默认关闭，无需改配置文件。

启动后保持雷达静止，等待日志出现 `IMU Initial Done`，再缓慢移动。检查：

```bash
rostopic hz /rslidar_points /rslidar_imu_data /Odometry
rostopic delay /Odometry
```

本次低负载版本的短时台架实测为：点云约 10 Hz、IMU 约 200 Hz、`/Odometry` 约
10 Hz；连续 100 条里程计的消息年龄平均约 20 ms、P95 约 24 ms、最大约 27 ms。
这个数字只说明本次静态短测没有继续积压，不是长期稳定性、动态定位精度或装桨飞行
验收。场景点数会变化，不能仅按固定点数判断故障。

当前低延迟配置采用“新数据优先”：Airy 的 ROS1 点云发布队列读取
`ros_queue_length=1`；FAST-LIO 点云订阅队列为 `1`、内部点云帧缓冲上限为 `2`，主动
丢弃年龄超过 `0.35 s` 的待处理帧，并禁止发布年龄超过 `0.45 s` 的 `/Odometry`。这会
在 Orin 短时过载时丢旧帧，避免把几十秒前的轨迹继续送给飞控。FAST-LIO 里程计发布、
Python 桥的里程计输入/视觉输出以及监控器的源/视觉输入队列也都是 `1`，因此整条非 IMU
位姿链路采用 latest-only 策略。

当前 Orin 默认档在不降低 10 Hz 雷达/里程计频率、不减少 3 次滤波迭代、保持 360°
视场和 60 m 距离的前提下，将 `point_filter_num` 从 `4` 温和调整为 `5`。若飞行场景
纹理特别稀疏，可临时切回保留更多点的配置：

```bash
roslaunch fast_lio mapping_airy.launch point_filter_num:=4
```

源码还会复用 ikd-tree 近邻查询缓存、直接读取 Airy `PointCloud2`、在无需校时时复用 IMU
消息，并把无数据主循环从 5 kHz 降为 1 kHz。`runtime_pos_log_enable=false` 时不会创建或
逐帧刷新调试矩阵文件；大容量计时数组也只在显式开启运行日志后分配。这些改动不改变
滤波观测模型或坐标变换。

验收完成后在这个 `roslaunch` 终端按一次 `Ctrl+C`，确认 `/laserMapping` 和
`/rslidar_sdk_node` 已退出，再继续飞机综合部署；否则综合启动器会拒绝重复节点。

## 6. 飞机首次部署

以下步骤必须拆下全部桨叶。确认飞控串口存在且当前用户有读写权限：

```bash
ls -l /dev/ttyTHS0
groups
```

若需要加入 `dialout`，执行后必须注销并重新登录：

```bash
sudo usermod -aG dialout "$USER"
```

### 6.1 创建本机私有配置

```bash
cd /path/to/FAST_LIO
test -f shfile/airy_px4.env || \
  cp shfile/airy_px4.env.example shfile/airy_px4.env
nano shfile/airy_px4.env
```

`shfile/airy_px4.env` 已被 Git 忽略。至少检查：

| 配置 | 作用 |
| --- | --- |
| `UAV_NAME` | MAVROS ROS 命名空间前缀 |
| `FCU_URL` | 飞控串口和波特率，例如 `/dev/ttyTHS0:921600` |
| `SENSOR_TO_BODY_Q_*` | Airy 内部 IMU `I` 到飞机 `base_link`/FLU `B` 的 `xyzw` 四元数 |
| `SENSOR_POSITION_IN_BODY_*` | 飞机原点到 Airy IMU 原点的 FLU 杆臂，单位米 |
| `PUBLISH_VISION` | 请求发布视觉位姿；不是绕过安全门控的开关 |
| `MOUNT_CONFIRMED` | 安装旋转和杆臂已标定、复核 |
| `DIRECTION_TEST_CONFIRMED` | 拆桨方向与失效测试已完成 |
| `ALLOW_VISION_YAW_FUSION` | PX4 使用 EV yaw 前的独立许可 |

不要把 `shfile/airy_px4.env` 复制给另一架飞机。模板中的单位四元数只是禁用占位值，
不是本机标定结果。

### 6.2 首次只诊断

```bash
bash shfile/start_airy_px4.sh --diagnostic-only --test-seconds 30
```

该命令会检查 Airy、FAST-LIO、MAVROS/PX4 串口、时间同步、PX4 参数、桥和监控器，
但强制让 `/<UAV_NAME>/mavros/vision_pose/pose` 保持 0 Hz。诊断模式下不能切入
Position 属于预期行为。

### 6.3 标定坐标系

详细方法见[完整手册的坐标系标定章节](AIRY_FASTLIO_MANUAL.md#10-雷达与飞机坐标系标定)。
双 IMU 安装旋转标定的最短流程为：

```bash
# 终端 1：拆桨、未解锁，只诊断
cd /path/to/FAST_LIO
bash shfile/start_airy_px4.sh --diagnostic-only

# 终端 2
cd /path/to/FAST_LIO
source shfile/setup_fastlio2_Airy.bash
source shfile/airy_px4.env
python3 shfile/calibrate_airy_body.py \
  --fcu-topic "/${UAV_NAME}/mavros/imu/data" \
  --state-topic "/${UAV_NAME}/mavros/state"
```

标定器只读，不写文件、不切模式、不解锁。得到 PASS 后人工填写四元数，实测杆臂，
再完成前/左/上平移及三轴正反转向检查。静态重力不能确定航向。标定结束后在终端 1
按 `Ctrl+C` 清理诊断会话；若终端异常退出，再执行 `bash shfile/stop_airy_px4.sh`，之后
才能启动下一次综合测试。

### 6.4 PX4 参数

启动脚本只读参数，不会替操作者写入。当前 PX4 1.13.x 方案要求：

- `SYS_MC_EST_GROUP=2`，使用 EKF2；
- `EKF2_AID_MASK` 至少包含 value `8` 的 EV position；
- 未验收视觉航向时建议只使用 EV position；使用当前 `24=8+16` 时，还必须完成
  EV yaw 验收并设置 `ALLOW_VISION_YAW_FUSION=1`；
- 不启用 value `64` 的 EV rotation，也不启用当前桥未提供的 value `256` EV velocity；
- 当前视觉高度方案为 `EKF2_HGT_MODE=3`；
- `EKF2_EV_POS_X/Y/Z=0`，因为桥已经做杆臂补偿；
- `EKF2_EV_DELAY` 必须依据 PX4 ULog/uORB 创新量实测，不能照抄其他机器。

修改需要重启的参数后，重启飞控并重新运行只读诊断。

### 6.5 拆桨限时发布测试

把已经标定和复核的旋转、杆臂写入私有配置，并先设置
`MOUNT_CONFIRMED=1`。在方向测试尚未完成时保持
`DIRECTION_TEST_CONFIRMED=0`、`ALLOW_APPROXIMATE_DIRECTION_PUBLISH=0`，然后进行
拆桨、未解锁的限时测试：

```bash
bash shfile/start_airy_px4.sh --publish-vision --test-seconds 90
```

在这 90 秒内保持飞控未解锁，可用遥控器尝试进入 Position，并轻缓完成前/左/上平移和
三轴正反转观察。断开雷达的失效测试应另开一次限时会话：预期桥锁存并停止发布、该次
测试以失败状态结束；恢复硬件后先运行停止脚本，再重新启动。

至少确认启动结果显示 FCU `CONNECTED`、FAST-LIO 正常、视觉实际发布门控为 `1`，
桥诊断为 `PUBLISHING_NATIVE_RATE`。Position 模式能被遥控器接受不等于 PX4 已正确
融合 Airy；最终仍要检查 ULog/uORB 的融合控制位、创新量和失位降级。

完成前/左/上、三轴转向、断流和 ULog 验收后，日常配置才设为：

```bash
PUBLISH_VISION=1
MOUNT_CONFIRMED=1
DIRECTION_TEST_CONFIRMED=1
ALLOW_APPROXIMATE_DIRECTION_PUBLISH=0
```

若 `EKF2_AID_MASK=24`，还必须在视觉航向独立验收通过后设置
`ALLOW_VISION_YAW_FUSION=1`；不融合视觉航向时保持为 `0` 并采用匹配的 PX4 参数方案。

## 7. 日常操作

综合启动器会自行加载 ROS 环境，正常启动只需：

```bash
cd /path/to/FAST_LIO
bash shfile/start_airy_px4.sh
```

该命令在终端 1 前台持续运行。启动总结必须再次显示“视觉实际发布门控：1”；若为 0，
不能尝试 Position，应按列出的阻断原因处理。

在终端 2 查看当前状态，不启动新组件：

```bash
cd /path/to/FAST_LIO
bash shfile/start_airy_px4.sh --status-only
source shfile/setup_fastlio2_Airy.bash
rostopic echo -n 1 /airy_px4/bridge/diagnostics
rostopic echo -n 1 /airy_px4/monitor/diagnostics
```

在终端 2 预览停止目标：

```bash
cd /path/to/FAST_LIO
bash shfile/stop_airy_px4.sh --dry-run
```

停止本项目综合链路：

```bash
cd /path/to/FAST_LIO
bash shfile/stop_airy_px4.sh
```

正常退出单独运行的 `roslaunch` 使用一次 `Ctrl+C`。不要直接断电；若启用了 PCD 保存，
应等待写盘结束。

运行日志位于：

```text
runtime/airy_px4/<时间_UAV_PID>/
```

该目录和本机私有配置均不会提交到 Git。

## 8. 关键话题

| 话题 | 含义 | 正常参考 |
| --- | --- | ---: |
| `/rslidar_points` | Airy `XYZIRT` 点云 | 约 10 Hz |
| `/rslidar_imu_data` | Airy 内置 IMU | 约 200 Hz |
| `/Odometry` | FAST-LIO `camera_init → body` 里程计 | 约 10 Hz |
| `/cloud_registered` | `camera_init` 下的配准点云 | 飞行配置默认关闭；地面调试时临时启用 |
| `/cloud_registered_body` | 当前 Airy IMU/body 下点云 | 飞行配置默认关闭；通常无需启用 |
| `/<UAV_NAME>/mavros/vision_pose/pose` | 桥送入 MAVROS 的 ENU/FLU 位姿 | 发布门控通过后约 10 Hz |
| `/<UAV_NAME>/mavros/local_position/pose` | PX4/MAVROS 本地位姿输出 | 只作间接观察 |
| `/airy_px4/bridge/diagnostics` | 坐标桥与失效门控状态 | 约 2 Hz；严重故障可锁存 |
| `/airy_px4/monitor/diagnostics` | 全链路只读监控 | 持续 |

`vision_pose/pose` 是 ROS→MAVROS 的输入，不会在飞控侧以同名 ROS 话题回显。

## 9. 常见问题

| 现象 | 优先处理 |
| --- | --- |
| 有线网口亮但没有 UDP | 检查 Airy 独立供电、航插、接口盒、目标 IP 和实际网卡 |
| 有 6699、没有点云 | 检查 DIFOP 7788；默认 `wait_for_difop: true` |
| 没有 IMU | 检查 UDP 6688，并重新确认驱动以 `ENABLE_IMU_DATA_PARSE=ON` 编译 |
| 点云缺 `ring/timestamp` | 重新运行安装脚本，确认驱动是 `XYZIRT` 构建 |
| `roslaunch` 连接旧 IP | 重新 `source shfile/setup_fastlio2_Airy.bash` |
| 有输入但没有 `/Odometry` | 静置等待 `IMU Initial Done`，再查时间戳、丢包和外参 |
| `/Odometry` 延迟持续增长 | 用 `rostopic delay /Odometry` 确认趋势；检查是否加载 `airy.yaml` 的 `1/2/0.35/0.45` 低延迟参数、是否重复启动节点、点云发布是否误开启，以及 Orin 温度/降频和 CPU 负载 |
| `/Odometry` 正常但 vision 为 0 Hz | 检查 `airy_px4.env`、启动输出中的发布门控和桥诊断 |
| 串口不存在或无权限 | 核对 `FCU_URL`、设备节点和 `dialout`，重新登录后再试 |
| `source timestamp gap` | 检查真实 `/Odometry` header 缺帧、UDP 缓冲和网络丢包 |
| `receipt stall` | 检查 Orin CPU/内存压力、进程调度和 ROS 回调阻塞 |
| `FAULT_LATCHED` | 先修复断流、时间或跳变原因，再重启综合脚本 |
| 能进 Position 但 `EV_FUSION_UNVERIFIED` | 查看 PX4 ULog/uORB；ROS 有话题和模式接受都不是融合证据 |

不要把 `use_lidar_clock` 改为 `true`，除非已通过 PTP/GPS 验证雷达使用 UTC，并确认
点云、逐点时间和 IMU 使用一致的 ROS 时间，且 MAVROS 到 PX4 的 timesync 映射健康。

## 10. 新机部署完成检查表

- [ ] Airy 稳定供电，板端网口为 `192.168.1.102/24`；
- [ ] `6699/7788/6688` 三路 UDP 都能抓到；
- [ ] 一键安装完整通过，UDP 系统参数达到要求；
- [ ] `check_airy_topics.sh` 一次性接口/时间自检通过，并另做持续频率检查；
- [ ] `/Odometry` 约 10 Hz 且 `rostopic delay` 不随运行时间持续增长；配准点云按需临时启用；
- [ ] 飞控串口、MAVROS 连接和 timesync 正常；
- [ ] 本机 `airy_px4.env` 已按这架飞机标定，未复制其他飞机的结果；
- [ ] 只诊断测试通过；
- [ ] 拆桨方向、断流、失位和冷启动测试通过；
- [ ] 停止脚本可正常清理本项目会话；
- [ ] 已用 PX4 日志验证实际融合，而非只看 ROS 话题；
- [ ] 已解决真实位姿频率低于飞行门槛等未完成项后，才考虑装桨飞行。

## 11. 进一步阅读与许可证

- [完整项目与二次开发手册](AIRY_FASTLIO_MANUAL.md)
- [脚本、配置和状态参考](shfile/README.md)
- [Airy 驱动环境说明](environment/README_AIRY.md)
- [FAST-LIO2 论文](doc/Fast_LIO_2.pdf)
- [FAST_LIO 上游项目](https://github.com/hku-mars/FAST_LIO)

本项目主许可证见 [LICENSE](LICENSE)（GPL-2.0）。RoboSense 驱动及其他第三方组件
遵循各自许可证，详见相应源码目录。
