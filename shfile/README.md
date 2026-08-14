# FAST-LIO2 一键部署脚本

本目录同时保留 Livox 与 RoboSense Airy 两套部署脚本。脚本面向 Ubuntu
20.04 + ROS Noetic，可在 `amd64`、`i386`、`arm64` 和 `armhf` 目标机原生
编译；RK3588 的 64 位 Ubuntu 会识别为 `arm64`。

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

脚本会在开始时获取 sudo 权限，安装系统依赖，然后把官方驱动编译为
`XYZIRT` 点型并开启 Airy IMU 解析，最后编译不依赖 Livox 消息包的
FAST-LIO2。每种 CPU 架构使用独立构建目录，不会混用 x86 与 ARM 产物。

RK3588 内存较紧时可限制并行度：

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

## 常用选项

```bash
# APT 索引已更新时
bash shfile/install_fastlio2_Airy.sh --skip-apt-update

# 安装 RViz 等完整 ROS 桌面组件
bash shfile/install_fastlio2_Airy.sh --desktop-full
```

脚本默认要求 Ubuntu 20.04。`--allow-unsupported-os` 只用于已经自行配置好
ROS Noetic 的非标准系统，不保证其 APT 软件源存在所需二进制包。
