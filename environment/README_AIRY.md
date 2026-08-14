# RoboSense Airy 环境与接线检查

本目录使用 RoboSense 官方 `rslidar_sdk` v1.5.19 及其 `rs_driver` 子模块。
Airy 配置在 `rslidar_sdk/config/config_airy.yaml`，默认接收端口为：

| 数据 | UDP 端口 | ROS 话题 |
| --- | ---: | --- |
| MSOP 点云 | 6699 | `/rslidar_points` |
| DIFOP 配置 | 7788 | 不单独发布 |
| 内置 IMU | 6688 | `/rslidar_imu_data` |

## 上板前的网络配置

1. 将开发板有线网口设置为与雷达同一网段的静态 IP。
2. 在 Airy 的配置工具或 Web 页面中，把 MSOP、DIFOP、IMU 的目标 IP
   设置为开发板有线网口 IP，目标端口设置为上表数值。
3. 确认防火墙允许这三个 UDP 端口；配置文件中的 `host_address` 和
   `group_address` 为 `0.0.0.0`，表示驱动在本机接口上接收单播数据。
4. 若雷达已配置为其他端口，只修改 `config_airy.yaml` 中对应端口即可。

可先用以下命令确认数据包确实到达开发板：

```bash
sudo tcpdump -ni any 'udp port 6699 or udp port 7788 or udp port 6688'
```

`wait_for_difop: true` 会让驱动等待 DIFOP 后再输出完整点云。如果只有
MSOP 而没有 DIFOP，先排查雷达目标 IP、7788 端口和网卡设置，不建议通过
关闭该选项掩盖网络问题。

## 编译与启动

```bash
bash shfile/install_fastlio2_Airy.sh --jobs 2
source shfile/setup_fastlio2_Airy.bash
roslaunch fast_lio mapping_airy.launch
```

驱动固定编译为 `XYZIRT`，其 PointCloud2 必须包含 `x/y/z/intensity/ring/
timestamp` 字段。FAST-LIO Airy 预处理器会把绝对秒时间戳转换成每帧内的
毫秒偏移，用于运动畸变补偿。不要改回默认的 `XYZI`。

连接雷达后运行：

```bash
bash shfile/check_airy_topics.sh
```

## 外参与飞行前检查

`config/airy.yaml` 内置的是 Airy 雷达坐标系到内置 IMU 坐标系的标称初值，
并开启了 FAST-LIO 在线外参估计。不同设备的 DIFOP 中保存有本机外参，正式
上机前必须用设备值替换 `extrinsic_T` 和 `extrinsic_R`，并静置初始化后在
手持/地面测试中检查 `/cloud_registered` 是否重影、倾斜或漂移。

还应确认：

- `/rslidar_points` 和 `/rslidar_imu_data` 持续发布且时间轴一致；
- 雷达安装刚性可靠，飞控/机体坐标变换另行标定；
- 点云不因网口丢包出现周期性缺口；
- 在飞行前完成录包回放和低动态地面测试。
