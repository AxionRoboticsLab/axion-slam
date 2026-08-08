# axion-slam

Axion 机器人 **2D 建图（SLAM）** 与地图存取服务。基于 **ROS 2 Humble**，封装 [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) 等开源算法，对外提供稳定的地图命令协议，供 [axion-console](https://github.com/AxionRoboticsLab/axion-console) 2D 控制台使用。

> 组织：[AxionRoboticsLab](https://github.com/AxionRoboticsLab)  
> 仓库：<https://github.com/AxionRoboticsLab/axion-slam>

---

## 在整体架构中的位置

```text
[axion-console 浏览器]
        │  WebSocket ws://<host>:9090
        ▼
[rosbridge_server]          ← 官方包，不单独建仓；由本仓库 launch 拉起
        │  ROS 2 topics / services
        ▼
[axion-slam]                ← 本仓库：状态机 + 存/载图 + 调算法
        │
        ▼
[slam_toolbox / mock]       ← 算法或无传感器时的协议桩
        │
        ▼
~/data/maps/{name}_2dmap.*  ← 地图落在机器（或云开发机）本地
```

| 组件 | 职责 |
|------|------|
| **axion-slam** | 状态机、`/map_command`、地图文件、启停 SLAM |
| **rosbridge_suite** | 浏览器 ↔ ROS 2 网关（apt 安装即可，不必自建 GitHub 项目） |
| **axion-console** | 2D 控制台：开始建图 / 保存 / 载入 / 停止，渲染 `/map` |
| **axion-edge-agent** | 本期不参与建图数据面 |

---

## 状态机

| 状态 | 说明 | 一期 |
|------|------|------|
| `idle` | 可开始建图或载图 | ✅ |
| `mapping` | SLAM 运行中，持续发布 `/map` | ✅ |
| `terminating` | 正在停止 SLAM | ✅ |
| `navigation` | 导航中 | 预留 |
| `charging` | 充电中 | 预留 |

典型流转：`idle → mapping → (save) → idle`；`stop → terminating → idle`。

---

## 对外协议（兼容现有 2D 控制台）

| 接口 | 类型 | 说明 |
|------|------|------|
| `/map_command` | `std_msgs/String` | `start` · `save <name>` · `load <name>` · `stop` · `list` |
| `/map_state` | `std_msgs/String` | 当前状态字符串 |
| `/map` | `nav_msgs/OccupancyGrid` | 栅格地图（建图实时 / 载图静态） |
| `/map_file_list` | `std_msgs/String` | `list` 命令回复：逻辑名逗号分隔（推荐前端用此路径） |
| `/get_map_files` | `std_srvs/srv/Trigger` | `message` 为地图逻辑名列表（逗号分隔；rosbridge 上偶发超时） |

### 地图命名与文件

- 逻辑名：仅英文，`^[A-Za-z][A-Za-z0-9_]{0,31}$`
- 目录：运行 `map_manager` 进程的机器上 `~/data/maps/`（展开后一般为 `/home/<user>/data/maps`；可用参数 `maps_dir` 覆盖）
- 文件：`{name}_2dmap.pgm`、`{name}_2dmap.yaml`
- **不是**写在浏览器/前端仓库里；Docker 部署时在**容器内**该路径（需挂载卷才能在宿主机看到）

示例：`save office_01` → `~/data/maps/office_01_2dmap.pgm` + `.yaml`

自检：
```bash
ls ~/data/maps/
ros2 service call /get_map_files std_srvs/srv/Trigger {}
```

---

## 一期范围

**做：** 建图 + 存图 + 载图（最小闭环）  
**不做：** 禁行区/路点/充电点编辑、导航、云端地图库（预留目录与命名即可）

### 无真机 / 无传感器时（云服务器）

| 阶段 | 状态 | 说明 |
|------|------|------|
| **① mock 协议+存盘** | ✅ 已实现 | `map_manager_node`：假 OccupancyGrid + save/load |
| **② bag 回放** | 可选 | `/scan`+`/odom` bag + slam_toolbox |
| **③ Gazebo headless** | 可选 | 无头仿真 |

---

## 仓库结构

本仓库是 **colcon 工作空间**，包在 `src/axion_slam`：

```text
axion-slam/
├── README.md
└── src/axion_slam/
    ├── package.xml
    ├── CMakeLists.txt
    ├── include/axion_slam/map_io.hpp
    ├── src/map_io.cpp
    ├── src/map_manager_node.cpp
    ├── launch/mock.launch.py      # 仅 map_manager
    ├── launch/bringup.launch.py  # rosbridge + map_manager
    └── config/map_manager.yaml
```

---

## 快速开始

### 依赖（Humble 主机）

```bash
sudo apt update
sudo apt install -y \
  ros-humble-rosbridge-suite \
  ros-humble-slam-toolbox \
  ros-humble-nav-msgs \
  ros-humble-std-srvs
```

### 编译

```bash
# 若已 clone 到 ~/axion-slam
cd ~/axion-slam
source /opt/ros/humble/setup.bash
colcon build --packages-select axion_slam
source install/setup.bash
```

### 启动

```bash
# 推荐：mock + rosbridge（给 console 联调）
ros2 launch axion_slam bringup.launch.py

# 仅本机反代场景，rosbridge 只绑 127.0.0.1：
ros2 launch axion_slam bringup.launch.py rosbridge_address:=127.0.0.1

# 只要节点、自己另起 rosbridge：
ros2 launch axion_slam mock.launch.py
```

launch 已默认加载 `config/fastdds_no_shm.xml`（禁用 FastDDS 共享内存，仅 UDP），避免云主机上常见的：

```text
[RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_portXXXX: open_and_lock_file failed
```

若仍看到该日志（未走本仓库 launch、或旧 install）：

```bash
# 临时：清残留锁 + 强制 UDP 配置
rm -f /dev/shm/fastrtps*
export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix axion_slam)/share/axion_slam/config/fastdds_no_shm.xml
ros2 launch axion_slam bringup.launch.py
```

### 命令行冒烟（无需浏览器）

```bash
# 另开终端
source ~/axion-slam/install/setup.bash

ros2 topic echo /map_state --once
ros2 topic pub --once /map_command std_msgs/msg/String "{data: 'start'}"
ros2 topic echo /map --once   # 应看到 OccupancyGrid

ros2 topic pub --once /map_command std_msgs/msg/String "{data: 'save office_01'}"
ros2 service call /get_map_files std_srvs/srv/Trigger {}
ls ~/data/maps/

ros2 topic pub --once /map_command std_msgs/msg/String "{data: 'stop'}"
```

### 接 axion-console

1. 设置 → 机器人 IP 填云主机公网 IP（console 会连 `ws://IP:9090`）
2. 打开 **2D 控制台** `/robot/amr`
3. 点「开始建图」→ 应看到逐渐展开的房间栅格 →「保存」英文名 →「载入」列表可见

安全组需放行 **9090/TCP**（或 SSH 隧道：`ssh -L 9090:127.0.0.1:9090 user@host`，console 填 `127.0.0.1`）。

---

## 相关仓库

| 仓库 | 说明 |
|------|------|
| [axion-console](https://github.com/AxionRoboticsLab/axion-console) | Web 控制台（2D 建图 UI） |
| [axion-edge-agent](https://github.com/AxionRoboticsLab/axion-edge-agent) | 机上 Agent（认证/用户等；建图数据面本期不经过） |

---

## License

见 [LICENSE](./LICENSE)。
