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
| `/map_command` | `std_msgs/String` | `start` · `save <name>` · `load <name>` · `stop` |
| `/map_state` | `std_msgs/String` | 当前状态字符串 |
| `/map` | `nav_msgs/OccupancyGrid` | 栅格地图（建图实时 / 载图静态） |
| `/get_map_files` | Service | 返回地图逻辑名列表（逗号分隔） |

### 地图命名与文件

- 逻辑名：仅英文，建议 `^[A-Za-z][A-Za-z0-9_]{0,31}$`
- 目录：`~/data/maps/`
- 文件：`{name}_2dmap.pgm`、`{name}_2dmap.yaml`（可选序列化文件同前缀）

示例：`save office_01` → `~/data/maps/office_01_2dmap.pgm` + `.yaml`

---

## 一期范围

**做：** 建图 + 存图 + 载图（最小闭环）  
**不做：** 禁行区/路点/充电点编辑、导航、云端地图库（预留目录与命名即可）

### 无真机 / 无传感器时（云服务器）

推荐分两级，不必一上来上完整仿真：

1. **协议 + 存盘闭环（优先）**  
   mock 节点响应 `/map_command`，发布简易 `/map`，把 pgm/yaml 写到 `~/data/maps/`。  
   用 axion-console 连 rosbridge，即可看到栅格、验证保存/列表/载入。

2. **可选：bag 回放**  
   有 `/scan` + `/odom`（及 TF）的 bag 时，`ros2 bag play` + slam_toolbox 做近似真建图。

3. **可选：Gazebo headless**  
   无 GUI 云主机可用无头仿真；资源占用较大，适合第二步再上。

前端 **可以** 展示效果：console 只订阅 OccupancyGrid，不关心地图来自真机、仿真、bag 还是 mock。

---

## 技术栈

- ROS 2 **Humble**
- 核心节点：建议 **C++（rclcpp）**；launch / 工具脚本：**Python**
- SLAM：优先 **slam_toolbox**
- 传输：与 console 之间经 **rosbridge_server**

---

## 快速开始（草稿）

> 包结构落地后补全具体包名与 launch 文件名。

```bash
# 依赖示例（主机已装 Humble）
sudo apt install ros-humble-rosbridge-suite ros-humble-slam-toolbox

# 工作空间编译（示意）
cd ~/ws && colcon build --packages-select axion_slam
source install/setup.bash

# 启动（将包含 rosbridge + axion-slam，端口 9090）
ros2 launch axion_slam bringup.launch.py

# 浏览器打开 axion-console，连接 ws://<云服务器公网IP>:9090
```

安全组需放行 **9090/TCP**（仅内网或加鉴权，勿裸奔公网）。

---

## 相关仓库

| 仓库 | 说明 |
|------|------|
| [axion-console](https://github.com/AxionRoboticsLab/axion-console) | Web 控制台（2D 建图 UI） |
| [axion-edge-agent](https://github.com/AxionRoboticsLab/axion-edge-agent) | 机上 Agent（认证/用户等；建图数据面本期不经过） |

---

## License

见 [LICENSE](./LICENSE)。
