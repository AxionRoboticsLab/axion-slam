#include "axion_slam/map_io.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace
{

  std::string expand_home_path(const std::string & path)
  {
    if (path.empty() || path[0] != '~') {
      return path;
    }
    const char * home = std::getenv("HOME");
    if (!home) {
      home = std::getenv("USERPROFILE");
    }
    // Docker/systemd 里偶发无 HOME：勿把字面量 "~/..." 当路径（会写到奇怪相对目录）
    std::string home_str;
    if (!home) {
  #ifdef _WIN32
      home_str = "C:/axion/data";
  #else
      home_str = "/var/tmp/axion";
  #endif
    } else {
      home_str = home;
    }
    if (path.size() == 1) {
      return home_str;
    }
    if (path[1] == '/' || path[1] == '\\') {
      return home_str + path.substr(1);
    }
    return path;
  }

  std::string trim_copy(const std::string & s)
  {
    const auto start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) {
      return "";
    }
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
  }

  /** QoS that matches typical rosbridge_suite publishers (often BEST_EFFORT). */
  rclcpp::QoS bridge_sub_qos()
  {
    return rclcpp::SensorDataQoS().keep_last(20);
  }

  rclcpp::QoS bridge_pub_qos()
  {
    return rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  }

}  // namespace

class MapManagerNode : public rclcpp::Node
{
public:
  MapManagerNode()
  : Node("map_manager")
  {
    maps_dir_ = expand_home_path(
      declare_parameter<std::string>("maps_dir", "~/data/maps"));
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 2.0);
    pose_rate_hz_ = declare_parameter<double>("pose_rate_hz", 20.0);
    // 前端摇杆默认角速度很小（如 0.05 rad/s），mock 下放大便于肉眼看见
    teleop_scale_ = declare_parameter<double>("teleop_scale", 8.0);
    mock_width_ = declare_parameter<int>("mock_width", 200);
    mock_height_ = declare_parameter<int>("mock_height", 200);
    mock_resolution_ = declare_parameter<double>("mock_resolution", 0.05);

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).transient_local());
    // volatile：与 rosbridge 订阅兼容；勿用 transient_local，否则浏览器经常收不到状态
    state_pub_ = create_publisher<std_msgs::msg::String>("/map_state", bridge_pub_qos());
    // BEST_EFFORT：浏览器经 rosbridge 订阅时常用 sensor QoS，Reliable 会对不上
    // 地图列表改走 edge-agent REST，不再发布 /map_file_list
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/robot_pose", bridge_sub_qos());

    // BEST_EFFORT: browser → rosbridge often cannot match default RELIABLE subscriptions.
    cmd_sub_ = create_subscription<std_msgs::msg::String>(
      "/map_command", bridge_sub_qos(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "map_command: '%s'", msg->data.c_str());
        handle_command(msg->data);
      });

    auto on_cmd_vel = [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_cmd_ = *msg;
        last_cmd_time_ = now();
        has_cmd_ = true;
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "cmd_vel: lin=(%.2f,%.2f) ang.z=%.2f",
          msg->linear.x, msg->linear.y, msg->angular.z);
      };
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, bridge_sub_qos(), on_cmd_vel);
    // 部分 rosbridge 对 Twist 使用 Reliable，再挂一路避免收不到
    cmd_vel_sub_reliable_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, bridge_pub_qos(), on_cmd_vel);

    get_maps_srv_ = create_service<std_srvs::srv::Trigger>(
      "/get_map_files",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        const auto joined = join_map_names();
        RCLCPP_INFO(
          get_logger(), "get_map_files maps_dir=%s -> '%s'",
          maps_dir_.c_str(), joined.c_str());
        response->success = true;
        response->message = joined;
      });

    const auto map_period = std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(map_period),
      [this]() { on_timer(); });

    const auto pose_period = std::chrono::duration<double>(1.0 / std::max(1.0, pose_rate_hz_));
    pose_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(pose_period),
      [this]() { on_pose_timer(); });

    reset_pose();
    publish_state();
    RCLCPP_INFO(
      get_logger(),
      "map_manager ready (mock). maps_dir=%s state=%s",
      maps_dir_.c_str(), state_.c_str());
  }

private:
  void reset_pose()
  {
    pose_x_ = 0.0;
    pose_y_ = 0.0;
    pose_yaw_ = 0.0;
    last_cmd_ = geometry_msgs::msg::Twist();
    has_cmd_ = false;
  }

  void publish_state()
  {
    std_msgs::msg::String msg;
    msg.data = state_;
    state_pub_->publish(msg);
  }

  void set_state(const std::string & next)
  {
    if (state_ == next) {
      return;
    }
    RCLCPP_INFO(get_logger(), "state: %s -> %s", state_.c_str(), next.c_str());
    state_ = next;
    publish_state();
  }

  void handle_command(const std::string & raw)
  {
    const auto cmd = trim_copy(raw);
    if (cmd.empty()) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    // 前端/桥接偶发连发两条相同命令，短窗口内去重
    const auto now_time = now();
    if (cmd == last_cmd_raw_ && (now_time - last_cmd_handled_time_).seconds() < 0.3) {
      return;
    }
    last_cmd_raw_ = cmd;
    last_cmd_handled_time_ = now_time;

    if (cmd == "list") {
      // 列表已迁至 edge-agent GET /api/maps；保留命令仅打日志便于运维
      RCLCPP_INFO(
        get_logger(), "map_command list (deprecated topic) maps_dir=%s -> '%s'",
        maps_dir_.c_str(), join_map_names().c_str());
      return;
    }

    if (cmd == "start") {
      if (state_ != "idle") {
        RCLCPP_WARN(get_logger(), "start ignored in state=%s", state_.c_str());
        return;
      }
      start_mock_mapping();
      set_state("mapping");
      return;
    }

    if (cmd == "stop") {
      if (state_ != "mapping" && state_ != "navigation") {
        RCLCPP_WARN(get_logger(), "stop ignored in state=%s", state_.c_str());
        return;
      }
      set_state("terminating");
      stop_requested_ = true;
      return;
    }

    if (cmd.rfind("save ", 0) == 0) {
      const auto name = trim_copy(cmd.substr(5));
      if (!axion_slam::is_valid_map_name(name)) {
        RCLCPP_ERROR(get_logger(), "invalid map name: '%s'", name.c_str());
        return;
      }
      if (state_ != "mapping" || !has_map_) {
        RCLCPP_WARN(get_logger(), "save ignored (need mapping + map)");
        return;
      }
      std::string error;
      const auto paths = axion_slam::make_map_paths(maps_dir_, name);
      if (!axion_slam::save_occupancy_grid(current_map_, paths, error)) {
        RCLCPP_ERROR(get_logger(), "save failed: %s", error.c_str());
        return;
      }
      RCLCPP_INFO(
        get_logger(),
        "saved map '%s' -> %s (+ %s)  [maps_dir=%s]",
        name.c_str(), paths.yaml_path.c_str(), paths.pgm_path.c_str(), maps_dir_.c_str());
      // 协议：mapping → save → idle；前端靠 /map_state 退出建图 UI
      has_map_ = false;
      mapping_tick_ = 0;
      reset_pose();
      set_state("idle");
      return;
    }

    if (cmd.rfind("load ", 0) == 0) {
      const auto name = trim_copy(cmd.substr(5));
      if (!axion_slam::is_valid_map_name(name) && !axion_slam::is_legacy_map_name(name)) {
        RCLCPP_ERROR(get_logger(), "invalid map name: '%s'", name.c_str());
        return;
      }
      if (state_ != "idle") {
        RCLCPP_WARN(get_logger(), "load ignored in state=%s", state_.c_str());
        return;
      }
      nav_msgs::msg::OccupancyGrid loaded;
      std::string error;
      auto paths = axion_slam::make_map_paths(maps_dir_, name);
      // 兼容旧文件 {name}_2dmap.*
      if (!std::filesystem::exists(paths.yaml_path) || !std::filesystem::exists(paths.pgm_path)) {
        const auto legacy_base = std::filesystem::path(maps_dir_) / (name + "_2dmap");
        if (std::filesystem::exists(legacy_base.string() + ".yaml") &&
          std::filesystem::exists(legacy_base.string() + ".pgm"))
        {
          paths.yaml_path = legacy_base.string() + ".yaml";
          paths.pgm_path = legacy_base.string() + ".pgm";
        }
      }
      if (!axion_slam::load_occupancy_grid(paths, loaded, error)) {
        RCLCPP_ERROR(get_logger(), "load failed: %s", error.c_str());
        return;
      }
      current_map_ = loaded;
      current_map_.header.frame_id = "map";
      has_map_ = true;
      mapping_tick_ = 0;
      idle_map_pub_ticks_ = 0;
      {
        size_t n_free = 0, n_occ = 0, n_unk = 0;
        for (const auto cell : current_map_.data) {
          if (cell < 0) {
            ++n_unk;
          } else if (cell == 0) {
            ++n_free;
          } else {
            ++n_occ;
          }
        }
        RCLCPP_INFO(
          get_logger(),
          "loaded map '%s' %ux%u cells free=%zu occ=%zu unk=%zu -> publishing /map",
          name.c_str(), current_map_.info.width, current_map_.info.height,
          n_free, n_occ, n_unk);
      }
      publish_map_locked();
      return;
    }

    RCLCPP_WARN(get_logger(), "unknown map_command: '%s'", cmd.c_str());
  }

  void start_mock_mapping()
  {
    mapping_tick_ = 0;
    has_map_ = true;
    reset_pose();
    current_map_ = make_blank_mock_map();
    paint_mock_room(current_map_, /*reveal_ratio=*/1.0);
    publish_map_locked();
  }

  nav_msgs::msg::OccupancyGrid make_blank_mock_map() const
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.info.resolution = static_cast<float>(mock_resolution_);
    grid.info.width = static_cast<uint32_t>(mock_width_);
    grid.info.height = static_cast<uint32_t>(mock_height_);
    grid.info.origin.position.x = -0.5 * mock_width_ * mock_resolution_;
    grid.info.origin.position.y = -0.5 * mock_height_ * mock_resolution_;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(static_cast<size_t>(mock_width_ * mock_height_), static_cast<int8_t>(-1));
    return grid;
  }

  void paint_mock_room(nav_msgs::msg::OccupancyGrid & grid, double reveal_ratio) const
  {
    const int w = static_cast<int>(grid.info.width);
    const int h = static_cast<int>(grid.info.height);
    const int margin = std::max(4, std::min(w, h) / 10);
    const double r = std::clamp(reveal_ratio, 0.05, 1.0);
    const int revealed_w = static_cast<int>(std::lround((w - 2 * margin) * r));
    const int revealed_h = static_cast<int>(std::lround((h - 2 * margin) * r));

    auto idx = [w](int x, int y) {
        return static_cast<size_t>(y * w + x);
      };

    for (int y = margin; y < margin + revealed_h && y < h - margin; ++y) {
      for (int x = margin; x < margin + revealed_w && x < w - margin; ++x) {
        // 空闲区画浅色格线，前端能看出「栅格」而不只是一片白
        const bool on_grid = (x - margin) % 10 == 0 || (y - margin) % 10 == 0;
        grid.data[idx(x, y)] = static_cast<int8_t>(on_grid ? 18 : 0);
      }
    }

    auto maybe_wall = [&](int x, int y) {
        if (x < 0 || y < 0 || x >= w || y >= h) {
          return;
        }
        if (x > margin + revealed_w || y > margin + revealed_h) {
          return;
        }
        grid.data[idx(x, y)] = 100;
      };

    for (int x = margin; x <= margin + revealed_w && x < w - margin; ++x) {
      maybe_wall(x, margin);
      if (revealed_h >= (h - 2 * margin)) {
        maybe_wall(x, h - margin - 1);
      }
    }
    for (int y = margin; y <= margin + revealed_h && y < h - margin; ++y) {
      maybe_wall(margin, y);
      if (revealed_w >= (w - 2 * margin)) {
        maybe_wall(w - margin - 1, y);
      }
    }

    if (r > 0.55) {
      const int cx = w / 2;
      const int cy = h / 2;
      for (int y = cy - 3; y <= cy + 3; ++y) {
        for (int x = cx - 3; x <= cx + 3; ++x) {
          if (x >= 0 && y >= 0 && x < w && y < h) {
            grid.data[idx(x, y)] = 100;
          }
        }
      }
    }
  }

  std::string join_map_names() const
  {
    const auto names = axion_slam::list_map_names(maps_dir_);
    if (names.empty()) {
      return "";
    }
    std::string joined = names[0];
    for (size_t i = 1; i < names.size(); ++i) {
      joined += ",";
      joined += names[i];
    }
    return joined;
  }

  void publish_map_locked()
  {
    if (!has_map_) {
      return;
    }
    current_map_.header.stamp = now();
    map_pub_->publish(current_map_);
  }

  void publish_pose_locked()
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.pose.position.x = pose_x_;
    msg.pose.position.y = pose_y_;
    msg.pose.position.z = 0.0;
    msg.pose.orientation.z = std::sin(pose_yaw_ * 0.5);
    msg.pose.orientation.w = std::cos(pose_yaw_ * 0.5);
    pose_pub_->publish(msg);
  }

  void on_pose_timer()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // 仅建图时发布 /robot_pose；idle 载图后交给 axion-nav，避免双节点抢话题
    if (state_ != "mapping") {
      return;
    }
    const double dt = 1.0 / std::max(1.0, pose_rate_hz_);

    // Stop integrating if cmd_vel goes silent (joystick released).
    if (has_cmd_) {
      const double age = (now() - last_cmd_time_).seconds();
      if (age < 0.5) {
        const double scale = std::max(0.1, teleop_scale_);
        const double vx = last_cmd_.linear.x * scale;
        const double vy = last_cmd_.linear.y * scale;
        const double wz = last_cmd_.angular.z * scale;
        const double c = std::cos(pose_yaw_);
        const double s = std::sin(pose_yaw_);
        pose_x_ += (c * vx - s * vy) * dt;
        pose_y_ += (s * vx + c * vy) * dt;
        pose_yaw_ += wz * dt;

        // 限制在地图范围内，避免箭头跑出画板
        if (has_map_) {
          const double pad = mock_resolution_ * 3.0;
          const double min_x = current_map_.info.origin.position.x + pad;
          const double min_y = current_map_.info.origin.position.y + pad;
          const double max_x =
            min_x + current_map_.info.width * current_map_.info.resolution - 2 * pad;
          const double max_y =
            min_y + current_map_.info.height * current_map_.info.resolution - 2 * pad;
          pose_x_ = std::clamp(pose_x_, min_x, max_x);
          pose_y_ = std::clamp(pose_y_, min_y, max_y);
        }
      }
    }

    publish_pose_locked();
  }

  void on_timer()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // 不再每 tick 广播 idle，避免把前端刚切到的 mapping「闪回」成 idle

    if (state_ == "terminating" && stop_requested_) {
      has_map_ = false;
      mapping_tick_ = 0;
      stop_requested_ = false;
      reset_pose();
      set_state("idle");
      return;
    }

    if (state_ == "mapping" && has_map_) {
      ++mapping_tick_;
      // 已全图展示，保持完整房间（可选轻微噪声可后续加）
      paint_mock_room(current_map_, 1.0);
      publish_map_locked();
    } else if (has_map_ && state_ == "idle") {
      // 静态地图不要 2Hz 狂发，否则前端反复换纹理会青/白闪烁；低频保活给晚订阅者
      ++idle_map_pub_ticks_;
      if (idle_map_pub_ticks_ == 1 || idle_map_pub_ticks_ % 10 == 0) {
        publish_map_locked();
      }
    }
  }

  std::string maps_dir_;
  std::string map_topic_;
  std::string cmd_vel_topic_;
  double publish_rate_hz_{2.0};
  double pose_rate_hz_{20.0};
  double teleop_scale_{8.0};
  int mock_width_{200};
  int mock_height_{200};
  double mock_resolution_{0.05};

  std::string state_{"idle"};
  bool has_map_{false};
  bool stop_requested_{false};
  int mapping_tick_{0};
  int idle_map_pub_ticks_{0};
  nav_msgs::msg::OccupancyGrid current_map_;

  double pose_x_{0.0};
  double pose_y_{0.0};
  double pose_yaw_{0.0};
  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  bool has_cmd_{false};
  std::string last_cmd_raw_;
  rclcpp::Time last_cmd_handled_time_{0, 0, RCL_ROS_TIME};

  std::mutex mutex_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_reliable_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_maps_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr pose_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapManagerNode>());
  rclcpp::shutdown();
  return 0;
}
