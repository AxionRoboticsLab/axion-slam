#include "axion_slam/map_io.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>

namespace fs = std::filesystem;

namespace axion_slam
{

  namespace
  {

    std::string trim(const std::string & s)
    {
      const auto start = s.find_first_not_of(" \t\r\n");
      if (start == std::string::npos) {
        return "";
      }
      const auto end = s.find_last_not_of(" \t\r\n");
      return s.substr(start, end - start + 1);
    }

    uint8_t occupancy_to_pgm(int8_t cell)
    {
      if (cell < 0) {
        return 205;  // unknown (nav2 convention-ish)
      }
      if (cell == 0) {
        return 254;  // free
      }
      if (cell >= 100) {
        return 0;  // occupied
      }
      // interpolate
      const double t = static_cast<double>(cell) / 100.0;
      return static_cast<uint8_t>(std::lround(254.0 * (1.0 - t)));
    }

    int8_t pgm_to_occupancy(uint8_t pixel, double occupied_thresh, double free_thresh, bool negate)
    {
      // 本仓库 occupancy_to_pgm 写入：0→占用, 205→未知, 254→空闲
      // 先按离散值识别，避免阈值把 254 误判成占用
      if (!negate) {
        if (pixel >= 250) {
          return 0;  // free (254)
        }
        if (pixel <= 25) {
          return 100;  // occupied (0)
        }
        if (pixel >= 180 && pixel <= 230) {
          return -1;  // unknown (205)
        }
      }

      // 与 ROS map_server / nav2 一致：negate=0 时黑=占用、白=空闲
      const double occ = negate
        ? (static_cast<double>(pixel) / 255.0)
        : (static_cast<double>(255 - pixel) / 255.0);
      if (occ > occupied_thresh) {
        return 100;
      }
      if (occ < free_thresh) {
        return 0;
      }
      return -1;
    }

  }  // namespace

  MapPaths make_map_paths(const std::string & maps_dir, const std::string & logical_name)
  {
    MapPaths paths;
    paths.logical_name = logical_name;
    const fs::path base = fs::path(maps_dir) / logical_name;
    paths.yaml_path = (base.string() + ".yaml");
    paths.pgm_path = (base.string() + ".pgm");
    return paths;
  }

  MapPaths make_legacy_map_paths(const std::string & maps_dir, const std::string & logical_name)
  {
    MapPaths paths;
    paths.logical_name = logical_name;
    const fs::path base = fs::path(maps_dir) / (logical_name + "_2dmap");
    paths.yaml_path = (base.string() + ".yaml");
    paths.pgm_path = (base.string() + ".pgm");
    return paths;
  }

  bool is_valid_map_name(const std::string & name)
  {
    // 与 axion-edge-agent / console identity name 一致：3–20，字母开头
    static const std::regex k_re("^[A-Za-z][A-Za-z0-9_]{2,19}$");
    return std::regex_match(name, k_re);
  }

  bool is_legacy_map_name(const std::string & name)
  {
    // 载入兼容旧名（如 v1）；新建/保存仍走 is_valid_map_name
    static const std::regex k_re("^[A-Za-z][A-Za-z0-9_]{0,31}$");
    return std::regex_match(name, k_re);
  }

  std::vector<std::string> list_map_names(const std::string & maps_dir)
  {
    std::vector<std::string> names;
    const fs::path dir(maps_dir);
    if (!fs::exists(dir) || !fs::is_directory(dir)) {
      return names;
    }

    for (const auto & entry : fs::directory_iterator(dir)) {
      if (!entry.is_regular_file()) {
        continue;
      }
      const auto filename = entry.path().filename().string();
      std::string logical;
      const std::string legacy_suffix = "_2dmap.yaml";
      if (filename.size() > legacy_suffix.size() &&
        filename.compare(
          filename.size() - legacy_suffix.size(), legacy_suffix.size(), legacy_suffix) == 0)
      {
        logical = filename.substr(0, filename.size() - legacy_suffix.size());
      } else if (filename.size() > 5 &&
        filename.compare(filename.size() - 5, 5, ".yaml") == 0)
      {
        logical = filename.substr(0, filename.size() - 5);
        if (logical.size() > 6 &&
          logical.compare(logical.size() - 6, 6, "_2dmap") == 0)
        {
          continue;  // already handled as legacy
        }
      } else {
        continue;
      }
      if (!is_valid_map_name(logical) && !is_legacy_map_name(logical)) {
        continue;
      }
      const auto paths = make_map_paths(maps_dir, logical);
      const auto legacy = make_legacy_map_paths(maps_dir, logical);
      if (fs::exists(paths.pgm_path) || fs::exists(legacy.pgm_path)) {
        if (std::find(names.begin(), names.end(), logical) == names.end()) {
          names.push_back(logical);
        }
      }
    }
    std::sort(names.begin(), names.end());
    return names;
  }

  bool save_occupancy_grid(
    const nav_msgs::msg::OccupancyGrid & grid,
    const MapPaths & paths,
    std::string & error)
  {
    try {
      fs::create_directories(fs::path(paths.yaml_path).parent_path());
    } catch (const std::exception & e) {
      error = std::string("create maps dir failed: ") + e.what();
      return false;
    }

    const auto width = static_cast<int>(grid.info.width);
    const auto height = static_cast<int>(grid.info.height);
    if (width <= 0 || height <= 0 ||
      static_cast<size_t>(width * height) != grid.data.size())
    {
      error = "invalid occupancy grid dimensions";
      return false;
    }

    {
      std::ofstream pgm(paths.pgm_path, std::ios::binary);
      if (!pgm) {
        error = "cannot open pgm for write: " + paths.pgm_path;
        return false;
      }
      pgm << "P5\n" << width << " " << height << "\n255\n";
      // OccupancyGrid row 0 is at origin (bottom); PGM row 0 is top → flip Y
      for (int y = height - 1; y >= 0; --y) {
        for (int x = 0; x < width; ++x) {
          const auto cell = grid.data[static_cast<size_t>(y * width + x)];
          const uint8_t px = occupancy_to_pgm(cell);
          pgm.write(reinterpret_cast<const char *>(&px), 1);
        }
      }
    }

    {
      std::ofstream yaml(paths.yaml_path);
      if (!yaml) {
        error = "cannot open yaml for write: " + paths.yaml_path;
        return false;
      }
      const auto image_name = fs::path(paths.pgm_path).filename().string();
      yaml << "image: " << image_name << "\n";
      yaml << "mode: trinary\n";
      yaml << "resolution: " << grid.info.resolution << "\n";
      yaml << "origin: ["
          << grid.info.origin.position.x << ", "
          << grid.info.origin.position.y << ", "
          << 0.0 << "]\n";
      yaml << "negate: 0\n";
      yaml << "occupied_thresh: 0.65\n";
      yaml << "free_thresh: 0.25\n";
    }

    return true;
  }

  bool load_occupancy_grid(
    const MapPaths & paths,
    nav_msgs::msg::OccupancyGrid & grid,
    std::string & error)
  {
    if (!fs::exists(paths.yaml_path) || !fs::exists(paths.pgm_path)) {
      error = "map files not found for " + paths.logical_name;
      return false;
    }

    std::string image_name;
    double resolution = 0.05;
    double origin_x = 0.0;
    double origin_y = 0.0;
    double occupied_thresh = 0.65;
    double free_thresh = 0.25;
    bool negate = false;

    {
      std::ifstream yaml(paths.yaml_path);
      if (!yaml) {
        error = "cannot open yaml: " + paths.yaml_path;
        return false;
      }
      std::string line;
      while (std::getline(yaml, line)) {
        const auto colon = line.find(':');
        if (colon == std::string::npos) {
          continue;
        }
        const auto key = trim(line.substr(0, colon));
        auto value = trim(line.substr(colon + 1));
        if (key == "image") {
          image_name = value;
        } else if (key == "resolution") {
          resolution = std::stod(value);
        } else if (key == "negate") {
          negate = (std::stoi(value) != 0);
        } else if (key == "occupied_thresh") {
          occupied_thresh = std::stod(value);
        } else if (key == "free_thresh") {
          free_thresh = std::stod(value);
        } else if (key == "origin") {
          // [x, y, yaw]
          value.erase(std::remove_if(value.begin(), value.end(),
              [](unsigned char c) { return c == '[' || c == ']'; }), value.end());
          std::stringstream ss(value);
          std::string part;
          if (std::getline(ss, part, ',')) {
            origin_x = std::stod(trim(part));
          }
          if (std::getline(ss, part, ',')) {
            origin_y = std::stod(trim(part));
          }
        }
      }
    }

    fs::path pgm_path = paths.pgm_path;
    if (!image_name.empty()) {
      const fs::path candidate = fs::path(paths.yaml_path).parent_path() / image_name;
      if (fs::exists(candidate)) {
        pgm_path = candidate;
      }
    }

    std::ifstream pgm(pgm_path, std::ios::binary);
    if (!pgm) {
      error = "cannot open pgm: " + pgm_path.string();
      return false;
    }

    std::string magic;
    pgm >> magic;
    if (magic != "P5" && magic != "P2") {
      error = "unsupported pgm magic: " + magic;
      return false;
    }

    auto skip_comments = [&pgm]() {
        while (pgm && (pgm.peek() == '#' || std::isspace(pgm.peek()))) {
          if (pgm.peek() == '#') {
            std::string comment;
            std::getline(pgm, comment);
          } else {
            pgm.get();
          }
        }
      };

    int width = 0;
    int height = 0;
    int maxval = 0;
    skip_comments();
    pgm >> width;
    skip_comments();
    pgm >> height;
    skip_comments();
    pgm >> maxval;
    pgm.get();  // single whitespace after maxval

    if (width <= 0 || height <= 0 || maxval <= 0 || maxval > 255) {
      error = "invalid pgm header";
      return false;
    }

    std::vector<uint8_t> pixels(static_cast<size_t>(width * height));
    if (magic == "P5") {
      pgm.read(reinterpret_cast<char *>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
      if (static_cast<size_t>(pgm.gcount()) != pixels.size()) {
        error = "pgm truncated";
        return false;
      }
    } else {
      for (auto & px : pixels) {
        int v = 0;
        pgm >> v;
        px = static_cast<uint8_t>(v);
      }
    }

    grid = nav_msgs::msg::OccupancyGrid();
    grid.info.resolution = static_cast<float>(resolution);
    grid.info.width = static_cast<uint32_t>(width);
    grid.info.height = static_cast<uint32_t>(height);
    grid.info.origin.position.x = origin_x;
    grid.info.origin.position.y = origin_y;
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(pixels.size());

    // Flip Y: PGM top row → OccupancyGrid last row
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const auto src = pixels[static_cast<size_t>(y * width + x)];
        const int dst_y = height - 1 - y;
        grid.data[static_cast<size_t>(dst_y * width + x)] =
          pgm_to_occupancy(src, occupied_thresh, free_thresh, negate);
      }
    }

    return true;
  }

}  // namespace axion_slam
