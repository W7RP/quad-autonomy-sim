// Offline replay: run EstimatorPipeline (the same code eskf_node runs live)
// over recorded sensor data, for tuning and regression testing without the
// simulator. ROS-free.
//
//   eskf_replay --in <logdir>/replay --params config/eskf.yaml --out est_replay.csv
//               [--set name=value ...]
//
// Inputs are the normalised CSVs from scripts/prepare_replay.py. Events are
// merged in PX4 publication order (t_pub_us), which approximates arrival order
// on the live system. Output is a headed CSV that scripts/eval_estimation.py
// accepts via --est.
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "quad_estimation/params.hpp"
#include "quad_estimation/pipeline.hpp"

using namespace quad_estimation;

namespace
{

using Row = std::unordered_map<std::string, double>;

bool getline_trimmed(std::istream & in, std::string & line)
{
  if (!std::getline(in, line)) {
    return false;
  }
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();  // tolerate CRLF files
  }
  return true;
}

std::vector<Row> read_csv(const std::string & path)
{
  std::ifstream f(path);
  if (!f) {
    std::cerr << "cannot open " << path << "\n";
    std::exit(2);
  }
  std::string line;
  getline_trimmed(f, line);
  std::vector<std::string> header;
  {
    std::stringstream ss(line);
    std::string h;
    while (std::getline(ss, h, ',')) {
      header.push_back(h);
    }
  }
  std::vector<Row> rows;
  while (getline_trimmed(f, line)) {
    std::stringstream ss(line);
    std::string cell;
    Row r;
    for (const auto & h : header) {
      if (!std::getline(ss, cell, ',')) {
        break;
      }
      r[h] = std::strtod(cell.c_str(), nullptr);
    }
    if (r.size() == header.size()) {
      rows.push_back(std::move(r));
    }
  }
  return rows;
}

// Minimal reader for the node's flat YAML: "  key: value  # comment" lines.
// Unknown keys (ROS-only ones like px4_namespace) are ignored.
void load_yaml(const std::string & path, PipelineConfig & cfg)
{
  std::ifstream f(path);
  if (!f) {
    std::cerr << "cannot open " << path << "\n";
    std::exit(2);
  }
  std::string line;
  while (std::getline(f, line)) {
    line = line.substr(0, line.find('#'));
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    std::string key = line.substr(0, colon);
    std::string val = line.substr(colon + 1);
    key.erase(0, key.find_first_not_of(" \t"));
    key.erase(key.find_last_not_of(" \t") + 1);
    val.erase(0, val.find_first_not_of(" \t"));
    if (val.empty()) {
      continue;  // a mapping header like "eskf:" or "ros__parameters:"
    }
    char * end = nullptr;
    const double v = std::strtod(val.c_str(), &end);
    if (end != val.c_str()) {
      (void)set_pipeline_param(cfg, key, v);
    }
  }
}

struct Event
{
  std::int64_t t_pub;
  int kind;  // 0 imu, 1 range, 2 flow, 3 mag (tie-break order)
  std::size_t idx;
};

}  // namespace

int main(int argc, char ** argv)
{
  std::string in_dir, params, out = "est_replay.csv";
  std::vector<std::string> overrides;
  for (int i = 1; i < argc; ++i) {
    const std::string_view a = argv[i];
    auto next = [&]() -> std::string {
        if (i + 1 >= argc) {
          std::cerr << "missing value for " << a << "\n";
          std::exit(2);
        }
        return argv[++i];
      };
    if (a == "--in") {
      in_dir = next();
    } else if (a == "--params") {
      params = next();
    } else if (a == "--out") {
      out = next();
    } else if (a == "--set") {
      overrides.push_back(next());
    } else {
      std::cerr << "usage: eskf_replay --in DIR [--params YAML] [--out CSV] [--set k=v]...\n";
      return 2;
    }
  }
  if (in_dir.empty()) {
    std::cerr << "--in is required\n";
    return 2;
  }

  PipelineConfig cfg;
  if (!params.empty()) {
    load_yaml(params, cfg);
  }
  for (const auto & o : overrides) {
    const auto eq = o.find('=');
    if (eq == std::string::npos ||
      !set_pipeline_param(cfg, o.substr(0, eq), std::strtod(o.c_str() + eq + 1, nullptr)))
    {
      std::cerr << "bad --set " << o << " (unknown parameter?)\n";
      return 2;
    }
  }

  const auto imu = read_csv(in_dir + "/imu.csv");
  const auto range = read_csv(in_dir + "/range.csv");
  const auto flow = read_csv(in_dir + "/flow.csv");
  const auto mag = read_csv(in_dir + "/mag.csv");

  std::vector<Event> ev;
  ev.reserve(imu.size() + range.size() + flow.size() + mag.size());
  auto add = [&ev](const std::vector<Row> & rows, int kind) {
      for (std::size_t i = 0; i < rows.size(); ++i) {
        ev.push_back({static_cast<std::int64_t>(rows[i].at("t_pub_us")), kind, i});
      }
    };
  add(imu, 0);
  add(range, 1);
  add(flow, 2);
  add(mag, 3);
  std::stable_sort(ev.begin(), ev.end(), [](const Event & a, const Event & b) {
      return a.t_pub != b.t_pub ? a.t_pub < b.t_pub : a.kind < b.kind;
    });

  EstimatorPipeline pipe(cfg);
  std::ofstream o(out);
  o << "t_us,px,py,pz,qw,qx,qy,qz,vx,vy,vz,var_px,var_py,var_pz,var_vx,var_vy,var_vz,"
    "var_ax,var_ay,var_az,bgx,bgy,bgz,bax,bay,baz\n";
  o.precision(9);
  for (const Event & e : ev) {
    switch (e.kind) {
      case 0: {
          const Row & r = imu[e.idx];
          const ImuInput in{static_cast<std::int64_t>(r.at("t_us")),
            static_cast<std::int64_t>(r.at("dt_us")),
            Vec3(r.at("gx"), r.at("gy"), r.at("gz")), Vec3(r.at("ax"), r.at("ay"), r.at("az"))};
          if (pipe.on_imu(in)) {
            const NominalState & x = pipe.filter().state();
            const auto P = pipe.filter().covariance().diagonal();
            o << pipe.time_us() << ',' << x.p.x() << ',' << x.p.y() << ',' << x.p.z() << ','
              << x.q.w() << ',' << x.q.x() << ',' << x.q.y() << ',' << x.q.z() << ','
              << x.v.x() << ',' << x.v.y() << ',' << x.v.z();
            for (int i = 0; i < 9; ++i) {
              o << ',' << P[kP + i];  // pos, vel, att variances are contiguous
            }
            o << ',' << x.bg.x() << ',' << x.bg.y() << ',' << x.bg.z() << ','
              << x.ba.x() << ',' << x.ba.y() << ',' << x.ba.z() << '\n';
          }
          break;
        }
      case 1: {
          const Row & r = range[e.idx];
          pipe.on_range({static_cast<std::int64_t>(r.at("t_us")), r.at("range")});
          break;
        }
      case 2: {
          const Row & r = flow[e.idx];
          pipe.on_flow({static_cast<std::int64_t>(r.at("t_us")),
              static_cast<std::int64_t>(r.at("window_us")), Vec2(r.at("fx"), r.at("fy")),
              static_cast<int>(r.at("quality"))});
          break;
        }
      default: {
          const Row & r = mag[e.idx];
          pipe.on_mag({static_cast<std::int64_t>(r.at("t_us")),
              Vec3(r.at("mx"), r.at("my"), r.at("mz"))});
          break;
        }
    }
  }

  const PipelineCounters & c = pipe.counters();
  std::printf("replayed %zu events -> %s\n", ev.size(), out.c_str());
  std::printf("  fused  range %lu flow %lu heading %lu | rejected %lu %lu %lu | skipped %lu %lu %lu\n",
    c.range.fused.load(), c.flow.fused.load(), c.heading.fused.load(),
    c.range.rejected.load(), c.flow.rejected.load(), c.heading.rejected.load(),
    c.range.skipped.load(), c.flow.skipped.load(), c.heading.skipped.load());
  std::printf("  imu gaps %lu, clock steps %lu, suspect stamps %lu, off-timeline measurements %lu, "
    "initialisations %lu\n", c.imu_gaps.load(), c.clock_steps.load(), c.suspect_stamps.load(),
    c.off_timeline_measurements.load(), c.initializations.load());
  const auto & x = pipe.filter().state();
  std::printf("  final gyro bias %.5f %.5f %.5f, accel bias %.4f %.4f %.4f\n",
    x.bg.x(), x.bg.y(), x.bg.z(), x.ba.x(), x.ba.y(), x.ba.z());
  return 0;
}
