#include "quad_estimation/params.hpp"

#include <array>
#include <cmath>
#include <numbers>

namespace quad_estimation
{
namespace
{
constexpr double kDeg = std::numbers::pi / 180.0;

#define QE_PARAM(NAME, FIELD) \
  ParamSpec{NAME, false, \
    [](const PipelineConfig & c) {return static_cast<double>(c.FIELD);}, \
    [](PipelineConfig & c, double v) {c.FIELD = v;}}

const std::array<ParamSpec, 17> kParams{{
  QE_PARAM("gravity", eskf.gravity),
  QE_PARAM("gyro_noise", eskf.gyro_noise),
  QE_PARAM("accel_noise", eskf.accel_noise),
  QE_PARAM("gyro_bias_walk", eskf.gyro_bias_walk),
  QE_PARAM("accel_bias_walk", eskf.accel_bias_walk),
  QE_PARAM("range_noise", eskf.range_noise),
  QE_PARAM("flow_noise_best", eskf.flow_noise_best),
  QE_PARAM("flow_noise_worst", eskf.flow_noise_worst),
  QE_PARAM("heading_noise", eskf.heading_noise),
  QE_PARAM("gate_1dof", eskf.gate_1dof),
  QE_PARAM("gate_2dof", eskf.gate_2dof),
  QE_PARAM("min_hagl", eskf.min_hagl),
  ParamSpec{"mag_declination_deg", false,
    [](const PipelineConfig & c) {return c.eskf.mag_declination / kDeg;},
    [](PipelineConfig & c, double v) {c.eskf.mag_declination = v * kDeg;}},
  ParamSpec{"max_tilt_deg", false,
    [](const PipelineConfig & c) {return std::acos(c.eskf.max_tilt_cos) / kDeg;},
    [](PipelineConfig & c, double v) {c.eskf.max_tilt_cos = std::cos(v * kDeg);}},
  ParamSpec{"max_measurement_delay_s", false,
    [](const PipelineConfig & c) {return static_cast<double>(c.max_delay_us) * 1e-6;},
    [](PipelineConfig & c, double v) {c.max_delay_us = static_cast<std::int64_t>(v * 1e6);}},
  ParamSpec{"flow_min_quality", true,
    [](const PipelineConfig & c) {return static_cast<double>(c.flow_min_quality);},
    [](PipelineConfig & c, double v) {c.flow_min_quality = static_cast<int>(v);}},
  ParamSpec{"alignment_samples", true,
    [](const PipelineConfig & c) {return static_cast<double>(c.alignment.samples);},
    [](PipelineConfig & c, double v) {c.alignment.samples = static_cast<int>(v);}},
}};
#undef QE_PARAM
}  // namespace

std::span<const ParamSpec> pipeline_params() noexcept {return kParams;}

bool set_pipeline_param(PipelineConfig & cfg, std::string_view name, double value) noexcept
{
  for (const auto & p : kParams) {
    if (name == p.name) {
      p.set(cfg, value);
      if (name == "gravity") {
        cfg.alignment.gravity = value;  // alignment's stillness test uses the same g
      }
      return true;
    }
  }
  return false;
}

}  // namespace quad_estimation
