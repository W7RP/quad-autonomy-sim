// The estimator's tunable parameters, listed once. The ROS node declares a ROS
// parameter per entry; the replay tool reads the same names from the node's
// YAML file. Units in names follow the YAML (degrees, seconds); set/get convert.
#pragma once

#include <span>
#include <string_view>

#include "quad_estimation/pipeline.hpp"

namespace quad_estimation
{

struct ParamSpec
{
  const char * name;
  bool is_int;
  double (*get)(const PipelineConfig &);
  void (*set)(PipelineConfig &, double);
};

[[nodiscard]] std::span<const ParamSpec> pipeline_params() noexcept;

// Returns false if `name` is not an estimator parameter.
bool set_pipeline_param(PipelineConfig & cfg, std::string_view name, double value) noexcept;

}  // namespace quad_estimation
