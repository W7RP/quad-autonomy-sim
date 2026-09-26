// Topic names for the PX4 uXRCE-DDS bridge.
//
// Since PX4 v1.16 the client appends "_v<N>" to a topic when its message has a
// non-zero MESSAGE_VERSION (e.g. /fmu/out/vehicle_status_v1). Building names from
// the px4_msgs constant keeps the node in lockstep with whatever px4_msgs it was
// compiled against, instead of silently subscribing to a topic nobody publishes.
#pragma once

#include <string>

namespace quad_offboard
{

template<typename MsgT>
[[nodiscard]] std::string px4_topic(const std::string & ns, const std::string & base)
{
  std::string name = ns + base;
  if constexpr (requires {MsgT::MESSAGE_VERSION;}) {
    if (MsgT::MESSAGE_VERSION != 0U) {
      name += "_v" + std::to_string(MsgT::MESSAGE_VERSION);
    }
  }
  return name;
}

}  // namespace quad_offboard
