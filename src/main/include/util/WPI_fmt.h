#pragma once

#include <fmt/format.h>
#include <frc/geometry/Pose2d.h>

template <> struct fmt::formatter<frc::Pose2d>: formatter<string_view> {
  // parse is inherited from formatter<string_view>.

  auto format(frc::Pose2d p, format_context& ctx) const
    -> format_context::iterator;
};

auto fmt::formatter<frc::Pose2d>::format(frc::Pose2d c, format_context& ctx) const
    -> format_context::iterator {

  return fmt::format_to( ctx.out(), "({:.3},{:.3},{:.3})", c.X(), c.Y(), c.Rotation().Degrees() );
}