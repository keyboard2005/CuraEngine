// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "infill/SpiralFill.h"

#include <cmath>
#include <numbers>
#include <vector>

#include "geometry/OpenPolyline.h"
#include "geometry/Shape.h"
#include "utils/AABB.h"

namespace cura
{

void SpiralFill::generateSpiralInfill(OpenLinesSet& result_lines, coord_t line_distance, const Shape& in_outline)
{
    if (in_outline.empty() || line_distance <= 0)
    {
        return;
    }

    // --- Bounding box & center ---
    const AABB aabb(in_outline);

    // Anchor the spiral center to an absolute grid (multiples of line_distance)
    // rather than the per-layer bounding-box center. This keeps the spiral at the
    // exact same position on every layer - even when the cross-section changes -
    // so the arms stack into continuous vertical webs, like the built-in infills.
    const double raw_cx = static_cast<double>(aabb.min_.X + aabb.max_.X) / 2.0;
    const double raw_cy = static_cast<double>(aabb.min_.Y + aabb.max_.Y) / 2.0;
    const double cx = std::round(raw_cx / static_cast<double>(line_distance)) * static_cast<double>(line_distance);
    const double cy = std::round(raw_cy / static_cast<double>(line_distance)) * static_cast<double>(line_distance);

    // Maximum radius: distance from the (snapped) center to the farthest corner,
    // with a 5% margin so the spiral always reaches every corner.
    const double max_radius = std::max({ std::hypot(static_cast<double>(aabb.min_.X) - cx, static_cast<double>(aabb.min_.Y) - cy),
                                         std::hypot(static_cast<double>(aabb.max_.X) - cx, static_cast<double>(aabb.min_.Y) - cy),
                                         std::hypot(static_cast<double>(aabb.min_.X) - cx, static_cast<double>(aabb.max_.Y) - cy),
                                         std::hypot(static_cast<double>(aabb.max_.X) - cx, static_cast<double>(aabb.max_.Y) - cy) })
                            * 1.05;

    if (max_radius < 1.0)
    {
        return;
    }

    // --- Archimedean spiral parameters ---
    // r(θ) = b * θ,  b = line_distance / (2π)
    // When r = max_radius: θ_max = 2π * max_radius / line_distance
    const double b          = static_cast<double>(line_distance) / (2.0 * std::numbers::pi);
    const double theta_max  = 2.0 * std::numbers::pi * max_radius / static_cast<double>(line_distance);

    // Number of sample points: ~360 per full revolution for smooth curves.
    const int num_revolutions = static_cast<int>(std::ceil(theta_max / (2.0 * std::numbers::pi)));
    const int num_points      = std::max(num_revolutions * 360 + 2, 3);

    // --- Build spiral polyline ---
    OpenPolyline spiral;
    spiral.reserve(static_cast<size_t>(num_points));

    for (int i = 0; i < num_points; ++i)
    {
        const double theta = theta_max * static_cast<double>(i) / static_cast<double>(num_points - 1);
        const double r     = b * theta;
        spiral.emplace_back(
            static_cast<coord_t>(std::round(cx + r * std::cos(theta))),
            static_cast<coord_t>(std::round(cy + r * std::sin(theta)))
        );
    }

    // --- Clip to fill area ---
    OpenLinesSet spiral_set;
    spiral_set.push_back(std::move(spiral));

    // Shape::intersection clips open polylines against the polygon boundary.
    // restitch=false keeps each clipped segment as its own polyline.
    const OpenLinesSet clipped = in_outline.intersection(spiral_set, /*restitch=*/false);
    result_lines.push_back(clipped);
}

} // namespace cura
