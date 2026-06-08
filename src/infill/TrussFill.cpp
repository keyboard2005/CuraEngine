// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "infill/TrussFill.h"

#include <algorithm>
#include <map>
#include <utility>
#include <vector>

#include "geometry/OpenPolyline.h"
#include "geometry/PointMatrix.h"
#include "geometry/Shape.h"
#include "utils/AABB.h"

namespace cura
{

namespace
{
//! Same scanline-index helper the built-in linear/zig-zag infill uses, so the
//! truss lands on the exact same absolute grid (and therefore the same position
//! on every layer).
int computeScanSegmentIdx(int x, int line_width)
{
    if (x < 0)
    {
        return (x + 1) / line_width - 1;
    }
    return x / line_width;
}
} // namespace

void TrussFill::generateTrussInfill(OpenLinesSet& result_lines, coord_t line_distance, const Shape& in_outline, double fill_angle, coord_t pattern_shift, bool mirror)
{
    if (in_outline.empty() || line_distance <= 0)
    {
        return;
    }

    // Work in a rotated frame so the truss runs along X (just like zig-zag does).
    const PointMatrix rotation_matrix(fill_angle);
    Shape outline = in_outline; // copy, we rotate this one
    outline.applyMatrix(rotation_matrix);

    // Normalise the shift into [0, line_distance), exactly like
    // Infill::generateLinearBasedInfill does, so the scanline grid is identical.
    coord_t shift = pattern_shift;
    if (shift < 0)
    {
        shift = line_distance - (-shift) % line_distance;
    }
    else
    {
        shift = shift % line_distance;
    }

    const AABB boundary(outline);
    if (boundary.max_.X <= boundary.min_.X || boundary.max_.Y <= boundary.min_.Y)
    {
        return;
    }

    // Scanline (apex column) positions: scanline_idx * line_distance + shift, the
    // very same formula as the zig-zag infill. We keep the absolute scanline index
    // so the up/down side of each apex stays consistent across layers and gaps.
    const int scanline_min_idx = computeScanSegmentIdx(boundary.min_.X - shift, line_distance);
    const int scanline_max_idx = computeScanSegmentIdx(boundary.max_.X - shift, line_distance) + 1;

    std::vector<std::pair<coord_t, int>> columns; // (x, scanline_idx)
    columns.reserve(static_cast<size_t>(scanline_max_idx - scanline_min_idx + 1));
    for (int idx = scanline_min_idx; idx <= scanline_max_idx; ++idx)
    {
        columns.emplace_back(idx * line_distance + shift, idx);
    }

    // Probe the local vertical extent of the region at every column in one go.
    OpenLinesSet scanlines;
    scanlines.reserve(columns.size());
    for (const auto& [x, idx] : columns)
    {
        OpenPolyline scanline;
        scanline.emplace_back(x, boundary.min_.Y - 1);
        scanline.emplace_back(x, boundary.max_.Y + 1);
        scanlines.push_back(std::move(scanline));
    }
    const OpenLinesSet probed = outline.intersection(scanlines, /*restitch=*/false);

    // For each column keep the overall (lowest, highest) Y of the material there.
    std::map<coord_t, std::pair<coord_t, coord_t>> extents; // x -> (y_low, y_high)
    for (const OpenPolyline& segment : probed)
    {
        for (const Point2LL& point : segment)
        {
            auto it = extents.find(point.X);
            if (it == extents.end())
            {
                extents.emplace(point.X, std::make_pair(point.Y, point.Y));
            }
            else
            {
                it->second.first = std::min(it->second.first, point.Y);
                it->second.second = std::max(it->second.second, point.Y);
            }
        }
    }

    // Build the zig-zag: one apex per column, alternating between the top and the
    // bottom of that column's extent. The side is derived from the absolute
    // scanline index, so it is identical on every layer. Columns with no material
    // break the polyline.
    OpenLinesSet pattern;
    OpenPolyline wave;
    for (const auto& [x, idx] : columns)
    {
        const auto it = extents.find(x);
        if (it == extents.end())
        {
            if (wave.size() >= 2)
            {
                pattern.push_back(std::move(wave));
            }
            wave.clear();
            continue;
        }
        const bool up = ((((idx % 2) + 2) % 2) == 0) != mirror;
        const coord_t y = up ? it->second.second : it->second.first;
        wave.emplace_back(x, y);
    }
    if (wave.size() >= 2)
    {
        pattern.push_back(std::move(wave));
    }

    // Clip the zig-zag to the real contour: trims the diagonals at the walls and
    // removes any parts that would bridge across an interior cut-out.
    const OpenLinesSet clipped = outline.intersection(pattern, /*restitch=*/true);

    // Emit the result as individual line segments, exactly like the "lines" infill
    // (SpaceFillType::Lines) does - rather than one long connected polyline. This
    // lets the path optimiser order them with its boustrophedon strategy, so the
    // print start/end no longer flips between layers (a single long polyline would
    // be entered from whichever of its two ends is nearest, alternating per layer).
    // Adjacent diagonals share their apex point, so the optimiser still chains them
    // with (near) zero travel, keeping the extrusion continuous.
    for (const OpenPolyline& segment : clipped)
    {
        bool has_prev = false;
        Point2LL prev;
        for (const Point2LL& point : segment)
        {
            const Point2LL current = rotation_matrix.unapply(point);
            if (has_prev && current != prev)
            {
                OpenPolyline line;
                line.push_back(prev);
                line.push_back(current);
                result_lines.push_back(std::move(line));
            }
            prev = current;
            has_prev = true;
        }
    }
}

} // namespace cura
