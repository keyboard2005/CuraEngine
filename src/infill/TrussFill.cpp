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

OpenLinesSet TrussFill::generateTemplate(coord_t line_distance, const Shape& template_outline, double fill_angle, coord_t pattern_shift, bool mirror)
{
    OpenLinesSet template_lines;
    if (template_outline.empty() || line_distance <= 0)
    {
        return template_lines;
    }

    // Work in a rotated frame so the truss runs along X (just like zig-zag does).
    const PointMatrix rotation_matrix(fill_angle);
    Shape outline = template_outline; // copy, we rotate this one
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
        return template_lines;
    }

    // --- Single full-height row -------------------------------------------------
    // One zig-zag row that spans the full height of the template region. The apex
    // X-columns are anchored to the absolute grid (col * line_distance + shift),
    // and the apex Y is locked to the local top/bottom of the template contour at
    // each column, so a single row always fills the whole height. The template
    // contour is the cross-layer envelope, so each column's extent is the furthest
    // any layer reaches there. Because this is built only once and then re-used on
    // every layer, the apex geometry is identical from layer to layer.

    const int col_min = computeScanSegmentIdx(boundary.min_.X - shift, line_distance);
    const int col_max = computeScanSegmentIdx(boundary.max_.X - shift, line_distance) + 1;

    std::vector<std::pair<coord_t, int>> columns; // (x, absolute column index)
    columns.reserve(static_cast<size_t>(col_max - col_min + 1));
    for (int col = col_min; col <= col_max; ++col)
    {
        columns.emplace_back(static_cast<coord_t>(col) * line_distance + shift, col);
    }

    // Probe the local vertical extent of the region at every column in one go.
    OpenLinesSet scanlines;
    scanlines.reserve(columns.size());
    for (const auto& [x, col] : columns)
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

    // Build the saw-tooth: one apex per column, alternating between the top and
    // the bottom of that column's local extent. The side comes from the absolute
    // column index so it stays consistent. A column with no material breaks the
    // run. The apex polylines are kept un-clipped: each layer (including the
    // template layer itself) clips them to its own contour later.
    std::vector<Point2LL> run; // apexes of the current uninterrupted run
    const auto flush_run = [&]()
    {
        // Lock the first/last strut to the interior rhythm: a boundary column only
        // ever has a partial local height, so snap the outermost apex onto the same
        // Y as the apex two columns inward (its same-side neighbour). That keeps the
        // end strut parallel to the adjacent interior one; the contour clip then
        // only changes its length.
        if (run.size() >= 3)
        {
            run.front().Y = run[2].Y;
            run.back().Y = run[run.size() - 3].Y;
        }
        if (run.size() >= 2)
        {
            OpenPolyline wave;
            for (const Point2LL& apex : run)
            {
                // Store the template directly in world coordinates so that any
                // layer can clip it without having to know the fill angle.
                wave.push_back(rotation_matrix.unapply(apex));
            }
            template_lines.push_back(std::move(wave));
        }
        run.clear();
    };
    for (const auto& [x, col] : columns)
    {
        const auto it = extents.find(x);
        if (it == extents.end())
        {
            flush_run();
            continue;
        }
        const bool up = ((((col % 2) + 2) % 2) == 0) != mirror;
        const coord_t y = up ? it->second.second : it->second.first;
        run.emplace_back(x, y);
    }
    flush_run();

    return template_lines;
}

void TrussFill::clipToOutline(OpenLinesSet& result_lines, const OpenLinesSet& template_lines, const Shape& in_outline)
{
    if (template_lines.empty() || in_outline.empty())
    {
        return;
    }

    // Clip the shared saw-tooth to this layer's real contour: trims the struts at
    // the walls and removes any parts that would bridge across an interior cut-out.
    const OpenLinesSet clipped = in_outline.intersection(template_lines, /*restitch=*/true);

    // Emit the result as individual line segments, exactly like the "lines" infill
    // (SpaceFillType::Lines) does - rather than one long connected polyline. This
    // lets the path optimiser order them with its boustrophedon strategy, so the
    // print start/end no longer flips between layers. Adjacent diagonals share
    // their apex point, so the optimiser still chains them with (near) zero travel.
    for (const OpenPolyline& segment : clipped)
    {
        bool has_prev = false;
        Point2LL prev;
        for (const Point2LL& point : segment)
        {
            if (has_prev && point != prev)
            {
                OpenPolyline line;
                line.push_back(prev);
                line.push_back(point);
                result_lines.push_back(std::move(line));
            }
            prev = point;
            has_prev = true;
        }
    }
}

void TrussFill::generateTrussInfill(OpenLinesSet& result_lines, coord_t line_distance, const Shape& in_outline, double fill_angle, coord_t pattern_shift, bool mirror)
{
    // Per-layer fallback: build a template from this layer alone and clip it to
    // the same outline. Used when no shared cross-layer template is available.
    const OpenLinesSet template_lines = generateTemplate(line_distance, in_outline, fill_angle, pattern_shift, mirror);
    clipToOutline(result_lines, template_lines, in_outline);
}

} // namespace cura
