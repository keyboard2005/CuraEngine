// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "infill/TrussFill.h"

#include "geometry/OpenPolyline.h"
#include "geometry/PointMatrix.h"
#include "geometry/Shape.h"
#include "utils/AABB.h"

namespace cura
{

namespace
{
//! Same scanline-index helper the built-in linear/zig-zag infill uses, so the
//! truss columns land on the exact same absolute grid (and therefore the same
//! position on every layer).
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

    // Work in a rotated frame so the truss rows are axis-aligned, then store the
    // result back in world coordinates so any layer can clip it directly.
    const PointMatrix rotation_matrix(fill_angle);
    Shape outline = template_outline; // copy, we rotate this one
    outline.applyMatrix(rotation_matrix);

    // Normalise the shift into [0, line_distance), exactly like
    // Infill::generateLinearBasedInfill does, so the column grid is identical.
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

    // One continuous, regular triangular wave spanning the whole (envelope)
    // bounding box. The apexes alternate between the flat top (max_y) and flat
    // bottom (min_y), so every triangle is identical and the wave has a single
    // constant angle. The columns are anchored to the absolute grid
    // (col * line_distance + shift) and the up/down side comes from the absolute
    // column index, so the geometry is fixed in space. Building this once and
    // clipping it on every layer is what makes the angle and position line up
    // perfectly from layer to layer; only the clipped lengths differ.
    const coord_t min_y = boundary.min_.Y;
    const coord_t max_y = boundary.max_.Y;
    const int col_min = computeScanSegmentIdx(boundary.min_.X - shift, line_distance);
    const int col_max = computeScanSegmentIdx(boundary.max_.X - shift, line_distance) + 1;

    OpenPolyline wave;
    wave.reserve(static_cast<size_t>(col_max - col_min + 1));
    for (int col = col_min; col <= col_max; ++col)
    {
        const coord_t x = static_cast<coord_t>(col) * line_distance + shift;
        const bool up = ((((col % 2) + 2) % 2) == 0) != mirror;
        // Store the template directly in world coordinates.
        wave.push_back(rotation_matrix.unapply(Point2LL(x, up ? max_y : min_y)));
    }
    if (wave.size() >= 2)
    {
        template_lines.push_back(std::move(wave));
    }

    return template_lines;
}

void TrussFill::clipToOutline(OpenLinesSet& result_lines, const OpenLinesSet& template_lines, const Shape& in_outline)
{
    if (template_lines.empty() || in_outline.empty())
    {
        return;
    }

    // Clip the shared wave to this layer's real contour. Segments inside holes or
    // outside the outline are removed; the remaining pieces touch the walls.
    // restitch=true rejoins the tiny pieces Clipper may split a segment into,
    // but the (large) gaps across holes stay separate. Because every layer clips
    // the very same template, the kept pieces share the exact same angle and
    // position across layers - only their lengths differ.
    const OpenLinesSet clipped = in_outline.intersection(template_lines, /*restitch=*/true);
    for (const OpenPolyline& segment : clipped)
    {
        if (segment.size() >= 2)
        {
            result_lines.push_back(segment);
        }
    }
}

void TrussFill::generateTrussInfill(OpenLinesSet& result_lines, coord_t line_distance, const Shape& in_outline, double fill_angle, coord_t pattern_shift, bool mirror)
{
    // Per-layer fallback (NOT layer-aligned): build a template sized to this
    // outline and clip it back to the same outline. Used when no shared
    // cross-layer template is available.
    const OpenLinesSet template_lines = generateTemplate(line_distance, in_outline, fill_angle, pattern_shift, mirror);
    clipToOutline(result_lines, template_lines, in_outline);
}

} // namespace cura
