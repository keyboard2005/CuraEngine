// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "infill/TrussFill.h"

#include "geometry/OpenPolyline.h"
#include "geometry/PointMatrix.h"
#include "geometry/Shape.h"
#include "utils/AABB.h"

namespace cura
{

void TrussFill::generateTrussInfill(OpenLinesSet& result_lines, coord_t line_distance, const Shape& in_outline, double fill_angle, bool mirror)
{
    if (in_outline.empty() || line_distance <= 0)
    {
        return;
    }

    // Work in a rotated frame so the truss rows are always axis-aligned. We undo
    // the rotation on the resulting points at the end.
    const PointMatrix rotation_matrix(fill_angle);
    Shape outline = in_outline; // copy, we rotate this one
    outline.applyMatrix(rotation_matrix);

    const AABB boundary(outline);
    const coord_t min_x = boundary.min_.X;
    const coord_t max_x = boundary.max_.X;
    const coord_t min_y = boundary.min_.Y;
    const coord_t max_y = boundary.max_.Y;

    if (max_x <= min_x || max_y <= min_y)
    {
        return;
    }

    // Build one continuous triangular wave that spans the whole bounding box.
    // The apexes sit on the top/bottom of the bounding box; clipping to the real
    // contour (below) cuts every segment exactly at the walls, so the triangle
    // tips end up touching the walls while holes and concavities are skipped.
    //
    // We deliberately do NOT connect apexes across gaps ourselves: instead we let
    // Shape::intersection() handle it, exactly like the other infill patterns do.
    // That is what keeps the pattern out of interior cut-outs.
    OpenPolyline wave;
    bool up = ! mirror; // mirror flips the starting side so layers can cross-brace

    coord_t x = min_x;
    while (true)
    {
        wave.emplace_back(x, up ? max_y : min_y);
        up = ! up;
        if (x >= max_x)
        {
            break;
        }
        x += line_distance;
        if (x > max_x)
        {
            x = max_x; // make sure the last apex lands exactly on the far edge
        }
    }

    OpenLinesSet wave_set;
    wave_set.push_back(std::move(wave));

    // Clip the triangular wave to the actual fill area. Segments inside holes or
    // outside the outline are removed; the remaining pieces touch the walls.
    // restitch=true rejoins the tiny pieces Clipper may split a segment into,
    // but the (large) gaps across holes stay separate.
    const OpenLinesSet clipped = outline.intersection(wave_set, /*restitch=*/true);

    // Undo the rotation to bring the lines back into the original coordinate frame.
    for (const OpenPolyline& segment : clipped)
    {
        OpenPolyline unrotated;
        unrotated.reserve(segment.size());
        for (const Point2LL& point : segment)
        {
            unrotated.push_back(rotation_matrix.unapply(point));
        }
        result_lines.push_back(std::move(unrotated));
    }
}

} // namespace cura
