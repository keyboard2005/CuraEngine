// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef INFILL_TRUSSFILL_H
#define INFILL_TRUSSFILL_H

#include "geometry/OpenLinesSet.h"
#include "utils/Coord_t.h"

namespace cura
{

class Shape;

/*!
 * \brief Triangular truss infill pattern.
 *
 * Generates a continuous zig-zag where each apex lands on the wall of the fill
 * area, forming a row of triangles. This is the lightweight-yet-stiff lattice
 * commonly used in concrete / large-format printing: little material, high
 * structural strength.
 *
 * The algorithm scans vertical columns (in a frame rotated by \p fill_angle).
 * For every column it takes the top/bottom boundary of the region and places an
 * apex there, alternating between top and bottom. Consecutive apexes are joined
 * by sloped segments, so the triangle tips naturally touch the walls. Columns
 * that fall outside the region (gaps / holes) break the polyline.
 */
class TrussFill
{
public:
    /*!
     * \brief Generate truss infill lines within the given outline.
     *
     * \param result_lines   Output: the truss polylines (open line segments).
     * \param line_distance  Horizontal spacing between consecutive apexes (µm).
     *                       The triangle base equals twice this value.
     * \param in_outline     The fill area (already offset by wall count etc.).
     * \param fill_angle     Orientation of the truss rows, in degrees.
     * \param mirror         When true, start from the opposite side so that the
     *                       pattern alternates between layers for cross-bracing.
     */
    static void generateTrussInfill(OpenLinesSet& result_lines, coord_t line_distance, const Shape& in_outline, double fill_angle, bool mirror);
};

} // namespace cura

#endif // INFILL_TRUSSFILL_H
