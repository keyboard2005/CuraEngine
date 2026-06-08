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
 * A single zig-zag that runs along the fill direction (a frame rotated by
 * \p fill_angle), with the triangle tips landing on the walls. This is the
 * lightweight-yet-stiff structure commonly used in concrete / large-format
 * printing: little material, high strength, continuous extrusion.
 *
 * The scanline grid is computed *exactly* like the built-in zig-zag infill:
 * apexes sit at <tt>scanline_index * line_distance + pattern_shift</tt> in the
 * rotated frame, with \p pattern_shift carrying the infill-origin offset. Using
 * the same absolute grid as zig-zag means the truss lands on identical positions
 * on every layer, so the webs stack vertically just like zig-zag does.
 *
 * The amplitude of each column follows the local height of the region at that
 * column, and the finished zig-zag is clipped to the real contour, so interior
 * cut-outs are not bridged.
 *
 * The pattern is emitted as individual line segments (like the "lines" infill,
 * SpaceFillType::Lines) instead of one long connected polyline. This makes the
 * downstream path ordering behave exactly like "lines": the print start/end is
 * stable from layer to layer, instead of flipping between the two ends of a
 * single long path. Adjacent diagonals share their apex point, so the optimiser
 * still chains them with (near) zero travel.
 */
class TrussFill
{
public:
    /*!
     * \brief Generate truss infill lines within the given outline.
     *
     * \param result_lines   Output: the truss polylines (open line segments).
     * \param line_distance  Spacing between consecutive apexes (µm). The triangle
     *                       base equals twice this value.
     * \param in_outline     The fill area (already offset by wall count etc.).
     * \param fill_angle     Orientation of the truss, in degrees.
     * \param pattern_shift  The scanline shift (infill-origin offset + global
     *                       shift), exactly as passed to the zig-zag infill, so
     *                       the truss aligns with the same absolute grid.
     * \param mirror         When true, flip the starting side so the pattern can
     *                       alternate between layers for cross-bracing.
     */
    static void generateTrussInfill(OpenLinesSet& result_lines, coord_t line_distance, const Shape& in_outline, double fill_angle, coord_t pattern_shift, bool mirror);
};

} // namespace cura

#endif // INFILL_TRUSSFILL_H
