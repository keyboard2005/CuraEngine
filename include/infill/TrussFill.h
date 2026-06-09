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
 * Generates a continuous, regular zig-zag whose apexes form a row of triangles.
 * This is the lightweight-yet-stiff lattice commonly used in concrete /
 * large-format printing: little material, high structural strength.
 *
 * Layer alignment is the primary goal: every layer must print the SAME triangles
 * at the SAME angle and the SAME position. Only the line lengths may differ from
 * layer to layer (because each layer clips the pattern to its own contour).
 *
 * To guarantee that, the whole pattern is built ONCE (\ref generateTemplate) in
 * world coordinates and stored; every layer then merely clips that single shared
 * template to its own contour (\ref clipToOutline). The template is a regular
 * triangular wave: the apexes alternate between a flat top and a flat bottom
 * (the top/bottom of the cross-layer envelope), and the columns are anchored to
 * the absolute infill grid - <tt>col * line_distance + pattern_shift</tt>. A flat
 * top/bottom keeps every triangle identical (a single, constant angle), and
 * because the template is shared, the angle and position are identical on every
 * layer. Clipping cuts each leg at the walls, which is what keeps the triangles
 * sitting in sensible positions even on complex models, and skips interior
 * cut-outs.
 */
class TrussFill
{
public:
    /*!
     * \brief Build the complete regular triangular-wave template.
     *
     * The columns are anchored to the absolute grid the built-in zig-zag uses
     * (<tt>col * line_distance + pattern_shift</tt>) and the apexes alternate
     * between the flat top and flat bottom of \p template_outline's bounding box,
     * so the whole wave has a single constant angle. The result is returned in
     * world coordinates (already un-rotated) and is NOT clipped, so callers can
     * clip it per layer.
     *
     * \param line_distance   Horizontal spacing between consecutive apexes (µm).
     *                        The triangle base equals twice this value, so the
     *                        triangle width scales with the infill density.
     * \param template_outline The outline to size the template from (typically
     *                        the cross-layer envelope of every layer's infill
     *                        area, so the template covers every layer).
     * \param fill_angle      Orientation of the truss rows, in degrees.
     * \param pattern_shift   The scanline shift (infill-origin offset + global
     *                        shift), exactly as the zig-zag infill uses it.
     * \param mirror          When true, flip the starting side.
     * \return The regular triangular wave as an open polyline in world coords.
     */
    static OpenLinesSet generateTemplate(coord_t line_distance, const Shape& template_outline, double fill_angle, coord_t pattern_shift, bool mirror);

    /*!
     * \brief Clip a pre-built template to one layer's contour and emit the lines.
     *
     * Because every layer clips the exact same \p template_lines, the resulting
     * lines have the same angle and position on every layer; only their lengths
     * differ.
     *
     * \param result_lines    Output: the truss line segments for this layer.
     * \param template_lines  The shared template from \ref generateTemplate
     *                        (world coordinates).
     * \param in_outline      This layer's fill area (already offset by wall
     *                        count etc.).
     */
    static void clipToOutline(OpenLinesSet& result_lines, const OpenLinesSet& template_lines, const Shape& in_outline);

    /*!
     * \brief Convenience: build a template from \p in_outline and clip it back
     *        to the same outline.
     *
     * Used as a per-layer fallback when no shared cross-layer template is
     * available (e.g. when the infill is generated without a mesh context). Note
     * that this fallback is NOT layer-aligned, since it sizes the wave from the
     * single outline it is given.
     */
    static void generateTrussInfill(OpenLinesSet& result_lines, coord_t line_distance, const Shape& in_outline, double fill_angle, coord_t pattern_shift, bool mirror);
};

} // namespace cura

#endif // INFILL_TRUSSFILL_H
