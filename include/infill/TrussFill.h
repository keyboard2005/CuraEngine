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
 * \brief Triangular saw-tooth ("truss") infill pattern.
 *
 * A single full-height zig-zag row that runs along the fill direction (a frame
 * rotated by \p fill_angle): one triangle apex per scanline column, alternating
 * between the top and the bottom of the region so the struts form a continuous
 * /\/\/\ saw-tooth that fills the whole height with very little material.
 *
 * Layer alignment - the important part:
 * The whole pattern is built ONCE from a single template outline (the layer with
 * the largest cross-section, see \ref generateTemplate) and stored in world
 * coordinates. Every individual layer then merely clips that one shared template
 * to its own contour (\ref clipToOutline). Because all layers share the exact
 * same apex geometry, the triangles line up perfectly from layer to layer; the
 * template layer prints the complete, fully-connected saw-tooth and every other
 * (smaller) layer prints a subset of it. The triangles of the smaller layers do
 * not need to stay fully connected - only the template layer does.
 *
 * The pattern is emitted as individual line segments (like the "lines" infill,
 * SpaceFillType::Lines) instead of one long connected polyline, so the
 * downstream path ordering - and thus the print start/end - is stable from layer
 * to layer. Adjacent struts share their apex point, so the optimiser still
 * chains them with (near) zero travel.
 */
class TrussFill
{
public:
    /*!
     * \brief Build the complete saw-tooth template from a single outline.
     *
     * The triangle apex columns are anchored to the same absolute grid the
     * built-in zig-zag uses for its vertical lines - <tt>col * line_distance +
     * pattern_shift</tt> - and the apex Y is locked to the top/bottom of \p
     * template_outline at each column, so the template spans the full height of
     * that outline. The result is returned in world coordinates (already
     * un-rotated) and is NOT clipped, so callers can clip it per layer.
     *
     * \param line_distance   Spacing between consecutive apexes (µm). The
     *                        triangle base equals twice this value, so the
     *                        triangle width scales with the infill density.
     * \param template_outline The outline to build the full template from
     *                        (typically the largest layer's infill region).
     * \param fill_angle      Orientation of the truss, in degrees.
     * \param pattern_shift   The scanline shift (infill-origin offset + global
     *                        shift), exactly as the zig-zag infill uses it, so
     *                        the truss aligns with the same absolute grid.
     * \param mirror          When true, flip the starting side.
     * \return The full saw-tooth as open apex polylines in world coordinates.
     */
    static OpenLinesSet generateTemplate(coord_t line_distance, const Shape& template_outline, double fill_angle, coord_t pattern_shift, bool mirror);

    /*!
     * \brief Clip a pre-built template to one layer's contour and emit the lines.
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
     * available (e.g. when the infill is generated without a mesh context).
     */
    static void generateTrussInfill(OpenLinesSet& result_lines, coord_t line_distance, const Shape& in_outline, double fill_angle, coord_t pattern_shift, bool mirror);
};

} // namespace cura

#endif // INFILL_TRUSSFILL_H
